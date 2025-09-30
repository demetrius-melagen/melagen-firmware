/*
RADFET Data Collection:
- Enabling sensors through tca9539 I2C to i/o converter
- Polling using ADC Channels
- Saving sample packets to internal flash (circular buffer)
- Put task to sleep per sample rate
*/

#include <gs/a3200/a3200.h>
#include <gs/a3200/adc_channels.h>
#include <gs/a3200/led.h>
#include <gs/util/time.h>
#include <gs/util/thread.h>
#include <gs/util/types.h>
#include <gs/util/vmem.h>
#include <gs/util/log.h>
#include <wdt.h>
#include <gs/util/drivers/i2c/master.h>
#include <inttypes.h>
#include <string.h>
#include <stddef.h>
#include "radfet.h"
#include <gs/thirdparty/flash/spn_fl512s.h>
#include <gs/embed/drivers/flash/mcu_flash.h>

// ===== Ring/packet sizing (relies on radfet.h definitions) =====
#define RING_CAP_PACKETS  (RADFET_FLASH_SIZE / PKT_SIZE)
#define RING_CAP_BYTES    (RING_CAP_PACKETS * PKT_SIZE)

// (Optional) runtime guard in case the ring would be zero-sized
static inline int ring_capacity_ok(void) {
    return (RING_CAP_PACKETS > 0);
}

// ADC RADFET pin configuration
static const uint8_t radfet_channels[NUM_RADFET] = {
    7,  // D1 → AD7
    6,  // D2 → AD6
    5,  // D3 → AD5
    4,  // D4 → AD4
    0   // D5 → AD0
};

// Global metadata (persisted in flash via load/save helpers)
radfet_metadata_t radfet_metadata = {
    .flash_write_offset = 0,
    .samples_saved      = 0,
    .sample_rate_ms     = 60000,  // default sample rate
    .crc16              = 0,
};

// ===== TCA9539 helpers =====
static inline gs_error_t write_tca9539_register(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg, value};
    return gs_i2c_master_transaction(0, TCA9539_I2C_ADDR, tx, 2, NULL, 0, I2C_TIMEOUT_MS);
}

static inline gs_error_t read_tca9539_register(uint8_t reg, uint8_t *value) {
    return gs_i2c_master_transaction(0, TCA9539_I2C_ADDR, &reg, 1, value, 1, I2C_TIMEOUT_MS);
}

// Write both OUT ports (preserves first error if both fail)
static gs_error_t update_io_expander(uint8_t port0, uint8_t port1) {
    gs_error_t err0 = write_tca9539_register(TCA9539_OUT_PORT0, port0);
    gs_error_t err1 = write_tca9539_register(TCA9539_OUT_PORT1, port1);
    return (err0 != GS_OK) ? err0 : err1;
}

static gs_error_t tca9539_config(void) {
    gs_error_t err;
    err = write_tca9539_register(TCA9539_OUT_PORT0, 0x00);
    if (err != GS_OK) { log_error("Failed to write OUT_PORT0: %s", gs_error_string(err)); return err; }

    err = write_tca9539_register(TCA9539_OUT_PORT1, 0x00);
    if (err != GS_OK) { log_error("Failed to write OUT_PORT1: %s", gs_error_string(err)); return err; }

    err = write_tca9539_register(TCA9539_CFG_PORT0, 0x00);
    if (err != GS_OK) { log_error("Failed to write CFG_PORT0: %s", gs_error_string(err)); return err; }

    err = write_tca9539_register(TCA9539_CFG_PORT1, 0x00);
    if (err != GS_OK) { log_error("Failed to write CFG_PORT1: %s", gs_error_string(err)); return err; }

    uint8_t cfg0, cfg1, out0, out1;

    err = read_tca9539_register(TCA9539_CFG_PORT0, &cfg0);
    if (err != GS_OK) { log_error("Failed to read CFG_PORT0: %s", gs_error_string(err)); return err; }
    log_info("CFG_PORT0 = 0x%02X → %s", cfg0, (cfg0 == 0x00) ? "OK (all outputs)" : "NOT OK");

    err = read_tca9539_register(TCA9539_CFG_PORT1, &cfg1);
    if (err != GS_OK) { log_error("Failed to read CFG_PORT1: %s", gs_error_string(err)); return err; }
    log_info("CFG_PORT1 = 0x%02X → %s", cfg1, (cfg1 == 0x00) ? "OK (all outputs)" : "NOT OK");

    err = read_tca9539_register(TCA9539_OUT_PORT0, &out0);
    if (err != GS_OK) { log_error("Failed to read OUT_PORT0: %s", gs_error_string(err)); return err; }
    log_info("OUT_PORT0 = 0x%02X", out0);

    err = read_tca9539_register(TCA9539_OUT_PORT1, &out1);
    if (err != GS_OK) { log_error("Failed to read OUT_PORT1: %s", gs_error_string(err)); return err; }
    log_info("OUT_PORT1 = 0x%02X", out1);

    return GS_OK;
}

// ===== CRC16-CCITT (0x1021, init 0xFFFF, no xorout) =====
uint16_t crc16_ccitt(const void *data, size_t length) {
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Metadata CRC over all fields except trailing crc16
static uint16_t calc_metadata_crc(const radfet_metadata_t *meta) {
    return crc16_ccitt(meta, sizeof(radfet_metadata_t) - sizeof(meta->crc16));
}

// ===== Metadata load/save =====
bool radfet_load_metadata(void) {
    radfet_metadata_t meta = radfet_metadata;
    const void *addr = (void *)RADFET_METADATA_ADDR;

    log_info("Reading metadata from address: 0x%08lx", (uint32_t)addr);
    gs_error_t err = gs_mcu_flash_read_data(&meta, addr, METADATA_PKT_SIZE);
    if (err != GS_OK) {
        log_error("Failed to read metadata from flash (err = %d)", err);
        return false;
    }

    uint16_t expected = meta.crc16;
    meta.crc16 = 0;
    uint16_t actual = calc_metadata_crc(&meta);

    log_info("Read metadata: offset=%" PRIu32 ", count=%" PRIu32 ", rate=%" PRIu32
             ", expected_crc=0x%04X, actual_crc=0x%04X",
             meta.flash_write_offset, meta.samples_saved, meta.sample_rate_ms, expected, actual);

    bool valid = (expected == actual) &&
                 (meta.flash_write_offset < RING_CAP_BYTES) &&
                 ((meta.flash_write_offset % PKT_SIZE) == 0);

    if (!valid) return false;

    // Normalize to ring capacity just in case sizes changed across builds
    meta.flash_write_offset %= RING_CAP_BYTES;

    radfet_metadata = meta;
    return true;
}

gs_error_t radfet_save_metadata(void) {
    radfet_metadata_t meta = radfet_metadata;
    // sanitize before saving
    meta.flash_write_offset %= RING_CAP_BYTES;
    meta.flash_write_offset -= (meta.flash_write_offset % PKT_SIZE);

    meta.crc16 = 0;
    meta.crc16 = calc_metadata_crc(&meta);

    return gs_mcu_flash_write_data(RADFET_METADATA_ADDR, (uint8_t *)&meta, METADATA_PKT_SIZE);
}

// ===== ADC sampling helpers =====
static gs_error_t radfet_read_all(radfet_sample_t *sample, int r) {
    int16_t all_adc_values[GS_A3200_ADC_NCHANS] = {0};
    gs_error_t err = gs_a3200_adc_channels_sample(all_adc_values);
    if (err != GS_OK) {
        log_error("ADC bulk sample failed: %s", gs_error_string(err));
        return err;
    }

    for (int i = 0; i < NUM_RADFET; i++) {
        uint8_t ch = radfet_channels[i];
        int16_t raw = all_adc_values[ch];
        sample->adc[i][r] = raw;
        log_info("ADC[%i] Dosimeter %i R%i: ADC = %d", ch, i + 1, r + 1, raw);
    }
    return GS_OK;
}

static gs_error_t radfet_enable_all(int r) {
    uint8_t port0 = 0;
    uint8_t port1 = 0;

    // Enable CTRL for all 5 dosimeters
    port0 |= (P00_D3_EN | P06_D1_EN | P03_D2_EN);
    port1 |= (P15_D4_EN | P12_D5_EN);

    // Choose R1 or R2 lines
    if (r == 0) {
        port0 |= (P01_D3_R1 | P07_D1_R1 | P04_D2_R1);
        port1 |= (P14_D4_R1 | P11_D5_R1);
    } else if (r == 1) {
        port0 |= (P05_D1_R2 | P02_D2_R2);
        port1 |= (P17_D3_R2 | P16_D4_R2 | P13_D5_R2);
    } else {
        log_error("Invalid RADFET mode: %d", r);
        return GS_ERROR_ARG;
    }

    gs_error_t err = update_io_expander(port0, port1);
    if (err != GS_OK) {
        log_error("Failed to update I2C I/O Expander: %s", gs_error_string(err));
        return err;
    }

    uint8_t out0, out1;
    if ((err = read_tca9539_register(TCA9539_OUT_PORT0, &out0)) != GS_OK) {
        log_error("Failed to read OUT_PORT0: %s", gs_error_string(err));
        return err;
    }
    log_info("OUT_PORT0 = 0x%02X", out0);

    if ((err = read_tca9539_register(TCA9539_OUT_PORT1, &out1)) != GS_OK) {
        log_error("Failed to read OUT_PORT1: %s", gs_error_string(err));
        return err;
    }
    log_info("OUT_PORT1 = 0x%02X", out1);

    // 200 ms based on Varadis guidance to allow voltages to settle
    gs_time_sleep_ms(200);
    return GS_OK;
}

static inline gs_error_t radfet_disable_all(void) {
    return update_io_expander(0, 0);
}

// ===== Main polling task =====
static void * radfet_poll_task(void * param) {
    gs_error_t err;

    if (!ring_capacity_ok()) {
        log_error("RADFET ring has zero capacity; check flash size and packet size.");
        gs_thread_exit(NULL);
    }

    if (!radfet_load_metadata()) {
        log_info("Metadata invalid or not found — initializing defaults");
        radfet_metadata.flash_write_offset = 0;
        radfet_metadata.samples_saved      = 0;
        radfet_metadata.sample_rate_ms     = 60000;
        err = radfet_save_metadata();
        if (err != GS_OK) {
            log_error("Failed to save metadata @ addr 0x%08lx", (uint32_t)RADFET_METADATA_ADDR);
        }
    } else {
        log_info("Metadata successfully loaded in polling task");
    }

    for (;;) {
        wdt_clear();

        // Assemble a packet; zero it so padding bytes are deterministic for CRC
        radfet_packet_t pkt;
        memset(&pkt, 0, sizeof(pkt));

        pkt.sample.index = radfet_metadata.samples_saved;

        log_info("=== RADFET Sample ===");

        // R1 then R2
        for (int r = 0; r < RADFET_PER_MODULE; r++) {
            err = radfet_enable_all(r);
            if (err == GS_OK) {
                if (radfet_read_all(&pkt.sample, r) != GS_OK) {
                    log_error("Failed to read R%d channels", r + 1);
                }
            } else {
                log_error("Failed to enable sensors: %s", gs_error_string(err));
            }

            err = radfet_disable_all();
            if (err != GS_OK) {
                log_error("Failed to disable sensors: %s", gs_error_string(err));
            }
        }

        // Write sample to internal flash (circular)
        // Normalize write offset for safety and guarantee alignment.
        radfet_metadata.flash_write_offset %= RING_CAP_BYTES;
        radfet_metadata.flash_write_offset -= (radfet_metadata.flash_write_offset % PKT_SIZE);

        uint32_t offset = radfet_metadata.flash_write_offset;
        void *target_addr = (uint8_t *)RADFET_FLASH_START + offset;

        for (int i = 0; i < NUM_RADFET; i++) {
            log_info("  D%i R1 = %d, R2 = %d", i + 1, pkt.sample.adc[i][0], pkt.sample.adc[i][1]);
        }

        // Compute CRC over the packet minus the CRC field
        pkt.crc16 = crc16_ccitt(&pkt, sizeof(pkt) - sizeof(pkt.crc16));

        err = gs_mcu_flash_write_data(target_addr, &pkt, sizeof(pkt));
        if (err != GS_OK) {
            log_error("Failed to write to internal flash: %s", gs_error_string(err));
        } else {
            log_info("Sample %" PRIu32 " written to internal flash @ offset %" PRIu32,
                     pkt.sample.index, offset);

            // Advance write pointer with modulo wrap
            radfet_metadata.flash_write_offset = (offset + PKT_SIZE) % RING_CAP_BYTES;

            // Grow total sample count (reader will clamp to ring size for available)
            radfet_metadata.samples_saved++;

            err = radfet_save_metadata();
            if (err != GS_OK) {
                log_error("Failed to save metadata: %s", gs_error_string(err));
            }
        }

        log_info("==============================");
        gs_time_sleep_ms(radfet_metadata.sample_rate_ms);
    }

    gs_thread_exit(NULL);
}

void radfet_task_init(void) {
    log_info("Initializing Radfet Polling Task");

    // quick gosh commands for bring-up (as comments)
    // peek 0x80040000 52
    // peek 0x80080200 14
    // reset

    if (tca9539_config() != GS_OK) {
        log_error("Failed to initialize I2C I/O Converter");
    }

    gs_thread_create("radfet_poll", radfet_poll_task, NULL,
                     3000, GS_THREAD_PRIORITY_LOW, 0, NULL);
}
