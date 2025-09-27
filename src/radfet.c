/*
RADFET Data Collection: 
- Enabling sensors through tca9539 I2C to i/o converter
- Polling using ADC Channels
- Saving sample packets to FRAM
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
#include <gs/util/drivers/i2c/master.h>  // for I2C transactions
#include <inttypes.h>
#include <string.h>  // for memset
#include "radfet.h"
#include <gs/thirdparty/flash/spn_fl512s.h>
#include <gs/embed/drivers/flash/mcu_flash.h>

//ADC RADFET pin configuration
static const uint8_t radfet_channels[NUM_RADFET] = {
    7,  // D1 → AD7
    6,  // D2 → AD6
    5,  // D3 → AD5
    4,  // D4 → AD4
    0   // D5 → AD0
}; 

// bool radfet_polling = true;
radfet_metadata_t radfet_metadata = {
    .flash_write_offset = 0,
    .samples_saved = 0,
    .sample_rate_ms = 60000,  // default sample rate
    .crc16 = 0,
};

//Helper function to write a value to a register over I2C
gs_error_t write_tca9539_register(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg, value};
    return gs_i2c_master_transaction(0, TCA9539_I2C_ADDR, tx, 2, NULL, 0, I2C_TIMEOUT_MS);
}

//Helper function to write to both ports on tca9539
gs_error_t update_io_expander(uint8_t port0, uint8_t port1) {
    gs_error_t err;
    err = write_tca9539_register(TCA9539_OUT_PORT0, port0);
    err = write_tca9539_register(TCA9539_OUT_PORT1, port1);
    return err;
}

//Helper function to read from register over I2C
gs_error_t read_tca9539_register(uint8_t reg, uint8_t *value) {
    return gs_i2c_master_transaction(0, TCA9539_I2C_ADDR, &reg, 1, value, 1, I2C_TIMEOUT_MS);
}

// Helper function to initialize i2c to i/o converter
gs_error_t tca9539_config(void) {
    gs_error_t err;
    err = write_tca9539_register(TCA9539_OUT_PORT0, 0x00);  
    if (err != GS_OK) {
        log_error("Failed to write OUT_PORT0: %s", gs_error_string(err));
        return err;
    }
    err = write_tca9539_register(TCA9539_OUT_PORT1, 0x00);  
    if (err != GS_OK) {
        log_error("Failed to write OUT_PORT1: %s", gs_error_string(err));
        return err;
    }
    err = write_tca9539_register(TCA9539_CFG_PORT0, 0x00);
    if (err != GS_OK) {
        log_error("Failed to write CFG_PORT0: %s", gs_error_string(err));
        return err;
    }
    err = write_tca9539_register(TCA9539_CFG_PORT1, 0x00);
    if (err != GS_OK) {
        log_error("Failed to write CFG_PORT1: %s", gs_error_string(err));
        return err;
    }
    uint8_t cfg0, cfg1;
    err = read_tca9539_register(TCA9539_CFG_PORT0, &cfg0);
    if (err == GS_OK){
        log_info("CFG_PO}RT0 = 0x%02X → %s", cfg0, (cfg0 == 0x00) ? "OK (all outputs)" : "NOT OK");
    }else{
        log_error("Failed to read CFG_PORT0: %s", gs_error_string(err));
        return err;
    }  
    err = read_tca9539_register(TCA9539_CFG_PORT1, &cfg1);
    if (err == GS_OK){
        log_info("CFG_PORT1 = 0x%02X → %s", cfg1, (cfg1 == 0x00) ? "OK (all outputs)" : "NOT OK");
    }
    else{
        log_error("Failed to read CFG_PORT1: %s", gs_error_string(err));
        return err;
    }
    uint8_t out0, out1;
    err = read_tca9539_register(TCA9539_OUT_PORT0, &out0);
    if (err == GS_OK){
        log_info("OUT_PORT0 = 0x%02X", out0);
    } else{
        log_error("Failed to read OUT_PORT0: %s", gs_error_string(err));
        return err;
    }
    err = read_tca9539_register(TCA9539_OUT_PORT1, &out1);
    if (err == GS_OK){
        log_info("OUT_PORT1 = 0x%02X", out1);
    } else {
        log_error("Failed to read OUT_PORT1: %s", gs_error_string(err));
        return err;
    }
    return err;
}

//ADC Read All Helper Function
gs_error_t radfet_read_all(radfet_sample_t *sample, int r){
    int16_t all_adc_values[GS_A3200_ADC_NCHANS] = {0};
    gs_error_t err = gs_a3200_adc_channels_sample(all_adc_values); //take a sample of all adc channels
    // uint32_t rel_ms = gs_time_rel_ms();
    if (err != GS_OK) {
        log_error("ADC bulk sample failed: %s", gs_error_string(err));
        return err;
    }
    uint8_t ch; // = radfet_channels[i];
    int16_t raw;
    //read all connected adcs
    for (int i = 0; i < NUM_RADFET; i++){ // NUM_RADFET
        ch = radfet_channels[i]; //get adc channel from corresponding radfet
        raw = all_adc_values[ch]; //get adc value from adc channel
        sample->adc[i][r] = raw;
        // log_info("ADC[%i] Dosimeter %i R%i: ADC = %d  @%" PRIu32, ch, i+1, r+1, raw, rel_ms);
        log_info("ADC[%i] Dosimeter %i R%i: ADC = %d", ch, i+1, r+1, raw);
    }
    return err;
}

// helper function to enable all the sensors based on r1 or r2
gs_error_t radfet_enable_all(int r) {
    // Start with all lines inactive (low)
    uint8_t port0 = 0;
    uint8_t port1 = 0;
    // Enable CTRL for all 5 dosimeters
    port0 |= (P00_D3_EN | P06_D1_EN | P03_D2_EN );
    port1 |= (P15_D4_EN | P12_D5_EN);
    // Make sure that if the sensor is enabled, at least r1 or r2 are enabled to prevent 12V spike
    // Depending on R1 or R2, enable the right control lines
    if (r == 0) {
        // R1 enable for all
        port0 |= (P01_D3_R1 | P07_D1_R1 | P04_D2_R1);
        port1 |= (P14_D4_R1 | P11_D5_R1);
    } else if (r == 1) {
        // R2 enable for all
        port0 |= (P05_D1_R2 | P02_D2_R2);
        port1 |= (P17_D3_R2 | P16_D4_R2 | P13_D5_R2);
    } else {
        log_error("Invalid RADFET mode: %d", r);
        return GS_ERROR_ARG;
    }
    gs_error_t err;
    // Send I/O update
    err = update_io_expander(port0, port1);
    if (err != GS_OK){
        log_error("Failed to update I2C I/O Expander: %s", gs_error_string(err));
    }
    uint8_t out0, out1;
    err = read_tca9539_register(TCA9539_OUT_PORT0, &out0);
    if (err == GS_OK){
        log_info("OUT_PORT0 = 0x%02X", out0);
    } else{
        log_error("Failed to read OUT_PORT0: %s", gs_error_string(err));
        return err;
    }
    err = read_tca9539_register(TCA9539_OUT_PORT1, &out1);
    if (err == GS_OK){
        log_info("OUT_PORT1 = 0x%02X", out1);
    } else {
        log_error("Failed to read OUT_PORT1: %s", gs_error_string(err));
        return err;
    }
    //25 ms based on Varadis data sheets
    //200 ms based on email correspondence with Varadis 
    gs_time_sleep_ms(200);  // Allow voltages to settle
    return err;
}

// helper function to disable all the sensors
gs_error_t radfet_disable_all(void) {
    // Set all control lines to LOW (inactive)
    gs_error_t err = update_io_expander(0, 0);
    return err;
}
// Helper function to perform cyclic redundancy check 
uint16_t crc16_ccitt(const void *data, size_t length) {
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}
// Helper function to perform cyclic redundancy check on metadata
static uint16_t calc_metadata_crc(const radfet_metadata_t *meta) {
    return crc16_ccitt(meta, sizeof(radfet_metadata_t) - sizeof(meta->crc16));
}

// Helper function to load metadata 
bool radfet_load_metadata(void) {
    radfet_metadata_t meta = radfet_metadata; 
    const void *addr = (void *)RADFET_METADATA_ADDR;
    log_info("Reading metadata from address: 0x%08lx", (uint32_t)addr);

    gs_error_t err = gs_mcu_flash_read_data(&meta, addr, METADATA_PKT_SIZE);
    if (err != GS_OK) {
        log_error("Failed to read metadata from flash (err = %d)", err);
        return false;
    }
    // Validate CRC
    uint16_t expected = meta.crc16;
    meta.crc16 = 0;
    uint16_t actual = calc_metadata_crc(&meta);

    log_info("Read metadata: offset=%" PRIu32 ", count=%" PRIu32 ", rate=%" PRIu32 ", expected_crc=0x%04X, actual_crc=0x%04X",
             meta.flash_write_offset, meta.samples_saved, meta.sample_rate_ms, expected, actual);

    bool valid = (expected == actual) &&
                 meta.flash_write_offset < RADFET_FLASH_SIZE &&
                 (meta.flash_write_offset % PKT_SIZE) == 0;
    if (valid) { radfet_metadata = meta; } 
    return valid;
}
// Helper function to save metadata
gs_error_t radfet_save_metadata(void) {
    radfet_metadata_t meta = radfet_metadata;
    meta.crc16 = 0;
    meta.crc16 = calc_metadata_crc(&meta);
    gs_error_t err = gs_mcu_flash_write_data(RADFET_METADATA_ADDR, (uint8_t *)&meta, METADATA_PKT_SIZE);
    return err;
}

// Helper function to test writing, reading and overwriting internal flash
void test_internal_flash_rw(void) {
    uint8_t first_write = 0x55;  // 01010101
    uint8_t second_write = 0xAA; // 10101010 

    uint8_t read_value = 0x00;
    void* addr = RADFET_FLASH_START;

    log_info("=== Internal Flash Overwrite Test ===");

    log_info("Writing 0x%02X to flash at %p", first_write, addr);
    if (gs_mcu_flash_write_data(addr, &first_write, 1) != GS_OK) {
        log_error("First flash write failed");
        return;
    }

    gs_mcu_flash_read_data(&read_value, addr, 1);
    log_info("Read back 0x%02X", read_value);
    if (read_value != first_write) {
        log_error("Mismatch after first write");
        return;
    }

    log_info("Attempting overwrite: writing 0x%02X to flash at %p", second_write, addr);
    if (gs_mcu_flash_write_data(addr, &second_write, 1) != GS_OK) {
        log_error("Second flash write failed");
        return;
    }

    gs_mcu_flash_read_data(&read_value, addr, 1);
    log_info("Read back after overwrite: 0x%02X", read_value);
    log_info("=== Flash Overwrite Test Complete ===");
}

// Task that periodically samples RADFETs and stores data
static void * radfet_poll_task(void * param)
{   
    gs_error_t err;
    radfet_packet_t pkt;
    if (!radfet_load_metadata()) {
        log_info("Metadata invalid or not found — initializing defaults");
        radfet_metadata.flash_write_offset = 0;
        // radfet_metadata.flash_write_offset = (RADFET_FLASH_SIZE - PKT_SIZE * 2) - ((RADFET_FLASH_SIZE - PKT_SIZE * 2) % PKT_SIZE); // rollover test
        radfet_metadata.samples_saved = 0;
        radfet_metadata.sample_rate_ms = 60000;
        err = radfet_save_metadata();
        if (err != GS_OK){
                log_error("Failed to save metadata @ addr from address: 0x%08lx", (uint32_t)RADFET_METADATA_ADDR);
            }
    } else {
        log_info("Metadata successfully loaded in polling task");
    }

    for (;;) {
        // Touch watchdog to prevent reset.
        // This should be tied into other tasks as well, to ensure everything is running.
        wdt_clear();
        // radfet_polling = true;
        pkt.sample.index = radfet_metadata.samples_saved;
        // pkt.sample.timestamp = gs_time_rel_ms();
        log_info("=== RADFET Sample ==="); 
        for (int r= 0; r < RADFET_PER_MODULE; r++){
            // enable all sensors and all R
            err = radfet_enable_all(r);
            if (err == GS_OK){// poll all adc channels 
                if (radfet_read_all(&pkt.sample, r) != GS_OK) {
                    log_error("Failed to read R%d channels", r + 1);
                }
            } else {
                log_error("Failed to enable sensors: %s", gs_error_string(err));
            }
            // disable all sensors  
            err = radfet_disable_all();
            if (err != GS_OK){
                log_error("Failed to disable sensors: %s", gs_error_string(err));
            }
        }

        // Write sample to internal flash with circular buffer behavior
        if ((radfet_metadata.flash_write_offset + PKT_SIZE) >= RADFET_FLASH_SIZE) {
            log_info("Flash offset exceeded size — wrapping to beginning");
            radfet_metadata.flash_write_offset = 0;
        }

        void *target_addr = (uint8_t *)RADFET_FLASH_START + radfet_metadata.flash_write_offset;

        // log_info("Writing to internal flash: timestamp = %" PRIu32, pkt.sample.timestamp);
        for (int i = 0; i < NUM_RADFET; i++) {
            log_info("  D%i R1 = %d, R2 = %d", i + 1, pkt.sample.adc[i][0], pkt.sample.adc[i][1]);
        }
        pkt.crc16 = crc16_ccitt(&pkt, sizeof(pkt) - sizeof(pkt.crc16));
        err = gs_mcu_flash_write_data(target_addr, &pkt, sizeof(pkt));
        if (err != GS_OK) {
            log_error("Failed to write to internal flash: %s", gs_error_string(err));
        } else {
            // log_info("Sample written to internal flash @ offset %" PRIu32 " (timestamp = %" PRIu32 ")",
            //         radfet_metadata.flash_write_offset, pkt.sample.timestamp);
            log_info("Sample %" PRIu32 " written to internal flash @ offset %" PRIu32,
                pkt.sample.index,
                radfet_metadata.flash_write_offset);
            radfet_metadata.flash_write_offset += PKT_SIZE;
            radfet_metadata.samples_saved++;
            err = radfet_save_metadata();
            if (err != GS_OK){
                log_error("Failed to save metadata: %s", gs_error_string(err));
            }
        }
           
        log_info("==============================");
        //Delay the task by sample rate
        // radfet_polling = false;
        gs_time_sleep_ms(radfet_metadata.sample_rate_ms); 
    }
    // Will never get here
    gs_thread_exit(NULL);
}

void radfet_task_init(void)
{
    log_info("Initializing Radfet Polling Task");
    // test_internal_flash_rw();
    // quick gosh commands to copy/paste
    // peek 0x80040000 52 
    // peek 0x80080200 14
    // reset

    // initialize i2c to i/o converter, set output ports to 0 and set ports to output mode
    if (tca9539_config() != GS_OK){
        log_error("Failed to initialize I2C I/O Converter");
    }
    gs_thread_create("radfet_poll", radfet_poll_task, NULL, 3000, GS_THREAD_PRIORITY_LOW, 0, NULL);
}