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

// static const gs_vmem_t *fFram = NULL;
uint32_t fram_write_offset = 0;
uint32_t flash_write_offset = 0;
// static uint32_t nor_block_index = 0;

#define TCA9539_I2C_ADDR  0x74 // I2C expander address
#define TCA9539_CFG_PORT0 0x06 
#define TCA9539_CFG_PORT1 0x07
#define TCA9539_OUT_PORT0 0x02
#define TCA9539_OUT_PORT1 0x03

// Register 0x02: Output Port 0 (P00–P07)
#define P00_D3_EN   (1 << 0)    // Dosimeter 3 Enable
#define P01_D3_R1   (1 << 1)    // Dosimeter 3 R1
#define P02_D2_R2   (1 << 2)    // Dosimeter 2 R2
#define P03_D2_EN   (1 << 3)    // Dosimeter 2 Enable
#define P04_D2_R1   (1 << 4)    // Dosimeter 2 R1
#define P05_D1_R2   (1 << 5)    // Dosimeter 1 R2
#define P06_D1_EN   (1 << 6)    // Dosimeter 1 Enable
#define P07_D1_R1   (1 << 7)    // Dosimeter 1 R1

// Register 0x03: Output Port 1 (P10–P17)
#define P10_UNUSED  (1 << 0)    // unused
#define P11_D5_R1   (1 << 1)    // Dosimeter 5 R1
#define P12_D5_EN   (1 << 2)    // Dosimeter 5 Enable
#define P13_D5_R2   (1 << 3)    // Dosimeter 5 R2
#define P14_D4_R1   (1 << 4)    // Dosimeter 4 R1
#define P15_D4_EN   (1 << 5)    // Dosimeter 4 Enable
#define P16_D4_R2   (1 << 6)    // Dosimeter 4 R2
#define P17_D3_R2   (1 << 7)    // Dosimeter 3 R2

#define I2C_TIMEOUT_MS    100

//Helper function to write a value to a register over I2C
gs_error_t write_tca9539_register(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg, value};
    return gs_i2c_master_transaction(0, TCA9539_I2C_ADDR, tx, 2, NULL, 0, I2C_TIMEOUT_MS);
}

//Helper function to write to both ports on tca9539
void update_io_expander(uint8_t port0, uint8_t port1) {
    write_tca9539_register(TCA9539_OUT_PORT0, port0);
    write_tca9539_register(TCA9539_OUT_PORT1, port1);
    // gs_time_sleep_ms(10);  // optional settle time
}

//Helper function to read from register over I2C
gs_error_t read_tca9539_register(uint8_t reg, uint8_t *value) {
    return gs_i2c_master_transaction(0, TCA9539_I2C_ADDR, &reg, 1, value, 1, I2C_TIMEOUT_MS);
}

// Helper function to initialize i2c to i/o converter
void tca9539_config(void) {
    gs_error_t err;
    err = write_tca9539_register(TCA9539_OUT_PORT0, 0x00);  
    if (err != GS_OK) {
        log_error("Failed to write OUT_PORT0: %s", gs_error_string(err));
        return;
    }
    err = write_tca9539_register(TCA9539_OUT_PORT1, 0x00);  
    if (err != GS_OK) {
        log_error("Failed to write OUT_PORT1: %s", gs_error_string(err));
        return;
    }
    err = write_tca9539_register(TCA9539_CFG_PORT0, 0x00);
    if (err != GS_OK) {
        log_error("Failed to write CFG_PORT0: %s", gs_error_string(err));
        return;
    }
    err = write_tca9539_register(TCA9539_CFG_PORT1, 0x00);
    if (err != GS_OK) {
        log_error("Failed to write CFG_PORT1: %s", gs_error_string(err));
        return;
    }
    uint8_t cfg0, cfg1;
    err = read_tca9539_register(TCA9539_CFG_PORT0, &cfg0);
    if (err == GS_OK){
        log_info("CFG_PO}RT0 = 0x%02X → %s", cfg0, (cfg0 == 0x00) ? "OK (all outputs)" : "NOT OK");
    }else{
        log_error("Failed to read CFG_PORT0: %s", gs_error_string(err));
    }  
    err = read_tca9539_register(TCA9539_CFG_PORT1, &cfg1);
    if (err == GS_OK){
        log_info("CFG_PORT1 = 0x%02X → %s", cfg1, (cfg1 == 0x00) ? "OK (all outputs)" : "NOT OK");
    }
    else{
        log_error("Failed to read CFG_PORT1: %s", gs_error_string(err));
    }
    uint8_t out0, out1;
    err = read_tca9539_register(TCA9539_OUT_PORT0, &out0);
    if (err == GS_OK){
        log_info("OUT_PORT0 = 0x%02X", out0);
    } else{
        log_error("Failed to read OUT_PORT0: %s", gs_error_string(err));
    }
    err = read_tca9539_register(TCA9539_OUT_PORT1, &out1);
    if (err == GS_OK){
        log_info("OUT_PORT1 = 0x%02X", out1);
    } else {
        log_error("Failed to read OUT_PORT1: %s", gs_error_string(err));
    }

}

//ADC Read All Helper Function
gs_error_t radfet_read_all(radfet_sample_t *sample, int r){
    int16_t all_adc_values[GS_A3200_ADC_NCHANS] = {0};
    gs_error_t err = gs_a3200_adc_channels_sample(all_adc_values); //take a sample of all adc channels
    uint32_t rel_ms = gs_time_rel_ms();
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
        log_info("ADC[%i] Dosimeter %i R%i: ADC = %d  @%" PRIu32, ch, i+1, r+1, raw, rel_ms);
    }
    return err;
}

// helper function to enable all the sensors based on r1 or r2
void radfet_enable_all(int r) {
    // Start with all lines inactive (low)
    uint8_t port0 = 0;
    uint8_t port1 = 0;
    // Enable CTRL for all 5 dosimeters
    port0 |= (P00_D3_EN | P06_D1_EN | P03_D2_EN );
    port1 |= (P15_D4_EN | P12_D5_EN);
    // make sure that if the sensor is enabled, at least r1 or r2 are enabled to prevent 12V spike
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
        log_error("Invalid RADFET mode: %d (expected 0 for R1, 1 for R2)", r);
        return;
    }
    // Send I/O update
    update_io_expander(port0, port1);

    gs_error_t err;
    uint8_t out0, out1;
    err = read_tca9539_register(TCA9539_OUT_PORT0, &out0);
    if (err == GS_OK){
        log_info("OUT_PORT0 = 0x%02X", out0);
    } else{
        log_error("Failed to read OUT_PORT0: %s", gs_error_string(err));
    }
    err = read_tca9539_register(TCA9539_OUT_PORT1, &out1);
    if (err == GS_OK){
        log_info("OUT_PORT1 = 0x%02X", out1);
    } else {
        log_error("Failed to read OUT_PORT1: %s", gs_error_string(err));
    }
    gs_time_sleep_ms(25);  // Allow voltages to settle
}

// helper function to disable all the sensors
void radfet_disable_all(void) {
    // Set all control lines to LOW (inactive)
    update_io_expander(0, 0);
    // gs_time_sleep_ms(100);  // Optional settle delay
}

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

void test_internal_flash_rw(void) {
    uint8_t test_value = 0xAB;     // Arbitrary byte to write
    uint8_t read_value = 0x00;     // Variable to store read-back
    void *flash_addr = (void *)0x80040000;  // Pick a safe offset in internal flash

    log_info("Writing 0x%02X to internal flash at address %p", test_value, flash_addr);

    gs_error_t res_write = gs_mcu_flash_write_data(flash_addr, &test_value, sizeof(test_value));
    if (res_write != GS_OK) {
        log_error("Flash write failed with code %d", res_write);
        return;
    }

    gs_error_t res_read = gs_mcu_flash_read_data(&read_value, flash_addr, sizeof(read_value));
    if (res_read != GS_OK) {
        log_error("Flash read failed with code %d", res_read);
        return;
    }

    log_info("Read back 0x%02X from internal flash at address %p", read_value, flash_addr);

    if (read_value == test_value) {
        log_info("Flash test PASSED: values match.");
    } else {
        log_error("Flash test FAILED: expected 0x%02X, got 0x%02X", test_value, read_value);
    }
}

// Sample interval (ms) — adjust as needed
uint32_t sample_rate_ms = 100; //60000;  // 60 seconds 
uint32_t samples_saved = 0;

// Task that periodically samples RADFETs and stores data
static void * radfet_poll_task(void * param)
{
    radfet_packet_t pkt;

    // int16_t sensor_mv[NUM_RADFET][RADFET_PER_MODULE]; // index [sensor][R1/R2]
    for (;;) {
        // Touch watchdog to prevent reset.
        // This should be tied into other tasks as well, to ensure everything is running.
        wdt_clear();
        if (safe_mode == false){
            pkt.sample.timestamp = gs_time_rel_ms();
            log_info("=== RADFET Sample ==="); 
        
            // updated loop 
            for (int r= 0; r < RADFET_PER_MODULE; r++){
                // enable all sensors and all R
                radfet_enable_all(r);
                // poll all adc channels 
                if (radfet_read_all(&pkt.sample, r) != GS_OK) {
                    log_error("Failed to read R%d channels", r + 1);
                }
                // disable all sensors  
                radfet_disable_all();
            }

            // Write sample to internal flash with circular buffer behavior
            if ((flash_write_offset + PKT_SIZE) >= RADFET_FLASH_SIZE) {
                log_info("Flash offset exceeded — wrapping to beginning");
                flash_write_offset = 0;
            }

            void *target_addr = (uint8_t *)RADFET_FLASH_START + flash_write_offset;

            log_info("Writing to internal flash: timestamp = %" PRIu32, pkt.sample.timestamp);
            for (int i = 0; i < NUM_RADFET; i++) {
                log_info("  D%i R1 = %d mV, R2 = %d mV", i + 1, pkt.sample.adc[i][0], pkt.sample.adc[i][1]);
            }

            pkt.crc16 = crc16_ccitt(&pkt, sizeof(pkt) - sizeof(pkt.crc16));

            gs_error_t err = gs_mcu_flash_write_data(target_addr, &pkt, sizeof(pkt));
            if (err != GS_OK) {
                log_error("Failed to write to internal flash: %s", gs_error_string(err));
            } else {
                log_info("Sample written to internal flash @ offset %" PRIu32 " (timestamp = %" PRIu32 ")",
                        flash_write_offset, pkt.sample.timestamp);
                flash_write_offset += PKT_SIZE;
                samples_saved++;
            }
            log_info("==============================");
        }
        //delay the task by 60 seconds
        gs_time_sleep_ms(sample_rate_ms);
    }
    // Will never get here
    gs_thread_exit(NULL);
}

void radfet_task_init(void)
{
    // test_internal_flash_rw();
    // peek 0x80040000 26

    // initialize i2c to i/o converter, set output ports to 0 and set ports to output mode
    tca9539_config();
    gs_thread_create("radfet_poll", radfet_poll_task, NULL, 3000, GS_THREAD_PRIORITY_LOW, 0, NULL);
}