#ifndef RADFET_H
#define RADFET_H

#include <stdint.h>
#include <avr32/io.h>

#define TCA9539_I2C_ADDR  0x74 // I2C expander address
#define TCA9539_CFG_PORT0 0x06 // I2C expander configuration port 0
#define TCA9539_CFG_PORT1 0x07 // I2C expander configuration port 1
#define TCA9539_OUT_PORT0 0x02 // I2C expander output port 0
#define TCA9539_OUT_PORT1 0x03 // I2C expander output port 1

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

#define NUM_RADFET 5            // 5 sensors
#define RADFET_PER_MODULE 2     // 2 Radfets per sensors, R1 and R2
#define PKT_SIZE sizeof(radfet_packet_t)    
#define METADATA_PKT_SIZE sizeof(radfet_metadata_t)

//0x80000000 to 0x8002f798 is used for firmware code (from nanomind-bsp.map)
#define RADFET_FLASH_START ((void *) 0x80040000) 
#define RADFET_FLASH_END   ((void *) 0x80080000)
#define RADFET_FLASH_SIZE  ((uintptr_t)RADFET_FLASH_END - (uintptr_t)RADFET_FLASH_START)
#define RADFET_METADATA_ADDR ((void *) (0x80080000 + AVR32_FLASH_PAGE_SIZE)) 

// extern bool radfet_polling;

typedef struct __attribute__((packed)) {
    uint32_t flash_write_offset;  //address to save and read from
    uint32_t samples_saved;       
    uint32_t sample_rate_ms;   // sample interval, 60 seconds by default
    uint16_t crc16;               
} radfet_metadata_t;

extern radfet_metadata_t radfet_metadata;

typedef struct __attribute__((packed)) {
    // uint32_t timestamp;   // 4 bytes
    uint32_t index; // 4 bytes
    int16_t adc[NUM_RADFET][RADFET_PER_MODULE];  // 2 x 5 x 2 = 20 bytes
} radfet_sample_t; // Total: 24 bytes

typedef struct __attribute__((packed)) {
    radfet_sample_t sample;  // 24 bytes (index + adc_mv) // 20 bytes
    uint16_t crc16;          // 2 bytes, CRC over all prior bytes
} radfet_packet_t;

// Packet structure to save in internal flash:
// one timestamp for each poll
// [timestamp][D1R1][D1R2][D2R1][D2R2][D3R1][D3R2][D4R1][D4R2][D5R1][D5R2][CRC]
// [index][D1R1][D1R2][D2R1][D2R2][D3R1][D3R2][D4R1][D4R2][D5R1][D5R2][CRC]

uint16_t crc16_ccitt(const void *data, size_t length);

#endif