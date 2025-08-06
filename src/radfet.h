#ifndef RADFET_H
#define RADFET_H

#include <stdint.h>

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

#define NUM_RADFET 5
#define RADFET_PER_MODULE 2
#define PKT_SIZE sizeof(radfet_packet_t)
#define METADATA_PKT_SIZE sizeof(radfet_metadata_t)

#define RADFET_METADATA_SIZE 0x00000400
#define RADFET_FLASH_START ((void *) 0x80040000)
#define RADFET_FLASH_END   ((void *) (0x80080000 - RADFET_METADATA_SIZE))
#define RADFET_FLASH_SIZE  ((uintptr_t)RADFET_FLASH_END - (uintptr_t)RADFET_FLASH_START)
#define RADFET_METADATA_ADDR ((void *) (0x80080000 - RADFET_METADATA_SIZE)) 

typedef struct __attribute__((packed)) {
    uint32_t flash_write_offset;  
    uint32_t samples_saved;       
    uint32_t sample_rate_ms;   
    uint16_t crc16;               
} radfet_metadata_t;

// extern uint32_t flash_write_offset;
// extern uint32_t samples_saved;
// // Sample interval (ms) — adjust as needed
// extern uint32_t sample_rate_ms;  
extern radfet_metadata_t radfet_metadata;

typedef struct __attribute__((packed)) {
    uint32_t timestamp;   // 4 bytes
    int16_t adc[NUM_RADFET][RADFET_PER_MODULE];  // 2 x 5 x 2 = 20 bytes
} radfet_sample_t; // Total: 24 bytes

typedef struct __attribute__((packed)) {
    radfet_sample_t sample;  // 24 bytes (timestamp + adc_mv)
    uint16_t crc16;          // 2 bytes, CRC over all prior bytes
} radfet_packet_t;

// Packet structure to save in FRAM:
// one timestamp for each poll
// [timestamp][D1R1][D1R2][D2R1][D2R2][D3R1][D3R2][D4R1][D4R2][D5R1][D5R2]

uint16_t crc16_ccitt(const void *data, size_t length);

#endif