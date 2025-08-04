#ifndef RADFET_H
#define RADFET_H

#include <stdint.h>

#define NUM_RADFET 5
#define RADFET_PER_MODULE 2
#define PKT_SIZE sizeof(radfet_packet_t)

#define RADFET_FLASH_START ((void *) 0x80040000)
#define RADFET_FLASH_END   ((void *) 0x80080000)
#define RADFET_FLASH_SIZE  (0x80000 - 0x40000)

extern uint32_t flash_write_offset;
extern bool safe_mode;
extern uint32_t samples_saved;

// Sample interval (ms) — adjust as needed
extern uint32_t sample_rate_ms;  

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