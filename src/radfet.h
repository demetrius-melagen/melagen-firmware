#ifndef RADFET_H
#define RADFET_H

#include <stdint.h>

#define NUM_RADFET 1
#define RADFET_PER_MODULE 2
#define PKT_SIZE sizeof(radfet_packet_t)

extern uint32_t fram_write_offset;
// Sample interval (ms) — adjust as needed
extern uint32_t sample_rate_ms;  

typedef struct __attribute__((packed)) {
    uint32_t timestamp;   // 4 bytes
    int16_t adc[NUM_RADFET][RADFET_PER_MODULE];  // 20 bytes
} radfet_sample_t; // Total: 24 bytes

typedef struct __attribute__((packed)) {
    // uint8_t  sync[2];        // 0xAA 0x55 2 bytes
    // uint8_t  cmd_id;         // 1 byte
    // uint16_t length;         // 2 bytes (24 for the sample)
    radfet_sample_t sample;  // 24 bytes (timestamp + adc_mv)
    uint16_t crc16;          // 2 bytes, CRC over all prior bytes
} radfet_packet_t;

// Packet structure to save in FRAM:
// one timestamp for each poll
// [timestamp][D1R1][D1R2][D2R1][D2R2][D3R1][D3R2][D4R1][D4R2][D5R1][D5R2]
// 4 + 2 readings x 5 sensors x 2 bytes = 24 bytes per 60 seconds 
// 120 samples per 2 hours
// 120 samples × 24 bytes = 2880 bytes
// (2880 / 32768) × 100 ≈ 8.8% of 32kB FRAM used every 2 hours

uint16_t crc16_ccitt(const void *data, size_t length);

#endif