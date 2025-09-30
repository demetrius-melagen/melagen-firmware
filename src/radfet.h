#ifndef RADFET_H
#define RADFET_H

#include <stdint.h>
#include <stddef.h>     // for size_t
#include <avr32/io.h>

// ---------- TCA9539 (I2C expander) ----------
#define TCA9539_I2C_ADDR   0x74
#define TCA9539_CFG_PORT0  0x06
#define TCA9539_CFG_PORT1  0x07
#define TCA9539_OUT_PORT0  0x02
#define TCA9539_OUT_PORT1  0x03

// OUT_PORT0 bits (P00–P07)
#define P00_D3_EN   (1u << 0)
#define P01_D3_R1   (1u << 1)
#define P02_D2_R2   (1u << 2)
#define P03_D2_EN   (1u << 3)
#define P04_D2_R1   (1u << 4)
#define P05_D1_R2   (1u << 5)
#define P06_D1_EN   (1u << 6)
#define P07_D1_R1   (1u << 7)

// OUT_PORT1 bits (P10–P17)
#define P10_UNUSED  (1u << 0)
#define P11_D5_R1   (1u << 1)
#define P12_D5_EN   (1u << 2)
#define P13_D5_R2   (1u << 3)
#define P14_D4_R1   (1u << 4)
#define P15_D4_EN   (1u << 5)
#define P16_D4_R2   (1u << 6)
#define P17_D3_R2   (1u << 7)

#define I2C_TIMEOUT_MS  100

// ---------- RADFET sampling/packet layout ----------
#define NUM_RADFET         5      // D1..D5
#define RADFET_PER_MODULE  2      // R1, R2

typedef struct __attribute__((packed)) {
    // uint32_t timestamp;   // optional — only enable if reader/writer both expect it
    uint32_t index;                    // 4 bytes
    int16_t  adc[NUM_RADFET][RADFET_PER_MODULE]; // 20 bytes
} radfet_sample_t;                     // = 24 bytes

typedef struct __attribute__((packed)) {
    radfet_sample_t sample;  // 24
    uint16_t        crc16;   //  2 (CRC over all prior bytes)
} radfet_packet_t;           // = 26 bytes

#define PKT_SIZE sizeof(radfet_packet_t)

// ---------- Metadata ----------
typedef struct __attribute__((packed)) {
    uint32_t flash_write_offset;   // byte offset within data ring
    uint32_t samples_saved;        // monotonically increasing counter
    uint32_t sample_rate_ms;       // default 60000 ms
    uint16_t crc16;                // CRC over this struct excluding crc16
} radfet_metadata_t;

extern radfet_metadata_t radfet_metadata;

#define METADATA_PKT_SIZE sizeof(radfet_metadata_t)

// ---------- Flash map ----------
// Code region (per nanomind-bsp.map): 0x80000000 .. ~0x8002F798
// Data ring lives here:
#define RADFET_FLASH_START   ((void *) 0x80040000u)
#define RADFET_FLASH_END     ((void *) 0x80080000u)     // exclusive end
#define RADFET_FLASH_SIZE    ((uintptr_t)RADFET_FLASH_END - (uintptr_t)RADFET_FLASH_START)

// Metadata lives outside the data ring
#define RADFET_METADATA_ADDR ((void *)(0x80080000u + AVR32_FLASH_PAGE_SIZE))

// ---------- CRC API ----------
uint16_t crc16_ccitt(const void *data, size_t length);

// ---------- Sanity checks ----------
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(radget_sample_t) == 24, "radfet_sample_t must be 24 bytes");
_Static_assert(sizeof(radget_packet_t) == 26, "radfet_packet_t must be 26 bytes");
#endif

#endif // RADFET_H
