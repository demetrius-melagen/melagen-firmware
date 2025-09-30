#include <gs/util/log.h>
#include <gs/util/time.h>
#include <gs/util/thread.h>
#include <wdt.h>  // libasf
#include <gs/a3200/a3200.h>
#include <gs/a3200/adc_channels.h>
#include <gs/util/types.h>
#include <gs/util/vmem.h>
#include <gs/util/drivers/i2c/master.h>
#include <inttypes.h>
#include <string.h>
#include "radfet.h"
#include <gs/util/clock.h>
#include <gs/util/rtc.h>
#include <gs/embed/drivers/uart/uart.h>
#include <gs/a3200/uart.h>
#include <gs/csp/csp.h>
#include <gs/csp/drivers/kiss/kiss.h>
#include <gs/thirdparty/flash/spn_fl512s.h>
#include <gs/embed/asf/drivers/spi/master.h>
#include <gs/util/string.h>
#include <gs/util/mutex.h>
#include <gs/embed/drivers/flash/mcu_flash.h>

// Constants
#define USART1 1
#define STX 0x02
#define NUM_SAMPLES_TO_SEND (60 * 24 * 5)
#define BLOCK_SIZE 64

// Derived from radfet.h
#define PKT_SIZE            (sizeof(radfet_packet_t))
#define RING_CAP_PACKETS    (RADFET_FLASH_SIZE / PKT_SIZE)
#define RING_CAP_BYTES      (RING_CAP_PACKETS * PKT_SIZE)

static void * task_mode_op(void * param) {
    log_info("Operation Modes initialization complete");

    uint8_t incoming_byte;
    gs_error_t err;

    for (;;) {
        wdt_clear();

        err = gs_uart_read(USART1, 1000, &incoming_byte);
        if (err == GS_OK) {
            log_info("Received byte on USART1: 0x%02X", incoming_byte);

            switch (incoming_byte) {
                case STX: {
                    // Clamp available samples to ring capacity
                    uint32_t available = (radfet_metadata.samples_saved < RING_CAP_PACKETS)
                                           ? radfet_metadata.samples_saved
                                           : RING_CAP_PACKETS;
                    uint32_t num_to_send = (available < NUM_SAMPLES_TO_SEND)
                                             ? available
                                             : NUM_SAMPLES_TO_SEND;

                    log_info("STX received: sending up to %" PRIu32 " samples from internal flash", num_to_send);
                    uint32_t start_time = gs_time_rel_ms();

                    // staging buffer
                    uint8_t txbuf[BLOCK_SIZE];
                    size_t buf_used = 0;
                    size_t total_bytes_planned = 0;
                    size_t total_bytes_sent = 0;
                    int valid_sample_count = 0;

                    // normalize writer
                    uint32_t write_idx = (radfet_metadata.flash_write_offset / PKT_SIZE) % RING_CAP_PACKETS;
                    uint32_t start_idx = (write_idx + RING_CAP_PACKETS - num_to_send) % RING_CAP_PACKETS;

                    for (uint32_t i = 0; i < num_to_send; i++) {
                        uint32_t pkt_idx = (start_idx + i) % RING_CAP_PACKETS;
                        uint32_t offset  = pkt_idx * PKT_SIZE;
                        void *read_addr  = (uint8_t *)RADFET_FLASH_START + offset;

                        radfet_packet_t pkt;
                        err = gs_mcu_flash_read_data(&pkt, read_addr, PKT_SIZE);
                        if (err != GS_OK) {
                            log_error("Flash read failed @ offset %u: %s",
                                      (unsigned)offset, gs_error_string(err));
                            continue;
                        }

                        uint16_t crc = crc16_ccitt(&pkt, PKT_SIZE - sizeof(pkt.crc16));
                        if (crc != pkt.crc16) {
                            log_error("Skipping invalid packet @ offset %u (CRC mismatch)", (unsigned)offset);
                            continue;
                        }

                        valid_sample_count++;
                        total_bytes_planned += PKT_SIZE;

                        // append into tx buffer
                        const uint8_t *p = (const uint8_t *)&pkt;
                        size_t remaining = PKT_SIZE;

                        while (remaining > 0) {
                            wdt_clear();

                            size_t space   = BLOCK_SIZE - buf_used;
                            size_t to_copy = (remaining < space) ? remaining : space;

                            memcpy(txbuf + buf_used, p, to_copy);
                            buf_used  += to_copy;
                            p         += to_copy;
                            remaining -= to_copy;

                            if (buf_used == BLOCK_SIZE) {
                                size_t block_sent = 0;
                                err = gs_uart_write_buffer(USART1, 1000, txbuf, BLOCK_SIZE, &block_sent);
                                if (err != GS_OK || block_sent == 0) {
                                    log_error("UART write error at %u/%u planned bytes: %s (sent %u of 64)",
                                              (unsigned int)total_bytes_sent,
                                              (unsigned int)total_bytes_planned,
                                              gs_error_string(err),
                                              (unsigned int)block_sent);
                                    goto TX_FINISH;
                                }
                                total_bytes_sent += block_sent;

                                if (block_sent < BLOCK_SIZE) {
                                    memmove(txbuf, txbuf + block_sent, BLOCK_SIZE - block_sent);
                                    buf_used = BLOCK_SIZE - block_sent;
                                } else {
                                    buf_used = 0;
                                }
                            }

                            gs_time_sleep_ms(25); // pacing
                        }
                    }

                    // flush tail
                    if (buf_used > 0) {
                        size_t block_sent = 0;
                        err = gs_uart_write_buffer(USART1, 1000, txbuf, buf_used, &block_sent);
                        if (err != GS_OK || block_sent == 0) {
                            log_error("UART tail flush error: %s (wanted %u, sent %u)",
                                      gs_error_string(err),
                                      (unsigned int)buf_used,
                                      (unsigned int)block_sent);
                            goto TX_FINISH;
                        }
                        total_bytes_sent += block_sent;

                        if (block_sent < buf_used) {
                            memmove(txbuf, txbuf + block_sent, buf_used - block_sent);
                        }
                        buf_used = (block_sent < buf_used) ? (buf_used - block_sent) : 0;
                    }

                    err = GS_OK;

                TX_FINISH:
                    if (err == GS_OK && total_bytes_sent == total_bytes_planned) {
                        log_info("Downlink complete: %d valid samples, %u bytes sent in 64-byte blocks",
                                 valid_sample_count, (unsigned int)total_bytes_sent);
                    } else {
                        log_error("Downlink incomplete: sent %u of %u bytes (%d valid samples)",
                                  (unsigned int)total_bytes_sent,
                                  (unsigned int)total_bytes_planned,
                                  valid_sample_count);
                    }

                    uint32_t total_elapsed = gs_time_diff_ms(start_time, gs_time_rel_ms());
                    log_info("Transmission took %u ms", (unsigned int)total_elapsed);
                    break;
                }
                default:
                    break;
            }
        } else if (err == GS_ERROR_TIMEOUT) {
            // idle
        } else {
            log_error("UART1 read failed with error: %d", err);
        }
    }

    gs_thread_exit(NULL);
}

void mode_op_init(void) {
    log_info("Operation Modes initialization");

    gs_uart_config_t uart_conf;
    gs_uart_get_default_config(&uart_conf);
    uart_conf.comm.bps = 57600;

    gs_error_t err = gs_a3200_uart_init(USART1, true, uart_conf.comm.bps);
    if (err != GS_OK) {
        log_error("USART1 initialization failed: %s (code: %d)", gs_error_string(err), err);
    }

    err = gs_thread_create("Operation Modes", task_mode_op, NULL,
                           gs_a3200_get_default_stack_size(),
                           GS_THREAD_PRIORITY_NORMAL, 0, NULL);

    if (err != GS_OK) {
        log_error("Operation Modes thread creation FAILED: %s (%d)", gs_error_string(err), err);
    } else {
        log_info("Operation Modes thread successfully created");
    }
}
