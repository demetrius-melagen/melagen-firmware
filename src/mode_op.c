#include <gs/util/log.h>
#include <gs/util/time.h>
#include <gs/util/thread.h>
#include <wdt.h>  // libasf
#include <gs/a3200/a3200.h>
#include <gs/a3200/adc_channels.h>
#include <gs/util/thread.h>
#include <gs/util/types.h>
#include <gs/util/vmem.h>
#include <wdt.h>
#include <gs/util/drivers/i2c/master.h>  // for I2C transactions
#include <inttypes.h>
#include <string.h>  // for memset
#include "radfet.h"
#include <gs/util/clock.h>
#include <gs/util/rtc.h>
#include <gs/embed/drivers/uart/uart.h>
#include <gs/a3200/uart.h>
#include <gs/csp/csp.h>
#include <gs/csp/drivers/kiss/kiss.h>
#include <gs/a3200/uart.h>
#include <gs/thirdparty/flash/spn_fl512s.h>
#include <gs/embed/asf/drivers/spi/master.h>
#include <gs/util/time.h>
#include <gs/util/string.h>
#include <gs/util/mutex.h>
#include <gs/embed/drivers/flash/mcu_flash.h>

#define USART1 1
#define STX 0x02
// #define ETX 0x03
#define NUM_SAMPLES_TO_SEND 60 * 24 * 5
#define BYTES_TO_SEND (NUM_SAMPLES_TO_SEND * PKT_SIZE)
// OOX specified 64 bytes to send at a time based on stress testing their rs422 link
#define BLOCK_SIZE 64

static void * task_mode_op(void * param)
{

    uint8_t incoming_byte;
    uint32_t num_to_send;
    gs_error_t err;
    for (;;) {
        // Touch watchdog to prevent reset.
        // This should be tied into other tasks as well, to ensure everything is running.
        wdt_clear();
        
        err = gs_uart_read(USART1, 1000, &incoming_byte);  
        if (err == GS_OK) {
            log_info("Received byte on USART1: 0x%02X", incoming_byte);
            // state machine with switch case
            // if received byte is STX
            switch (incoming_byte){
                case STX: 
                    // if (!radfet_polling) {
                    num_to_send = (radfet_metadata.samples_saved < NUM_SAMPLES_TO_SEND)
                                    ? radfet_metadata.samples_saved
                                    : NUM_SAMPLES_TO_SEND;
                    log_info("STX received: sending up to %" PRIu32 " samples from internal flash", num_to_send);
                    uint32_t start_time = gs_time_rel_ms();

                    // --- 64-byte staging buffer ---
                    uint8_t txbuf[BLOCK_SIZE];
                    size_t buf_used = 0;              // bytes currently buffered (<= BLOCK_SIZE)
                    size_t total_bytes_planned = 0;   // sum of valid packets * PKT_SIZE
                    size_t total_bytes_sent = 0;
                    int    valid_sample_count = 0;

                    for (uint32_t i = 0; i < num_to_send; i++) {
                        uint32_t packet_index = i;
                        if (packet_index >= radfet_metadata.samples_saved) continue;

                        int32_t offset = (int32_t)radfet_metadata.flash_write_offset
                                    - ((radfet_metadata.samples_saved - packet_index) * PKT_SIZE);
                        if (offset < 0) offset += RADFET_FLASH_SIZE - (RADFET_FLASH_SIZE % PKT_SIZE);

                        void *read_addr = (uint8_t *)RADFET_FLASH_START + offset;
                        radfet_packet_t pkt;  // read one packet at a time on the stack

                        err = gs_mcu_flash_read_data(&pkt, read_addr, PKT_SIZE);
                        if (err != GS_OK) {
                            log_error("Flash read failed at offset %" PRId32 ": %s", offset, gs_error_string(err));
                            continue;
                        }

                        uint16_t crc = crc16_ccitt(&pkt, PKT_SIZE - sizeof(pkt.crc16));
                        if (crc != pkt.crc16) {
                            log_error("Skipping invalid packet @ offset %" PRId32 " (CRC mismatch)", offset);
                            continue;
                        }

                        valid_sample_count++;
                        total_bytes_planned += PKT_SIZE;

                        // Append this packet into the 64-byte buffer; flush when full
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
                                            (unsigned int)total_bytes_sent, (unsigned int)total_bytes_planned,
                                            gs_error_string(err), (unsigned int)block_sent);
                                    goto TX_FINISH; // abort transmit loop, finalize below
                                }
                                total_bytes_sent += block_sent;

                                if (block_sent < BLOCK_SIZE) {
                                    // shift any unsent tail down (rare)
                                    memmove(txbuf, txbuf + block_sent, BLOCK_SIZE - block_sent);
                                    buf_used = BLOCK_SIZE - block_sent;
                                } else {
                                    buf_used = 0;
                                }

                            }
                            gs_time_sleep_ms(25); //25 ms delay between 64 byte blocks
                        }
                    }

                    // Flush any tail (<64 bytes) after all packets
                    if (buf_used > 0) {
                        size_t block_sent = 0;
                        err = gs_uart_write_buffer(USART1, 1000, txbuf, buf_used, &block_sent);
                        if (err != GS_OK || block_sent == 0) {
                            log_error("UART tail flush error: %s (wanted %u, sent %u)",
                                    gs_error_string(err), (unsigned int)buf_used, (unsigned int)block_sent);
                            goto TX_FINISH;
                        }
                        total_bytes_sent += block_sent;

                        if (block_sent < buf_used) {
                            // shift remainder (very unlikely on a stable link)
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
                                (unsigned int)total_bytes_sent, (unsigned int)total_bytes_planned, valid_sample_count);
                    }


                    uint32_t total_elapsed = gs_time_diff_ms(start_time, gs_time_rel_ms());
                    log_info("Transmission took %u ms", (unsigned int)total_elapsed);
                    // }
                    break;

                // case ETX:
                    // log_info("Data transmission successful!");
                    //save amount of successful transmissions to metadata?
                    // break;
            }
        } else if (err == GS_ERROR_TIMEOUT) {
        } else {
            log_error("UART1 read failed with error: %d", err);
        }
    }
    // Will never get here
    gs_thread_exit(NULL);
}

void mode_op_init(void)
{
    gs_uart_config_t uart_conf;
    gs_uart_get_default_config(&uart_conf);
    uart_conf.comm.bps = 57600;
    gs_error_t err = gs_a3200_uart_init(USART1, true, uart_conf.comm.bps);
    if (err != GS_OK) {
        log_error("USART1 initialization failed: %s (code: %d)", gs_error_string(err), err);
    }

    err = gs_thread_create("Operation Modes", task_mode_op, NULL,
                     gs_a3200_get_default_stack_size(), GS_THREAD_PRIORITY_NORMAL, 0, NULL);

    if (err != GS_OK) {
        log_error("Operation Modes thread creation FAILED: %s (%d)", gs_error_string(err), err);
    } else {
        log_info("Operation Modes thread successfully created");
    }
}
