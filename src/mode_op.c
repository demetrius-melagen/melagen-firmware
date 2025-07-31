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

#define USART1 1
#define STX 0x32
#define ETX 0x03
 
static const gs_vmem_t *fram = NULL;
// uint32_t fram_write_offset = 0;

static void * task_mode_op(void * param)
{
    // log_info("UART test task started!");
    for (;;) {
        // Touch watchdog to prevent reset.
        // This should be tied into other tasks as well, to ensure everything is running.
        wdt_clear();
        uint8_t incoming_byte;
        gs_error_t err = gs_uart_read(USART1, 1000, &incoming_byte);  
        if (err == GS_OK) {
            log_info("Received byte on USART1: 0x%02X (%c)", incoming_byte, incoming_byte);
            // gs_uart_write(USART1, 10000, incoming_byte);
            // state machine with switch case
            // if received byte is STX
            switch (incoming_byte){
                case STX:
                    log_info("STX received: sending last two samples from FRAM");
                    for (int i = 2; i > 0; i--) {
                        int32_t offset = (int32_t)fram_write_offset - i * PKT_SIZE;
                        if (offset < 0) {
                            offset += fram->size;
                        }
                        radfet_packet_t pkt;
                        gs_vmem_cpy(&pkt, fram->virtmem.p + offset, sizeof(pkt));

                        uint16_t crc = crc16_ccitt(&pkt, sizeof(pkt) - sizeof(pkt.crc16));
                        if (crc != pkt.crc16) {
                            log_error("CRC mismatch @ offset %d: expected 0x%04X, got 0x%04X", (int)offset, crc, (unsigned int)pkt.crc16);
                            continue;
                        }

                        log_info("Sending sample with timestamp %u", (unsigned int)pkt.sample.timestamp);

                        size_t bytes_sent = 0;
                        gs_error_t tx_err = gs_uart_write_buffer(USART1, 1000, (uint8_t *)&pkt, sizeof(pkt), &bytes_sent);
                        if (tx_err == GS_OK && bytes_sent == sizeof(pkt)) {
                            log_info("Successfully sent packet (%u bytes)", (unsigned int)bytes_sent);
                        } else {
                            log_error("Failed to send packet: error %d, sent %u bytes", tx_err, (unsigned int)bytes_sent);
                        }
                    }
                    break;
                case ETX:
                    // if received byte is ETX
                    // tunnel is closed

                    break;
                // if received byte is (safe mode)
                // put radfet data collection task to sleep
                // 
            }
            
        } else if (err == GS_ERROR_TIMEOUT) {
            // log_info("UART1 read timeout");
        } else {
            log_error("UART1 read failed with error: %d", err);
        }
    }
    // Will never get here
    gs_thread_exit(NULL);
}

void mode_op_init(void)
{
    // log_info("UART test init starting...");

    fram = gs_vmem_get_by_name("fram");
    if (!fram) {
        log_error("FRAM not found!");
        return;
    }
    log_info("FRAM found at address %x", (unsigned int)fram->virtmem.p);

    gs_uart_config_t uart_conf;
    gs_uart_get_default_config(&uart_conf);
    uart_conf.comm.bps = 57600;
    gs_error_t err = gs_a3200_uart_init(USART1, true, uart_conf.comm.bps);
    if (err != GS_OK) {
        log_error("USART1 initialization failed: %s (code: %d)", gs_error_string(err), err);
    }

    gs_error_t thread_err = gs_thread_create("Operation Modes", task_mode_op, NULL,
                     gs_a3200_get_default_stack_size(), GS_THREAD_PRIORITY_NORMAL, 0, NULL);

    if (thread_err != GS_OK) {
        log_error("Operation Modes thread creation FAILED: %s (%d)", gs_error_string(thread_err), thread_err);
    } else {
        log_info("Operation Modes thread successfully created");
    }
}
