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
#define ETX 0x03
#define NUM_SAMPLES_TO_SEND 60 * 24 * 5 // 5 days worth of data 
#define CHUNK_SIZE 100

static void * task_mode_op(void * param)
{

    log_info("UART test task started!");
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
                    num_to_send = (radfet_metadata.samples_saved < NUM_SAMPLES_TO_SEND) ? radfet_metadata.samples_saved : NUM_SAMPLES_TO_SEND;
                    log_info("STX received: sending up to %" PRIu32 " samples from internal flash", num_to_send);
                    uint32_t start_time = gs_time_rel_ms();

                    for (int chunk = 0; chunk < NUM_SAMPLES_TO_SEND; chunk += CHUNK_SIZE) {
                        wdt_clear(); 
                        // Check elapsed time before processing chunk
                        // uint32_t now = gs_time_rel_ms();
                        uint32_t samples_this_chunk = (num_to_send - chunk >= CHUNK_SIZE) ? CHUNK_SIZE : (num_to_send - chunk);
                        radfet_packet_t *packets = malloc(samples_this_chunk * PKT_SIZE);
                        
                        if (!packets) {
                            log_error("Failed to allocate memory for %u packets", (unsigned int)samples_this_chunk);
                            break;
                        }
                        int valid_sample_count = 0;
                        for (uint32_t i = 0; i < (uint32_t)samples_this_chunk; i++) {
                            uint32_t packet_index = chunk + i;
                            if (packet_index >= radfet_metadata.samples_saved) continue;
                            int32_t offset = (int32_t)radfet_metadata.flash_write_offset - ((radfet_metadata.samples_saved - packet_index) * PKT_SIZE);
                            if (offset < 0) offset += RADFET_FLASH_SIZE - (RADFET_FLASH_SIZE % PKT_SIZE);

                            void *read_addr = (uint8_t *)RADFET_FLASH_START + offset;
                            radfet_packet_t temp_pkt;
                            err = gs_mcu_flash_read_data(&temp_pkt, read_addr, PKT_SIZE);
                            if (err != GS_OK) {
                                log_error("Flash read failed at offset %" PRId32 ": %s", offset, gs_error_string(err));
                                continue;
                            }
                            uint16_t crc = crc16_ccitt(&temp_pkt, PKT_SIZE - sizeof(temp_pkt.crc16));
                            if (crc != temp_pkt.crc16) {
                                log_error("Skipping invalid packet @ offset %" PRId32 " (CRC mismatch)", offset);
                                continue;
                            }
                            packets[valid_sample_count++] = temp_pkt;
                        }
                        size_t bytes_to_send = valid_sample_count * PKT_SIZE;
                        size_t bytes_sent = 0;
                        err = gs_uart_write_buffer(USART1, 1000, (uint8_t *)packets, bytes_to_send, &bytes_sent);
                        free(packets);
                        if (err == GS_OK && bytes_sent == bytes_to_send) {
                            if (bytes_sent > 0){
                                log_info("Sent chunk %d: %u samples (%u bytes)", 
                                chunk / CHUNK_SIZE, (unsigned int)valid_sample_count, (unsigned int)bytes_sent);
                            }
                        } else {
                            log_error("Failed to send chunk %d: error %s, sent %u of %u bytes",
                                chunk / CHUNK_SIZE, gs_error_string(err), (unsigned int)bytes_sent, (unsigned int)bytes_to_send);
                            break;
                        }
                    }
                    uint32_t total_elapsed = gs_time_diff_ms(start_time, gs_time_rel_ms());
                    log_info("Downlink completed in %u ms", (unsigned int)total_elapsed);
                    break;
                case ETX:
                    log_info("Data transmission successful!");
                    //save amount of successful transmissions to metadata?
                    break;
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
