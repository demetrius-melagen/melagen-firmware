/*Ground communication task
Using deferred interrupt handling, sends data saved in FRAM through KISS interface connected to THVD4421RHBR
Current configuration is automatic to debug
*/

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
#include <gs/embed/asf/drivers/uart/uart.h>
#include <gs/util/clock.h>
#include <gs/util/rtc.h>
#include <gs/embed/drivers/uart/uart.h>
#include <gs/csp/csp.h>
#include <gs/csp/drivers/kiss/kiss.h>

#define COMM_BAUDRATE      57600  
#define NUM_SAMPLES 5
#define TWO_HOURS_MS (1000 * 60 * 60 * 2)
static const gs_vmem_t *fram = NULL;

static void * task_ground_comm(void * param) {
        uint32_t fram_read_offset = 0;
        for (;;) {
            // Touch watchdog to prevent reset.
            // This should be tied into other tasks as well, to ensure everything is running.
            wdt_clear();
            log_info("Comm Task Test @ %u",  (unsigned int)gs_time_rel_ms() / 1000);
            //listens for incoming request packets, custom command
            // 
            // implement deferred interrupt handling, where an interrupt will signal this task to go through
            // for (int i = 0; i < 5; i++){ 
                // log_info("UART Status of Device %i: %u",i, (unsigned int)gs_uart_status(i));
            // }
            // uint8_t incoming_byte;
            // gs_error_t res = gs_uart_read(4, 1000, &incoming_byte);  // Wait up to 1000 ms

            // if (res == GS_OK) {
                // log_info("Received byte on UART4: 0x%02X (%c)", incoming_byte, incoming_byte);

                radfet_packet_t pkt;
                
                //  for all samples
                while (fram_read_offset != fram_write_offset) {
                    // Handle wraparound if packet would exceed FRAM end
                    if (fram_read_offset + PKT_SIZE > fram->size) {
                        log_info("Partial packet at end — wrapping to 0");
                        fram_read_offset = 0;
                        continue;
                    }
                    // Log all samples
                    gs_vmem_cpy(&pkt, fram->virtmem.p + fram_read_offset, sizeof(pkt));
                    uint16_t expected = crc16_ccitt(&pkt, sizeof(pkt) - sizeof(pkt.crc16));
                    if (expected != pkt.crc16) {
                        log_error("CRC mismatch: expected 0x%04X, got 0x%04X", expected, pkt.crc16);
                    } else {
                        log_info("Correct packet CRC")
                    }
                    log_info("Sample @ offset %u",  (unsigned int)fram_read_offset);
                    log_info("  Timestamp: %u", (unsigned int)pkt.sample.timestamp);
                    for (int i = 0; i < NUM_RADFET; i++) {
                        log_info("  D%i R1 = %d mV, R2 = %d mV", i + 1, pkt.sample.adc[i][0], pkt.sample.adc[i][1]);
                    }
                    // create packet for transmission including
                        // payload sample packet of size 
                        // (timestamp size 4 bytes) + (number of sensors * number of radfets per sensor * 2 bytes) 
                        // header, command id, length, payload, checksum
                    // send out packet through rs-232 -> rs422 
                    // iterate read address pointer based on sample packet size
                    fram_read_offset += PKT_SIZE;
                    if (fram_read_offset >= fram->size) {
                        fram_read_offset = 0;
                        log_info("Read offset wrapped to 0");
                    }
                }
                //after transmission
                // clear FRAM
                if (fram) {
                    gs_vmem_lock_by_name(fram->name, false);
                    uint8_t wtmp[128];
                    memset(wtmp, 0, sizeof(wtmp));
                    for (unsigned int i = 0; i < fram->size; i += sizeof(wtmp)) {
                        gs_vmem_cpy(fram->virtmem.p + i, &wtmp, sizeof(wtmp));
                    }
                    log_info("All FRAM set to zero (cleared)");
                }
            
            // } else if (res == GS_ERROR_TIMEOUT) {
            //     log_info("UART4 read timeout");
            // } else {
            //     log_error("UART4 read failed with error: %d", res);
            // }
            // // reset fram_offset used in radfet.c
            // // only reset if at end of buffer
            // fram_write_offset = 0;
            // log_info("FRAM offset reset to 0");
            //automation debug to verify task management
            uint32_t tx_rate_ms = sample_rate_ms * NUM_SAMPLES;
            gs_time_sleep_ms(tx_rate_ms);
        }
           gs_thread_exit(NULL);
}


void comm_task_init(void) {
    log_info("Initializing USART <-> RS-422 comms at 57600 bps");
    // current configuration of UART to RS-485 Transceiver – THVD4421
    // on DIR pin: Enable RS232 to RS422/RS485
    // on MODE0 and MODE1: RS485 Mode set to RS485 Full Duplex
    // on TERM_RX and TERM_TX: RS422/RS485 Full Duplex Mode set to Enable 120ohm termination
    // on SLR: Slew rate set to 500kps 
    // slew rate = rate at which voltage changes
    // baud rate = rate at which bits are sent

    //set usart for rs-232 
    // usart_init_rs232(usart, &usart_conf,
			// sysclk_get_peripheral_bus_hz(usart));
    // usart_init(4, sysclk_get_cpu_hz(), 57600);
    
    gs_uart_config_t uart_conf;
    gs_uart_get_default_config(&uart_conf);
    uart_conf.comm.bps = 57600;
    log_info("Default UART config:");
    log_info("  Baudrate:     %"PRIu32, uart_conf.comm.bps);
    log_info("  Data bits:    %u", (unsigned int)uart_conf.comm.data_bits);
    log_info("  Stop bits:    %u", (unsigned int)uart_conf.comm.stop_bits);
    log_info("  Parity:       %u", (unsigned int)uart_conf.comm.parity_bit);     
    log_info("  Flow control: %u", (unsigned int)uart_conf.comm.flow_control);    
    log_info("  TX queue size: %u", (unsigned int)uart_conf.tx_queue_size);
    log_info("  RX queue size: %u", (unsigned int)uart_conf.rx_queue_size);

    // const char *msg = "Testing CSP KISS Interface\n";
    // for (int i = 0; msg[i]; i++) {
    //     gs_uart_write(4, -1, msg[i]);
    // }

    // if(gs_uart_init(4, &uart_conf) == GS_ERROR_IN_USE){
    //     log_info("Device 4 already in use");
    // } else {
    //     log_info("UART Device 4 Initialized"); 
    // }
   
    fram = gs_vmem_get_by_name("fram");
    gs_thread_create("COMM", task_ground_comm, NULL,
                    gs_a3200_get_default_stack_size(), GS_THREAD_PRIORITY_HIGH, 0, NULL);
}