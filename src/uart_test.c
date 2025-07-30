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
#include <gs/a3200/uart.h>

static void * task_uart_test(void * param)
{
    for (;;) {
        // Touch watchdog to prevent reset.
        // This should be tied into other tasks as well, to ensure everything is running.
        wdt_clear();
        // log_info("Status of UART Device 4", gs_uart_status(4));
        uint8_t incoming_byte;
        gs_error_t res = gs_uart_read(4, 1000, &incoming_byte);  
        if (res == GS_OK) {
            log_info("Received byte on UART4: 0x%02X (%c)", incoming_byte, incoming_byte);
            // if received byte is STX
                // tunnel is open
                // send X days worth of data 
            // if received byte is ETX
                // tunnel is closed
            // if received byte is (safe mode)
        } else if (res == GS_ERROR_TIMEOUT) {
            // log_info("UART4 read timeout");
        } else {
            log_error("UART4 read failed with error: %d", res);
        }

        // gs_uart_write(4, 10000, 'A');
        // log_info("Writing A to UART");
    }
    // Will never get here
    gs_thread_exit(NULL);
}

void uart_test_init(void)
{
    
    // gs_uart_config_t uart_conf;
    // gs_uart_get_default_config(&uart_conf);
    // uart_conf.comm.bps = 57600;
    // gs_uart_init(4, &uart_conf);
    // gs_uart_change_comm(4,&uart_conf.comm);

    // log_info("UART config:");
    // log_info("  Baudrate:     %"PRIu32, uart_conf.comm.bps);
    // log_info("  Data bits:    %u", (unsigned int)uart_conf.comm.data_bits);
    // log_info("  Stop bits:    %u", (unsigned int)uart_conf.comm.stop_bits);
    // log_info("  Parity:       %u", (unsigned int)uart_conf.comm.parity_bit);     
    // log_info("  Flow control: %u", (unsigned int)uart_conf.comm.flow_control);    
    // log_info("  TX queue size: %u", (unsigned int)uart_conf.tx_queue_size);
    // log_info("  RX queue size: %u", (unsigned int)uart_conf.rx_queue_size);

    gs_thread_create("UART Test", task_uart_test, NULL,
                     gs_a3200_get_default_stack_size(),
                     GS_THREAD_PRIORITY_LOW, 0, NULL);
}
