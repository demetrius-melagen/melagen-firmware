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
#define STX 0x02
#define ETX 0x03

static void * task_uart_test(void * param)
{
    for (;;) {
        // Touch watchdog to prevent reset.
        // This should be tied into other tasks as well, to ensure everything is running.
        wdt_clear();
        uint8_t incoming_byte;
        gs_error_t err = gs_uart_read(USART1, 1000, &incoming_byte);  
        if (err == GS_OK) {
            log_info("Received byte on UART4: 0x%02X (%c)", incoming_byte, incoming_byte);
            // gs_uart_write(USART1, 10000, incoming_byte);
            // state machine with switch case
            // if received byte is STX
            if (incoming_byte == STX){
                // tunnel is open
                // send X days worth of data 
                
            }
            // if received byte is ETX
                // tunnel is closed
            // if received byte is (safe mode)
                // put radfet data collection task to sleep
                // 
        } else if (err == GS_ERROR_TIMEOUT) {
            // log_info("UART1 read timeout");
        } else {
            log_error("UART1 read failed with error: %d", err);
        }
    }
    // Will never get here
    gs_thread_exit(NULL);
}

void uart_test_init(void)
{
    gs_uart_config_t uart_conf;
    gs_uart_get_default_config(&uart_conf);
    uart_conf.comm.bps = 57600;
    gs_error_t err = gs_a3200_uart_init(USART1, true, uart_conf.comm.bps);
    if (err != GS_OK) {
        log_error("UART1 initialization failed: %s (code: %d)", gs_error_string(err), err);
    }
    gs_thread_create("UART Test", task_uart_test, NULL,
                     gs_a3200_get_default_stack_size(),
                     GS_THREAD_PRIORITY_LOW, 0, NULL);
}
