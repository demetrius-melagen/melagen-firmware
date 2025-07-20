/* Copyright (c) 2013-2019 GomSpace A/S. All rights reserved. */

#include <gs/a3200/a3200.h>
#include <gs/csp/csp.h>
#include <gs/csp/router.h>
#include <gs/csp/command.h>
#include <gs/csp/service_dispatcher.h>
#include <gs/csp/drivers/can/can.h>
#include <gs/csp/drivers/i2c/i2c.h>
#include <gs/csp/drivers/kiss/kiss.h>
#include <gs/embed/asf/drivers/i2c/i2c.h>
#include <gs/embed/asf/avr32/drivers/can/can.h>
#include <gs/util/vmem.h>
#include <gs/util/thread.h>
#include <conf_a3200.h>  // a3200 options - set via wscript

// Configure CSP to use CAN and I2C
void configure_csp(void)
{
    // Initialize CSP with address 1
    {
        gs_csp_conf_t conf;
        gs_csp_conf_get_defaults_embedded(&conf);
        conf.hostname = GS_A3200_HOSTNAME;
        conf.model = GS_A3200_MODEL;
        conf.revision = GS_A3200_REVISION;
        conf.csp_buffers = 5;
        conf.address = 1;
        gs_csp_init(&conf);
    }

    csp_cmp_set_memcpy((csp_memcpy_fnc_t) gs_vmem_cpy);

    // Add I2C interface
    gs_a3200_init_twi(0, GS_AVR_I2C_MULTIMASTER, csp_get_address(), 400000);
    gs_csp_i2c_init(0, csp_get_address());

    // Add CAN interface
    gs_a3200_init_can(0, true, 1000000, 0);
    gs_csp_can_init2(0, csp_get_address(), 0, NULL, false, NULL);

    // Add KISS interface
    gs_csp_kiss_init(4);

    // Add routes
    const char * rtable = "15/5 KISS, 10/5 I2C, 0/0 CAN";
    gs_csp_rtable_load(rtable, true, true);

    // Start CSP service dispatch task -handles incoming CSP request, e.g. ping.
    {
        static const gs_csp_service_handler_t handlers[10] = {
            GS_CSP_BASIC_SERVICE_HANDLERS,
        };

        static const gs_csp_service_dispatcher_conf_t conf = {
            .name = "CSP-DISP",
            .bind_any = true, // bind to any un-bounded ports, used for logging unexpected connections
            .handler_array = handlers,
            .handler_array_size = GS_ARRAY_SIZE(handlers),
            .disable_watchdog = true,
        };

        gs_csp_service_dispatcher_create(&conf, gs_a3200_get_default_stack_size(), GS_THREAD_PRIORITY_NORMAL, NULL);
    }

    // Start CSP router task, routes incoming messages to tasks, e.g. CSP service dispatch task
    gs_csp_router_task_start(gs_a3200_get_default_stack_size(), GS_THREAD_PRIORITY_HIGH);

    gs_csp_register_commands();
}
