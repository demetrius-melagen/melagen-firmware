/* Copyright (c) 2013-2018 GomSpace A/S. All rights reserved. */
/**
 * NanoMind Board Support Package
 */

#include <board.h>
#include <gs/a3200/a3200.h>
#include <gs/a3200/pwm.h>
#include <gs/a3200/hmc5843.h>
#include <gs/a3200/mpu3300.h>
#include <gs/a3200/spi_slave.h>
#include <gs/a3200/pwr_switch.h>
#include <gs/a3200/adc_channels.h>
#include <gs/a3200/lm71.h>
#include <gs/util/gosh/command.h>
#include <gs/util/gosh/console.h>
#include <gs/util/time.h>
#include <gs/thirdparty/fram/fm33256b.h>
#include <gs/thirdparty/flash/spn_fl512s.h>
#include <gs/embed/command.h>
#include <conf_a3200.h>  // a3200 options - set via wscript
#include "checkout/checkout_cmd.h"

// Setup locking/protection of FRAM region - lock/unlock entire region
static const gs_fm33256b_vmem_driver_data_t fm33256b_lock = {
    .device = 0,
    .protect = GS_FM33256B_PROTECT_ALL,
};

// Setup VMEM
static const gs_vmem_t vmem_map[] = {
    // FRAM layout: fm33256b - 32KB, 0x0000 - 0x7FFF
    {.name = "fram", .virtmem.u = 0x10000000, .physmem.u = 0x0000, .size = 0x8000, .drv = &gs_fm33256b_vmem_driver, .drv_data = &fm33256b_lock},
    {.name = NULL}, // end-of-entries, do not remove
};

// Overwrite default function from liba3200, reducing default stack size, so software can run from internal memory
size_t gs_a3200_get_default_stack_size(void)
{
    return 5000;
}

// Configure SPI channels to Flash SPN FL512S
static void configure_flash(void)
{
    // Turn on power to the flash
    gs_a3200_pwr_switch_enable(GS_A3200_PWR_SD);

    // Initialize SPI for accessing both dies on the flash
    const spn_fl512s_config_t config = {
        .bps = 8000000,
        .cs_part_0 = SPN_FL512S_CS0,
        .cs_part_1 = SPN_FL512S_CS1,
        .spi_slave_part_0 = GS_A3200_SPI_SLAVE_SPN_FL512_0,
        .spi_slave_part_1 = GS_A3200_SPI_SLAVE_SPN_FL512_1,
        .spi_handle = 1,
    };
    gs_error_t error = spn_fl512s_init(&config);
    if (error) {
        log_warning("failed to initialize flash driver, error=%s", gs_error_string(error));
    }

    gs_spn_fl512s_register_commands();
}

/**
   Hook called from gs_a3200_run(), when basic initializaton of devices are done.
*/
static void hook_init_complete(void)
{
    // Setup VMEM (virtuel memory) mapping
    gs_vmem_set_map(vmem_map);

    // Initialize command (GOSH) framework with a stack-size of 10000 bytes.
    gs_command_init(10000);

    // Start LED task - toggles the LEDs and touches the watchdog to prevent reset
    extern void led_init(void);
    led_init();

    // LM71 temperature sensors - initialized by gs_a3200_run()
    gs_a3200_lm71_register_commands();

    // Flash
    configure_flash();

    // Register commands for On-board devices and generel debug.
    // Gyro
    gs_mpu3300_init(GS_MPU3300_BW_5, GS_MPU3300_FSR_225);
    gs_mpu3300_register_commands();

    // Magnetometer
    gs_hmc5843_init();
    gs_hmc5843_register_commands();

    // FRAM - initialized by gs_a3200_run()
    gs_fm33256b_register_commands();

    // ADC - initialized by gs_a3200_run()
    gs_a3200_adc_register_commands();

    // PWM
    gs_a3200_pwm_register_commands();

    // PWR switches
    gs_a3200_pwr_register_commands();

    // Misc. commands - useful for debugging
    gs_embed_register_commands();
    gs_vmem_register_commands();

    // Checkout test commands - used for production checkout
    gs_checkout_register_commands();

    // Unlock FRAM and clear (zero) all content
    const gs_vmem_t * fram = gs_vmem_get_by_name("fram");
    if (fram) {
        gs_vmem_lock_by_name(fram->name, false);
        uint8_t wtmp[128];
        memset(wtmp, 0, sizeof(wtmp));
        for (unsigned int i = 0; i < fram->size; i += sizeof(wtmp)) {
            gs_vmem_cpy(fram->virtmem.p + i, &wtmp, sizeof(wtmp));
        }
        log_info("All FRAM set to zero (cleared)");
    }

    // Start command console on standard I/O (uart).
    gs_console_start(GS_A3200_HOSTNAME, 0);

    // Configure CSP
    extern void configure_csp(void);
    configure_csp();

    // Start radfet poll task - polls sensors at specified values
    extern void radfet_task_init(void);
    radfet_task_init();

    extern void mode_op_init(void);
    mode_op_init();
}//

/**
   Hooks to the A3200 system boot process.
*/
static const gs_a3200_hooks_t hooks = {
    .init_complete = hook_init_complete,
};

int main()
{
    // Initiaize: clock, watchdog - use interal memory for heap.
    gs_a3200_init_set_heap(false);

    // Spawns a task to complete the initialization and launch application tasks through hooks.
    gs_a3200_run(&hooks);

    // Will never get here, as gs_a3200_run() never returns
}
