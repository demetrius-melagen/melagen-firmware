# melagen-firmware
Embedded firmware for the NanoMind A3200 on-board computer (OBC), developed by Melagen Labs for autonomous radiation dosimetry in space environments.

This firmware interfaces with five Varadis VT01 RADFET sensors via an I2C-controlled GPIO expander, buffers samples into non-volatile FRAM, and supports RS-422 uplink/downlink using the THVD4421 transceiver and CubeSat Space Protocol (CSP) over UART.


---

## Features

- Polls 5x RADFET dosimeters using external ADCs
- I2C expander (TCA9539) used for sensor bias enable/disable
- Periodic sampling with timestamped ADC measurements
- Internal Flash Memory circular buffer for non-volatile logging (`radfet_sample_t`)
- CSP interface for remote data dump and control
- RS-422-compatible packet structure for satellite downlink
- Watchdog integration for autonomous resets
---

This repository currently contains files changed or added in the src directory of the Board Support Package provided by GOMSpace.
