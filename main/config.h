/*
 * config.h — Hardware constants for the pcie_power_control firmware
 *
 * Firmware DEDICATED to the PCIe-PC-Power-Control-ESP32-H2 project.
 * No runtime configuration (no NVS wizard/configure.py): all parameters
 * are fixed here, per the project spec.
 *
 * Channels:
 *   CH_POWER (0): out=GPIO4 -> MOSFET PWR_SW, 300ms pulse. Commanded by
 *                 Zigbee on BOTH ON and OFF commands (every command, in
 *                 either direction, counts as "press the power button"
 *                 once - same behavior as a real ATX button).
 *                 Its On/Off attribute does NOT reflect the pulse, but the
 *                 real state of the PC read on GPIO1 (see pc_sense.c).
 *   CH_RESET (1): out=GPIO5 -> MOSFET RESET, 300ms pulse, commanded by
 *                 Zigbee only on ON (OFF command ignored).
 *                 Standard behavior: attribute ON during the pulse,
 *                 automatically back to OFF at the end of the pulse -
 *                 never stuck ON.
 *
 * Sense:
 *   GPIO1: real PC state input. Connected (through the hardware
 *   protection already provided on the schematic, user's own design) to
 *   the "standard" 3.3V of the PCIe slot (main rail, NOT the 3.3Vaux that
 *   powers the ESP32 even in S5). That rail is present only when the
 *   motherboard is actually powered on (S0): HIGH = PC on, LOW = PC
 *   off/S5. Verify this polarity assumption on the schematic before the
 *   first flash.
 */

#pragma once

#include "driver/gpio.h"

/* -- Relay channels (fixed indices) ------------------------------------- */
#define CH_POWER   0
#define CH_RESET   1
#define NUM_CH     2

/* -- Fixed hardware GPIOs ------------------------------------------------ */
#define RELAY_OUT_GPIO_POWER   GPIO_NUM_4
#define RELAY_OUT_GPIO_RESET   GPIO_NUM_5
#define PC_SENSE_GPIO          GPIO_NUM_1

/* -- Timing --------------------------------------------------------------- */
#define RELAY_PULSE_MS         300     /* PWR_SW / RESET pulse duration (ms)     */
#define PC_SENSE_DEBOUNCE_MS   2000    /* PC state reading stabilization (ms)    */
#define PC_SENSE_POLL_MS       100     /* GPIO1 sampling period (ms)             */
