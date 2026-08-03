/*
 * relay.h — Impulse outputs (Power / Reset) for pcie_power_control
 *
 * Only 2 fixed outputs, no physical input (the PCIe project has no
 * buttons wired to the ESP32 - GPIO1 is reserved for the real state
 * sense, see pc_sense.h).
 *
 * CH_POWER: "silent" pulse - does NOT notify main.c, because this
 *           channel's Zigbee attribute is driven exclusively by
 *           pc_sense.c (real PC state), not by the pulse itself.
 * CH_RESET: "standard" pulse - notifies main.c at the end of the pulse to
 *           bring the ZCL attribute back to OFF (classic impulse relay
 *           behavior, inherited from smart_switch).
 */

#pragma once

#include "config.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Callback called ONLY for CH_RESET when the pulse ends (-> false).
 * main.c uses it to bring the Reset channel's ZCL attribute back to OFF.
 * Never called for CH_POWER.
 */
typedef void (*relay_change_cb_t)(uint8_t ch, bool state);

/** Initializes the output GPIOs (Power, Reset). */
void relay_init(relay_change_cb_t cb);

/**
 * Starts a RELAY_PULSE_MS pulse on the given channel (CH_POWER or
 * CH_RESET). Called from the incoming Zigbee ON command. OFF commands are
 * ignored (impulse channels, consistent with smart_switch).
 */
void relay_pulse(uint8_t ch);

/** Current physical state of the output (true = pulse in progress). */
bool relay_get(uint8_t ch);
