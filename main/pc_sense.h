/*
 * pc_sense.h — Reading the PC's real power state on GPIO1
 *
 * GPIO1 is connected (protection is the user's own responsibility,
 * already accounted for on the schematic) to the "standard" 3.3V of the
 * PCIe slot (main rail, present only when the motherboard is powered on
 * in S0 - different from the 3.3Vaux that powers the ESP32 even in S5).
 *
 * Polarity assumption: HIGH = PC on, LOW = PC off.
 * Verify it on the schematic/with a multimeter before the first flash: if
 * inverted, just swap the two branches in pc_sense_task(), or invert the
 * reading in pc_sense_read_raw().
 *
 * The module samples the GPIO every PC_SENSE_POLL_MS and only considers
 * the state "stable" after PC_SENSE_DEBOUNCE_MS of a constant level, to
 * ignore ATX power supply transients during power-on/power-off.
 *
 * Self-correction: on every debounce cycle, if the current stable level
 * differs from the last value actually written to the Zigbee attribute,
 * the callback is invoked again - this covers both genuine state changes
 * and correcting any attribute forced by an incoming ON/OFF command that
 * didn't actually change the real PC state (main.c also re-asserts the
 * true state right after processing a command, see handle_on_off() in
 * main.c, so in practice the correction there is usually immediate; this
 * loop is the safety net that keeps working even for state changes that
 * happen independently of any command).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Callback invoked when the PC's real (debounced) state changes compared
 * to the last value reported over Zigbee. */
typedef void (*pc_sense_change_cb_t)(bool pc_on);

/** Initializes the sense GPIO and starts the polling/debounce task. */
void pc_sense_init(pc_sense_change_cb_t cb);

/** Last known stable state (used for the keepalive's periodic resync). */
bool pc_sense_get_state(void);
