/*
 * led.h — Status LED handling on GPIO22
 *
 * WARNING: GPIO22 is the standard pin for the PCB A/B variants of the
 * ESP32-H2-Custom-Zigbee family (4ch relay and single-relay). The PCIe
 * project's hardware README doesn't explicitly document a status LED
 * wired to GPIO22 - check the PCIe-PC-Power-Control schematic to confirm
 * the LED listed in the BOM is really on this pin before the first flash.
 * If the PCB has no status LED wired up, this module still drives a
 * harmless GPIO (no visible effect, no risk).
 *
 * Blink patterns:
 *   LED_BOOT              -> fast blink 100ms (firmware starting up)
 *   LED_ZIGBEE_SEARCHING  -> double blink every 2s (searching / steering)
 *   LED_OPERATIONAL       -> single blink every 5s (operational, heartbeat)
 *   LED_OTA_IN_PROGRESS   -> slow 500ms on/off (firmware download)
 *   LED_ERROR             -> 3 fast blinks every 3s (problem)
 *   LED_FACTORY_RESET     -> solid on (reset in progress)
 */

#pragma once

#define LED_GPIO 22

typedef enum {
    LED_BOOT = 0,
    LED_ZIGBEE_SEARCHING,
    LED_OPERATIONAL,
    LED_OTA_IN_PROGRESS,
    LED_ERROR,
    LED_FACTORY_RESET,
} led_state_t;

/**
 * @brief Initializes the GPIO and starts the LED's FreeRTOS task.
 *        Call once in app_main(), before the Zigbee stack.
 */
void led_init(void);

/**
 * @brief Changes the LED state. Thread-safe.
 *        The task picks it up on the next cycle without blocking the caller.
 */
void led_set_state(led_state_t state);
