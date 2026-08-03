/*
 * led.c — FreeRTOS task for the status LED (GPIO22)
 *
 * Unchanged from smart_switch/main/led.c (see the note in led.h about
 * verifying the physical wiring for the PCIe project).
 */

#include "led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static volatile led_state_t s_state = LED_BOOT;

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                   */
/* ------------------------------------------------------------------ */

static inline void led_on(void)
{
    gpio_set_level(LED_GPIO, 1);
}

static inline void led_off(void)
{
    gpio_set_level(LED_GPIO, 0);
}

/* Single blink: on for on_ms, off for off_ms */
static void blink(uint32_t on_ms, uint32_t off_ms)
{
    led_on();
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    led_off();
    vTaskDelay(pdMS_TO_TICKS(off_ms));
}

/* ------------------------------------------------------------------ */
/*  LED task                                                            */
/* ------------------------------------------------------------------ */

static void led_task(void *pv)
{
    for (;;) {
        switch (s_state) {

        case LED_BOOT:
            /* Continuous fast blink - firmware initializing */
            blink(100, 100);
            break;

        case LED_ZIGBEE_SEARCHING:
            /* Double blink every ~2s - searching / connecting to the Zigbee network */
            blink(100, 100);
            blink(100, 1700);
            break;

        case LED_OPERATIONAL:
            /* Short single blink every 5s - heartbeat, everything OK */
            blink(100, 4900);
            break;

        case LED_OTA_IN_PROGRESS:
            /* Slow 500ms on/off - firmware download in progress */
            blink(500, 500);
            break;

        case LED_ERROR:
            /* Three fast blinks every 3s - error detected */
            blink(100, 100);
            blink(100, 100);
            blink(100, 2600);
            break;

        case LED_FACTORY_RESET:
            /* Solid on - factory reset in progress */
            led_on();
            vTaskDelay(pdMS_TO_TICKS(100));
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    led_off();

    xTaskCreate(led_task, "led_task", 2048, NULL, 2, NULL);
}

void led_set_state(led_state_t state)
{
    s_state = state;
}
