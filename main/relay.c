/*
 * relay.c — Power/Reset impulse outputs (pcie_power_control)
 *
 * Ported from smart_switch's relay.c, simplified: only 2 fixed channels,
 * no physical input handling (no button wired on this project - GPIO1 is
 * dedicated to the state sense, see pc_sense.c), no NVS persistence
 * (pulses have no "true" state to remember across reboots: on cold boot
 * the output is always OFF).
 *
 * Pulse: one-shot FreeRTOS timer (never vTaskDelay in the Zigbee task).
 */

#include "relay.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"

static const char *TAG = "RELAY";

static const gpio_num_t OUT_GPIO[NUM_CH] = {
    RELAY_OUT_GPIO_POWER, RELAY_OUT_GPIO_RESET
};

static bool             s_state[NUM_CH]          = { false, false };
static TimerHandle_t    s_impulse_timers[NUM_CH]  = { NULL, NULL };
static relay_change_cb_t s_cb                     = NULL;

/*
 * Timer callback: physically switches the relay off.
 * For CH_RESET, notifies main.c (-> ZCL attribute OFF).
 * For CH_POWER, notifies nothing: this channel's Zigbee attribute follows
 * pc_sense exclusively, not the pulse.
 */
static void impulse_timer_cb(TimerHandle_t xTimer)
{
    uint8_t ch = (uint8_t)(uint32_t)pvTimerGetTimerID(xTimer);
    s_state[ch] = false;
    gpio_set_level(OUT_GPIO[ch], 0);
    ESP_LOGI(TAG, "CH%d pulse finished -> OFF", ch);
    if (ch == CH_RESET && s_cb) s_cb(ch, false);
}

void relay_init(relay_change_cb_t cb)
{
    s_cb = cb;

    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        gpio_config_t out = {
            .pin_bit_mask = (1ULL << OUT_GPIO[ch]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&out);
        gpio_set_level(OUT_GPIO[ch], 0);   /* always OFF on cold boot */

        s_impulse_timers[ch] = xTimerCreate(
            ch == CH_POWER ? "relay_pwr" : "relay_rst",
            pdMS_TO_TICKS(RELAY_PULSE_MS),
            pdFALSE,                       /* one-shot */
            (void *)(uint32_t)ch,
            impulse_timer_cb
        );
        ESP_LOGI(TAG, "CH%d init: out=GPIO%d pulse=%dms", ch, OUT_GPIO[ch], RELAY_PULSE_MS);
    }
}

void relay_pulse(uint8_t ch)
{
    if (ch >= NUM_CH) return;
    ESP_LOGI(TAG, "CH%d pulse (Zigbee) -> timer", ch);
    s_state[ch] = true;
    gpio_set_level(OUT_GPIO[ch], 1);
    if (s_impulse_timers[ch]) xTimerReset(s_impulse_timers[ch], 0);
    /* For CH_RESET, impulse_timer_cb will call s_cb(ch, false) at the end
     * of the pulse. For CH_POWER, no notification: pc_sense writes the
     * attribute. */
}

bool relay_get(uint8_t ch)
{
    if (ch >= NUM_CH) return false;
    return s_state[ch];
}
