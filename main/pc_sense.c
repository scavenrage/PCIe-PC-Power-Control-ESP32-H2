/*
 * pc_sense.c — Polling/debounce task for the PC's real power state (GPIO1)
 */

#include "pc_sense.h"
#include "config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "PC_SENSE";

static pc_sense_change_cb_t s_cb           = NULL;
static volatile bool        s_reported     = false;  /* last value sent to Zigbee */

/* Raw GPIO reading -> true if the PCIe 3.3V rail is present (PC on).
 * If the real polarity is inverted compared to the assumption documented
 * in pc_sense.h, invert the comparison here (== 1 -> == 0). */
static inline bool pc_sense_read_raw(void)
{
    return gpio_get_level(PC_SENSE_GPIO) == 1;
}

bool pc_sense_get_state(void)
{
    return s_reported;
}

static void pc_sense_task(void *pv)
{
    /* Immediate initial reading, without waiting for debounce, so the
     * first report over Zigbee (at join) is already correct instead of
     * defaulting to "false". */
    bool candidate   = pc_sense_read_raw();
    s_reported       = candidate;
    uint32_t stable_ms = PC_SENSE_DEBOUNCE_MS;   /* already "stable" at boot */
    ESP_LOGI(TAG, "Initial PC state: %s", candidate ? "ON" : "OFF");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(PC_SENSE_POLL_MS));

        bool level = pc_sense_read_raw();
        if (level == candidate) {
            if (stable_ms < UINT32_MAX - PC_SENSE_POLL_MS) stable_ms += PC_SENSE_POLL_MS;
        } else {
            candidate = level;
            stable_ms = 0;
            continue;
        }

        if (stable_ms < PC_SENSE_DEBOUNCE_MS) continue;

        /* Level stable for >= PC_SENSE_DEBOUNCE_MS: if it differs from
         * what was reported over Zigbee, update it (covers both genuine
         * changes and self-correcting an attribute forced by an ON/OFF
         * command). */
        if (candidate != s_reported) {
            s_reported = candidate;
            ESP_LOGI(TAG, "Real PC state -> %s", candidate ? "ON" : "OFF");
            if (s_cb) s_cb(candidate);
        }
    }
}

void pc_sense_init(pc_sense_change_cb_t cb)
{
    s_cb = cb;

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PC_SENSE_GPIO),
        .mode         = GPIO_MODE_INPUT,
        /* Internal pull-down as a safety net: if the external signal were
         * left undriven (e.g. hardware protection with only a series
         * resistor, no reference to GND), the pin still reads a
         * deterministic LOW instead of floating. Doesn't interfere with
         * any protection/divider already present on the schematic. */
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);

    xTaskCreate(pc_sense_task, "pc_sense", 3072, NULL, 4, NULL);
}
