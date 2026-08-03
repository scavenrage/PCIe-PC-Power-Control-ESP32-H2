/*
 * ota.h — Zigbee OTA Upgrade client for ESP32-H2 (pcie_power_control)
 *
 * OTA_IMAGE_TYPE is distinct from smart_switch (0x0104) to prevent a
 * coordinator/ZHA from mistakenly offering an incompatible OTA image to
 * this device or vice versa. Self-assigned value (no official registry
 * for the custom manufacturer code 0x1001).
 */

#pragma once

#include "esp_err.h"
#include "esp_zigbee_core.h"

#define OTA_MANUFACTURER_CODE   0x1001
#define OTA_IMAGE_TYPE          0x0105   /* 0x0105 = pcie_power_control (smart_switch=0x0104) */
#define OTA_HW_VERSION          0x0001
#define OTA_MAX_DATA_SIZE       223

#define OTA_FILE_VERSION        0x01000000   /* v1.0.0 */
#define OTA_SW_BUILD_ID         "\x06""v1.0.0"

esp_zb_attribute_list_t *ota_cluster_create(void);
esp_err_t ota_upgrade_handler(const esp_zb_zcl_ota_upgrade_value_message_t *message);
void ota_mark_valid(void);
