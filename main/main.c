/*
 * main.c — pcie_power_control (ESP32-H2)
 *
 * Firmware DEDICATED to the PCIe-PC-Power-Control-ESP32-H2 project, forked
 * from smart_switch. No runtime configuration: 2 fixed impulse channels
 * (300ms) + real PC power sense on GPIO1.
 *
 * Zigbee endpoints:
 *   EP1 "Power": On/Off cluster + OTA client.
 *     - ON or OFF command -> 300ms pulse on GPIO4 (PWR_SW), in BOTH cases:
 *       just like a real ATX power button, every command (in either
 *       direction) counts as "press the power button" once, which turns
 *       the PC on if it's off or starts shutdown if it's on.
 *     - The On/Off attribute does NOT reflect the pulse: it reflects the
 *       REAL state of the PC read by pc_sense.c (GPIO1, standard PCIe
 *       3.3V). The pulse itself is "silent" on the Zigbee side (relay.c
 *       does not notify it) - it's pc_sense that writes the attribute when
 *       the PC actually changes state.
 *   EP2 "Reset": On/Off cluster.
 *     - ON command -> 300ms pulse on GPIO5 (RESET). OFF command ignored.
 *     - Standard impulse relay behavior: attribute ON during the pulse,
 *       automatically back to OFF at the end of the pulse (no native
 *       ZCL/ZHA way to expose it as a stateless "button" on a server
 *       On/Off endpoint - see README).
 *
 * All the Zigbee resilience logic (coordinator keepalive, application
 * watchdog, hard/phy reset, GPIO9 factory reset, OTA) is unchanged from
 * smart_switch: it's already field-proven and unrelated to the channel
 * configuration, so it hasn't been touched.
 */

#include "config.h"
#include "ota.h"
#include "led.h"
#include "relay.h"
#include "pc_sense.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_ota.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "esp_partition.h"
#include "esp_sleep.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "MAIN";

/* -- Zigbee configuration --------------------------------------------- */
#define INSTALLCODE_POLICY_ENABLE    false
#define MAX_CHILDREN                 10
#define ESP_ZB_PRIMARY_CHANNEL_MASK  (1UL << 25)   /* fixed channel 25 */
#define OTA_UPGRADE_QUERY_INTERVAL   240   /* minutes */

#define ESP_MANUFACTURER_NAME  "\x09""Handmade!"
#define ESP_MODEL_IDENTIFIER   "\x0B""PCIePwrCtrl"

#define ESP_ZB_ZR_CONFIG() {                                        \
    .esp_zb_role         = ESP_ZB_DEVICE_TYPE_ROUTER,               \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,               \
    .nwk_cfg.zczr_cfg    = { .max_children = MAX_CHILDREN },        \
}
#define ESP_ZB_DEFAULT_RADIO_CONFIG() { .radio_mode = ZB_RADIO_MODE_NATIVE }
#define ESP_ZB_DEFAULT_HOST_CONFIG()  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE }

/* -- Endpoint -> channel map (fixed: no NVS) --------------------------- */
static const uint8_t s_ch_to_ep[NUM_CH] = { 1, 2 };  /* CH_POWER->EP1, CH_RESET->EP2 */
static const uint8_t s_ota_ep           = 1;

static bool zigbee_ready             = false;
static bool s_coordinator_confirmed  = false;   /* true only after a successful ZDO ping */
static uint8_t s_ping_tick           = 0;
static uint8_t s_ping_fail_count     = 0;       /* consecutive ZDO failures         */
static bool    s_ping_in_flight      = false;   /* ping sent, waiting for callback  */
static uint8_t s_steering_fail_count = 0;       /* consecutive steering failures    */

/*
 * s_reporting_ready: becomes true 500ms after sending bind + configure
 * reporting for the On/Off attributes (see setup_reporting()). While it's
 * false, zb_report_on_off() only updates the local attribute value
 * (visible via a manual Read Attribute) but does NOT transmit it over the
 * air.
 *
 * WARNING - known bug (already fixed once in ConsoleZ906 for the Analog
 * Output cluster): esp_zb_zcl_set_attribute_val(..., true) alone does NOT
 * guarantee an unsolicited report is sent to the coordinator. The boolean
 * parameter of that function is "check_value_range" (min/max validation),
 * not "send a report now". An unsolicited report is only sent if a
 * binding + a reporting configuration exist for that attribute - for
 * state changes originated locally (here: pc_sense reading GPIO1 on its
 * own, not an incoming Zigbee command) this has to be configured
 * explicitly and the push has to be forced with
 * esp_zb_zcl_report_attr_cmd_req().
 */
static bool s_reporting_ready = false;

/* -- Keepalive: constants and forward declarations --------------------- */
/*
 * esp_zb_scheduler_alarm internally truncates the delay to 16 bits:
 * values > 65535 ms overflow. We use 60000 ms (safe, same value as the
 * watchdog) and count COORDINATOR_PING_TICKS ticks before each actual
 * ping -> effective interval = 60000 x 5 = 5 minutes.
 *
 * If the coordinator doesn't respond for COORDINATOR_PING_MAX_FAILS
 * consecutive cycles (~4-5 min), the device is considered permanently
 * disconnected (typically due to RF interference) -> zigbee_hard_reset():
 * erases zb_storage/zb_fct and reboots for a clean join.
 *
 * COORDINATOR_PING_TIMEOUT_MS: safety timeout for the case where the ZDO
 * callback is never invoked (route error instead of a ZDP timeout).
 */
#define COORDINATOR_PING_TICK_MS       60000    /* safe interval for the scheduler  */
#define COORDINATOR_PING_TICKS         5        /* 5 x 60s = 5 min between pings    */
#define COORDINATOR_PING_MAX_FAILS     6        /* hard reset after 6 fails x ~45s ~= 4-5 min */
#define COORDINATOR_PING_TIMEOUT_MS    40000    /* timeout if the callback never arrives */
#define STEERING_MAX_FAILS             10       /* phy reset after 10 fails x 30s ~= 5 min */
static void coordinator_ping_cb(uint8_t param);         /* defined further below */
static void coordinator_ping_timeout_cb(uint8_t param); /* defined further below */

/* -- OTA ---------------------------------------------------------------- */
static void ota_find_server(void)
{
    esp_zb_ota_upgrade_client_query_interval_set(s_ota_ep, OTA_UPGRADE_QUERY_INTERVAL);
    esp_zb_ota_upgrade_client_query_image_req(0x0000, 1);
    ESP_LOGI(TAG, "OTA: querying coordinator, polling every %d min", OTA_UPGRADE_QUERY_INTERVAL);
}

/* -- Zigbee hard reset (permanent disconnection) ------------------------ */
static void zigbee_hard_reset(const char *reason)
{
    ESP_LOGW(TAG, "*** ZIGBEE HARD RESET (%s) - erasing Zigbee partitions ***", reason);
    led_set_state(LED_ERROR);
    const char *zb_parts[] = { "zb_storage", "zb_fct", "phy_init" };
    for (int i = 0; i < 3; i++) {
        const esp_partition_t *p = esp_partition_find_first(
            ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, zb_parts[i]);
        if (p) {
            esp_err_t err = esp_partition_erase_range(p, 0, p->size);
            ESP_LOGW(TAG, "  '%s' (0x%"PRIx32", %"PRIu32"B): %s",
                     zb_parts[i], p->address, p->size,
                     err == ESP_OK ? "OK" : esp_err_to_name(err));
        } else {
            ESP_LOGE(TAG, "  '%s': partition not found!", zb_parts[i]);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

static void zigbee_phy_reset(const char *reason)
{
    ESP_LOGW(TAG, "*** ZIGBEE PHY RESET (%s) - erasing phy_init ***", reason);
    led_set_state(LED_ERROR);
    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, "phy_init");
    if (p) {
        esp_err_t err = esp_partition_erase_range(p, 0, p->size);
        ESP_LOGW(TAG, "  'phy_init' (0x%"PRIx32", %"PRIu32"B): %s",
                 p->address, p->size,
                 err == ESP_OK ? "OK" : esp_err_to_name(err));
    } else {
        ESP_LOGE(TAG, "  'phy_init': partition not found!");
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

/* -- Hardware -> Zigbee callbacks --------------------------------------- */
/*
 * IMPORTANT: as in smart_switch, all communication with the Zigbee stack
 * from tasks other than the Zigbee task go ONLY through
 * esp_zb_scheduler_alarm (never esp_zb_lock_acquire), to avoid race
 * conditions on the stack's internal QueueSet.
 */

/*
 * zb_report_on_off() - updates the local On/Off attribute AND forces an
 * unsolicited Report Attribute to the coordinator (0x0000), instead of
 * relying only on the "check" parameter of esp_zb_zcl_set_attribute_val
 * (which does NOT transmit anything over the air). Must be called from
 * the Zigbee task (via esp_zb_scheduler_alarm), never directly from other
 * tasks.
 */
static void zb_report_on_off(uint8_t ep, bool value)
{
    if (!zigbee_ready) {
        ESP_LOGW(TAG, "zb_report_on_off: EP%d val=%d DROPPED (zigbee_ready=false)", ep, value);
        return;
    }

    uint8_t val = value ? 1 : 0;
    esp_zb_zcl_status_t st = esp_zb_zcl_set_attribute_val(ep,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
        &val, false);
    ESP_LOGI(TAG, "zb_report_on_off: EP%d -> %s (set_attribute_val status=0x%02x, reporting_ready=%d)",
             ep, value ? "ON" : "OFF", st, s_reporting_ready);

    if (!s_reporting_ready) return;   /* bind/config report not active yet */

    esp_zb_zcl_report_attr_cmd_t rep = { 0 };
    rep.address_mode                        = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    rep.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;   /* coordinator */
    rep.zcl_basic_cmd.dst_endpoint          = 1;
    rep.zcl_basic_cmd.src_endpoint          = ep;
    rep.clusterID   = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
    rep.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
    rep.direction   = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    esp_err_t rep_err = esp_zb_zcl_report_attr_cmd_req(&rep);
    ESP_LOGI(TAG, "zb_report_on_off: EP%d report_attr_cmd_req -> %s", ep, esp_err_to_name(rep_err));
}

/* -- Reset (CH_RESET) -> Zigbee: end of pulse -> attribute OFF --------- */
static void do_reset_zigbee_sync(uint8_t param)
{
    (void)param;
    zb_report_on_off(s_ch_to_ep[CH_RESET], false);
}

static void on_reset_changed(uint8_t ch, bool state)
{
    (void)ch; (void)state;   /* called only for CH_RESET, only at the end of the pulse (state=false) */
    if (!zigbee_ready) return;
    esp_zb_scheduler_alarm(do_reset_zigbee_sync, 0, 0);
}

/* -- PC sense (CH_POWER) -> Zigbee: real state of the PC --------------- */
static volatile bool s_pc_on_pending;

static void do_pc_sense_zigbee_sync(uint8_t param)
{
    (void)param;
    zb_report_on_off(s_ch_to_ep[CH_POWER], s_pc_on_pending);
}

static void on_pc_state_changed(bool pc_on)
{
    if (!zigbee_ready) return;
    s_pc_on_pending = pc_on;
    esp_zb_scheduler_alarm(do_pc_sense_zigbee_sync, 0, 0);
}

/*
 * Bind + Configure Reporting for the On/Off attribute (Power/Reset).
 * Needed so that locally originated state changes (pc_sense on GPIO1,
 * Reset pulse end) actually get transmitted to the coordinator instead of
 * staying only in the local attribute table. Called once, on the first
 * confirmed coordinator reachability after a (re)join (see
 * coordinator_ping_result).
 */
static void reporting_ready_cb(uint8_t param)
{
    (void)param;
    s_reporting_ready = true;
    ESP_LOGI(TAG, "Zigbee On/Off reporting+binding ready");

    /* First REAL push after boot/join: while s_reporting_ready was false,
     * zb_report_on_off() only updated the local attribute without
     * transmitting it (see the comment on s_reporting_ready). At this
     * point bind+config report are finally active, so we push the
     * current state right away - pc_sense already read GPIO1 at boot
     * (before Zigbee even connected), so there's no need to wait for a PC
     * state change or a user command to see the correct value in HA: if
     * the PC is off, OFF is sent; if it's on, ON is sent. */
    zb_report_on_off(s_ch_to_ep[CH_POWER], pc_sense_get_state());
    zb_report_on_off(s_ch_to_ep[CH_RESET], relay_get(CH_RESET));
}

static void setup_reporting(const esp_zb_ieee_addr_t coord_ieee)
{
    uint16_t own_addr = esp_zb_get_short_address();

    /* Configure Reporting, addressed to ourselves: registers the On/Off
     * attribute in the local reporting table as "to be sent" (direction
     * SEND). BOOL type = discrete: no reportable_change threshold is
     * defined by the ZCL spec, only min/max interval. */
    esp_zb_zcl_config_report_record_t rec = { 0 };
    rec.direction    = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
    rec.attributeID  = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
    rec.attrType     = ESP_ZB_ZCL_ATTR_TYPE_BOOL;
    rec.min_interval = 0;
    rec.max_interval = 300;
    rec.reportable_change = NULL;

    esp_zb_zcl_config_report_cmd_t cfg = { 0 };
    cfg.address_mode  = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cfg.clusterID     = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
    cfg.record_number = 1;
    cfg.record_field  = &rec;
    cfg.zcl_basic_cmd.dst_addr_u.addr_short = own_addr;

    cfg.zcl_basic_cmd.src_endpoint = s_ch_to_ep[CH_POWER];
    cfg.zcl_basic_cmd.dst_endpoint = s_ch_to_ep[CH_POWER];
    esp_zb_zcl_config_report_cmd_req(&cfg);

    cfg.zcl_basic_cmd.src_endpoint = s_ch_to_ep[CH_RESET];
    cfg.zcl_basic_cmd.dst_endpoint = s_ch_to_ep[CH_RESET];
    esp_zb_zcl_config_report_cmd_req(&cfg);

    /* Binding towards the coordinator: tells the stack WHERE to send
     * reports once the reporting condition is met. */
    esp_zb_zdo_bind_req_param_t bind = { 0 };
    bind.cluster_id    = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
    bind.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    bind.dst_endp      = 1;
    bind.req_dst_addr  = own_addr;
    esp_zb_ieee_address_by_short(own_addr, bind.src_address);
    memcpy(bind.dst_address_u.addr_long, coord_ieee, sizeof(esp_zb_ieee_addr_t));

    bind.src_endp = s_ch_to_ep[CH_POWER];
    esp_zb_zdo_device_bind_req(&bind, NULL, NULL);

    bind.src_endp = s_ch_to_ep[CH_RESET];
    esp_zb_zdo_device_bind_req(&bind, NULL, NULL);

    esp_zb_scheduler_alarm(reporting_ready_cb, 0, 500);
    ESP_LOGI(TAG, "Zigbee On/Off reporting+binding sent - active in 500ms");
}

/* -- Basic cluster construction ------------------------------------------ */
static esp_zb_attribute_list_t *create_basic_cluster(void)
{
    esp_zb_basic_cluster_cfg_t cfg = {
        .zcl_version  = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,
    };
    esp_zb_attribute_list_t *c = esp_zb_basic_cluster_create(&cfg);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  ESP_MODEL_IDENTIFIER);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID,          OTA_SW_BUILD_ID);
    return c;
}

/* -- On/Off endpoint construction (Power / Reset) ------------------------ */
static void add_relay_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep, uint8_t ch, const char *name)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t id_cfg = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cl,
        esp_zb_identify_cluster_create(&id_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_on_off_cluster_cfg_t oo = { .on_off = 0 };
    esp_zb_cluster_list_add_on_off_cluster(cl,
        esp_zb_on_off_cluster_create(&oo), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* CLIENT role needed to be able to call esp_zb_zcl_report_attr_cmd_req()
     * with direction=TO_CLI on this cluster without hitting a stack
     * assert (esp-idf#15962) - same workaround already used in
     * ConsoleZ906 for the Analog Output cluster. */
    esp_zb_cluster_list_add_on_off_cluster(cl,
        esp_zb_on_off_cluster_create(&oo), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    if (ep == s_ota_ep)
        esp_zb_cluster_list_add_ota_cluster(cl,
            ota_cluster_create(), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_endpoint_config_t epcfg = {
        .endpoint           = ep,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
    ESP_LOGI(TAG, "EP%d: On/Off %s (CH%d)", ep, name, ch);
}

/* -- Endpoint creation (fixed: Power=EP1, Reset=EP2) --------------------- */
static void create_endpoints(esp_zb_ep_list_t *ep_list)
{
    add_relay_endpoint(ep_list, s_ch_to_ep[CH_POWER], CH_POWER, "Power");
    add_relay_endpoint(ep_list, s_ch_to_ep[CH_RESET], CH_RESET, "Reset");
}

/* -- Zigbee command handler ----------------------------------------------- */

static esp_err_t handle_on_off(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    uint8_t ep = msg->info.dst_endpoint;
    uint8_t ch;
    if (ep == s_ch_to_ep[CH_POWER])       ch = CH_POWER;
    else if (ep == s_ch_to_ep[CH_RESET])  ch = CH_RESET;
    else return ESP_OK;

    if (msg->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) return ESP_OK;
    if (msg->attribute.id != ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) return ESP_OK;
    if (msg->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_BOOL) return ESP_OK;

    bool state = *(bool *)msg->attribute.data.value;
    ESP_LOGI(TAG, "Zigbee On/Off -> EP%d CH%d command=%s", ep, ch, state ? "ON" : "OFF");

    if (ch == CH_POWER) {
        /* "Power" mirrors the REAL state of the PC (pc_sense), not a
         * classic impulse relay: every command received - both ON and
         * OFF - counts as "press the power button" once, exactly like a
         * real ATX button (turns the PC on if it's off, turns it off /
         * starts shutdown if it's on). That's why we react to BOTH
         * commands with a single pulse, not just ON.
         *
         * BEFORE this fix: the stack writes the attribute to the command
         * value (ON or OFF) anyway before calling this handler -
         * automatic On/Off cluster behavior, unavoidable. If the PC
         * didn't really change state, the attribute stayed stuck on that
         * WRONG value: pc_sense compares the GPIO1 level against its own
         * last known reading, not against the Zigbee attribute, so if the
         * PC doesn't really change state pc_sense never notices any
         * discrepancy to correct and the wrong value stays forever.
         *
         * Fix (v2): we re-assert the true state read from pc_sense - BUT
         * not by calling esp_zb_zcl_set_attribute_val() synchronously
         * right here. This handler is invoked by ZBOSS while it's still
         * finishing processing the incoming On/Off command (we're inside
         * ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID): writing the same attribute
         * again at this point risks ZBOSS silently overwriting our
         * correction with the original command value once it finishes
         * handling the command right after this handler returns - which
         * is exactly the observed symptom (no visible change at all, not
         * even transiently).
         * Instead we use esp_zb_scheduler_alarm (via on_pc_state_changed,
         * the same mechanism already verified for real GPIO1 changes),
         * which queues the correction to run AFTER ZBOSS has finished
         * handling the current command. The attribute must only change
         * because of a genuine report from pc_sense, never as a direct
         * effect of the received command. */
        relay_pulse(CH_POWER);
        on_pc_state_changed(pc_sense_get_state());
    } else {
        /* CH_RESET: pure impulse relay, unchanged from smart_switch -
         * reacts only to the ON command. The attribute goes back to OFF
         * by itself at the end of the pulse (relay.c -> on_reset_changed),
         * so an explicit OFF command from Zigbee shouldn't trigger
         * anything. */
        if (state) relay_pulse(CH_RESET);
    }
    return ESP_OK;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    ESP_RETURN_ON_FALSE(msg, ESP_FAIL, TAG, "empty msg");
    ESP_RETURN_ON_FALSE(msg->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
        ESP_ERR_INVALID_ARG, TAG, "status error");

    if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        return handle_on_off(msg);
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t cb_id, const void *msg)
{
    switch (cb_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)msg);
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID: {
        const esp_zb_zcl_ota_upgrade_value_message_t *ota =
            (esp_zb_zcl_ota_upgrade_value_message_t *)msg;
        if (ota->upgrade_status == ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START)
            led_set_state(LED_OTA_IN_PROGRESS);
        return ota_upgrade_handler(ota);
    }
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID: {
        const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *r = msg;
        if (r->query_status == ESP_ZB_ZCL_STATUS_SUCCESS)
            ESP_LOGI(TAG, "OTA: new image v0x%08"PRIx32" (%"PRIu32" bytes)",
                     r->file_version, r->image_size);
        else
            ESP_LOGI(TAG, "OTA: no new image (0x%02x)", r->query_status);
        return ESP_OK;
    }
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        return ESP_OK;
    default:
        ESP_LOGW(TAG, "Unhandled action: 0x%x", cb_id);
        return ESP_OK;
    }
}

/* -- Zigbee signals -------------------------------------------------------- */
static void bdb_start_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(
        esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,
        , TAG, "BDB commissioning error");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *sig)
{
    uint32_t *p          = sig->p_app_signal;
    esp_err_t err        = sig->esp_err_status;
    esp_zb_app_signal_type_t type = *p;

    switch (type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Starting steering...");
            led_set_state(LED_ZIGBEE_SEARCHING);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Init failed, retrying...");
            led_set_state(LED_ERROR);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        zigbee_ready            = false;
        s_coordinator_confirmed = false;
        s_reporting_ready       = false;
        ESP_LOGW(TAG, "LEAVE signal received - retrying steering in 30s...");
        led_set_state(LED_ZIGBEE_SEARCHING);
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            /* Sanity check: if the address is 0xFFFF the radio is in a
             * corrupted state (ZBOSS bug #727 - steering reports success
             * but the radio isn't actually associated). Unrecoverable
             * state -> hard reset. */
            if (esp_zb_get_short_address() == 0xFFFF) {
                zigbee_hard_reset("addr=0xFFFF");
            }
            s_steering_fail_count = 0;
            zigbee_ready = true;
            ESP_LOGI(TAG, "Network found: channel %d, PAN 0x%04hx, addr 0x%04hx - checking coordinator...",
                     esp_zb_get_current_channel(),
                     esp_zb_get_pan_id(),
                     esp_zb_get_short_address());
            s_ping_tick = COORDINATOR_PING_TICKS - 1;   /* next tick -> ping right away */
            esp_zb_scheduler_alarm(coordinator_ping_cb, 0, 1000);
        } else {
            s_steering_fail_count++;
            ESP_LOGW(TAG, "Steering failed (%d/%d), retrying in 30s...",
                     s_steering_fail_count, STEERING_MAX_FAILS);
            if (s_steering_fail_count >= STEERING_MAX_FAILS) {
                zigbee_phy_reset("steering loop");
            }
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
        }
        break;

    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        if (zigbee_ready) {   /* guard: avoid scheduling multiple times */
            zigbee_ready            = false;
            s_coordinator_confirmed = false;
            s_reporting_ready       = false;
            ESP_LOGW(TAG, "No active links (0x18) - forcing reconnection");
            led_set_state(LED_ZIGBEE_SEARCHING);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_NLME_STATUS_INDICATION:
        ESP_LOGD(TAG, "NLME Status Indication (0x32) - ignored");
        break;

    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        ESP_LOGD(TAG, "ZDO Device Unavailable (0x3c) - ignored");
        break;

    default:
        ESP_LOGI(TAG, "Signal: %s (0x%x) %s",
                 esp_zb_zdo_signal_to_string(type), type, esp_err_to_name(err));
        break;
    }
}

/* -- ZDO keepalive towards the coordinator (island detection) ---------- */
static void coordinator_ping_result(esp_zb_zdp_status_t zdo_status,
                                    esp_zb_zdo_ieee_addr_rsp_t *resp,
                                    void *user_ctx)
{
    s_ping_in_flight = false;   /* callback received (or forced timeout) */
    if (!zigbee_ready) return;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        s_ping_tick       = 0;
        s_ping_fail_count = 0;
        ESP_LOGD(TAG, "Coordinator keepalive: OK");

        if (!s_coordinator_confirmed) {
            s_coordinator_confirmed = true;
            ESP_LOGI(TAG, "Coordinator reachable - operational");
            led_set_state(LED_OPERATIONAL);
            ota_find_server();
            /* Bind + configure reporting for On/Off: only needs to be
             * sent once per join, using the coordinator's IEEE address
             * just received in this same ZDO response. */
            if (resp) setup_reporting(resp->ieee_addr);
        }

        /* Periodic resync (every 5 minutes): updates ZHA and stimulates
         * route discovery. CH_POWER reports the REAL state of the PC
         * (pc_sense), not the pulse. CH_RESET reports the transient
         * pulse state (relay.c). zb_report_on_off also forces the
         * over-the-air push, not just the local attribute update. */
        zb_report_on_off(s_ch_to_ep[CH_POWER], pc_sense_get_state());
        zb_report_on_off(s_ch_to_ep[CH_RESET], relay_get(CH_RESET));

        /* Next keepalive in 5 minutes */
        esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);
    } else {
        s_ping_fail_count++;
        ESP_LOGW(TAG, "Coordinator unreachable (0x%02x) - fail %d/%d",
                 zdo_status, s_ping_fail_count, COORDINATOR_PING_MAX_FAILS);
        zigbee_ready            = false;
        s_coordinator_confirmed = false;
        s_reporting_ready       = false;
        s_ping_tick             = 0;
        led_set_state(LED_ZIGBEE_SEARCHING);

        if (s_ping_fail_count >= COORDINATOR_PING_MAX_FAILS) {
            zigbee_hard_reset("ping fail");
        }

        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
    }
}

/* Safety timeout: if the ZDO callback doesn't arrive within
 * COORDINATOR_PING_TIMEOUT_MS (e.g. route error instead of a ZDP
 * timeout), we force the failure ourselves. Without this, the device
 * would stay stuck with s_coordinator_confirmed=false with no other
 * alarm scheduled - permanent double-blink. */
static void coordinator_ping_timeout_cb(uint8_t param)
{
    (void)param;
    if (!s_ping_in_flight) return;   /* callback already received normally */
    ESP_LOGW(TAG, "Keepalive: no ZDO response in %ds - forcing failure",
             COORDINATOR_PING_TIMEOUT_MS / 1000);
    coordinator_ping_result(ESP_ZB_ZDP_STATUS_TIMEOUT, NULL, NULL);
}

static void coordinator_ping_cb(uint8_t param)
{
    (void)param;
    if (!zigbee_ready) return;

    s_ping_tick++;
    if (s_ping_tick < COORDINATOR_PING_TICKS) {
        /* Not time for the ping yet: wait for the next tick */
        esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);
        return;
    }
    s_ping_tick = 0;

    esp_zb_zdo_ieee_addr_req_param_t req = {
        .dst_nwk_addr     = 0x0000,
        .addr_of_interest = 0x0000,
        .request_type     = 0,
        .start_index      = 0,
    };
    ESP_LOGD(TAG, "Keepalive: ZDO ieee_addr_req -> coordinator");
    s_ping_in_flight = true;
    esp_zb_zdo_ieee_addr_req(&req, coordinator_ping_result, NULL);
    /* Safety timeout: if it's a route error (not a ZDP timeout), the
     * callback never gets called. The alarm simulates it after 40s. */
    esp_zb_scheduler_alarm(coordinator_ping_timeout_cb, 0, COORDINATOR_PING_TIMEOUT_MS);
}

/* -- Zigbee application watchdog ----------------------------------------- */
/*
 * The zb_wdt_feed_cb callback is scheduled every 60s by the Zigbee
 * scheduler. If the stack silently hangs, the callback stops being
 * called. The zb_watchdog_task checks every 30s that the feed arrived; if
 * it's missing for more than 3 minutes it forces esp_restart().
 *
 * Everything starts inside esp_zb_task() after esp_zb_start(), so only
 * once the stack is already running.
 */
#define ZB_WDT_FEED_INTERVAL_MS   60000    /* feed every 60s               */
#define ZB_WDT_TIMEOUT_MS         180000   /* reset if stuck > 3 min       */
#define ZB_WDT_CHECK_INTERVAL_MS  30000    /* check every 30s              */

static volatile TickType_t s_zb_last_feed = 0;

static void zb_wdt_feed_cb(uint8_t param)
{
    (void)param;
    s_zb_last_feed = xTaskGetTickCount();
    esp_zb_scheduler_alarm(zb_wdt_feed_cb, 0, ZB_WDT_FEED_INTERVAL_MS);
}

static void zb_watchdog_task(void *pv)
{
    /* Wait for the stack to start and the first feed to be registered */
    vTaskDelay(pdMS_TO_TICKS(ZB_WDT_FEED_INTERVAL_MS + 30000));

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ZB_WDT_CHECK_INTERVAL_MS));
        TickType_t elapsed = xTaskGetTickCount() - s_zb_last_feed;
        if (elapsed > pdMS_TO_TICKS(ZB_WDT_TIMEOUT_MS)) {
            ESP_LOGE("ZB_WDT", "Zigbee stack not responding for %lu s - deep sleep 3s",
                     (unsigned long)(elapsed * portTICK_PERIOD_MS / 1000));
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_sleep_enable_timer_wakeup(3ULL * 1000 * 1000);
            esp_deep_sleep_start();
        }
    }
}

/* -- Zigbee factory reset (BOOT button / GPIO9) -------------------------- */
/*
 * A dedicated task continuously monitors GPIO9. If the button is held
 * down for FACTORY_RESET_HOLD_S seconds (at any time, not just at boot),
 * the firmware performs a Zigbee factory reset and reboots the device.
 * The NVS configuration is left untouched.
 *
 * LED feedback: while the button is held, the LED switches to
 * LED_FACTORY_RESET (solid on). If released before the time is up, it
 * goes back to its previous state.
 */
#define FACTORY_RESET_GPIO       GPIO_NUM_9
#define FACTORY_RESET_HOLD_S     5

static void factory_reset_task(void *pv)
{
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << FACTORY_RESET_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);

    /* Ignore the first 10s: during boot the USB-serial chip can hold
       DTR/RTS low -> GPIO9 would appear falsely pressed */
    vTaskDelay(pdMS_TO_TICKS(10000));

    for (;;) {
        /* Wait for GPIO9 to be pressed (active low) */
        if (gpio_get_level(FACTORY_RESET_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* Pressed - start counting */
        ESP_LOGW(TAG, "GPIO9 pressed - hold for %ds to factory reset Zigbee...",
                 FACTORY_RESET_HOLD_S);
        led_set_state(LED_FACTORY_RESET);

        bool held = true;
        for (int i = 0; i < FACTORY_RESET_HOLD_S * 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (gpio_get_level(FACTORY_RESET_GPIO) != 0) {
                held = false;
                break;
            }
        }

        if (!held) {
            ESP_LOGI(TAG, "GPIO9 released - cancelled");
            led_set_state(zigbee_ready ? LED_OPERATIONAL : LED_ZIGBEE_SEARCHING);
            continue;
        }

        /* Held long enough - erase Zigbee partitions and reboot.
         * We erase the zb_storage and zb_fct flash partitions directly,
         * without going through the Zigbee stack (which would require
         * its own task). */
        ESP_LOGW(TAG, "*** ZIGBEE FACTORY RESET - erasing partitions... ***");
        const char *zb_parts[] = { "zb_storage", "zb_fct" };
        for (int i = 0; i < 2; i++) {
            const esp_partition_t *p = esp_partition_find_first(
                ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, zb_parts[i]);
            if (p) {
                esp_err_t err = esp_partition_erase_range(p, 0, p->size);
                ESP_LOGW(TAG, "Partition '%s' (0x%"PRIx32", %"PRIu32" bytes): %s",
                         zb_parts[i], p->address, p->size,
                         err == ESP_OK ? "ERASED" : esp_err_to_name(err));
            } else {
                ESP_LOGE(TAG, "Partition '%s' NOT FOUND!", zb_parts[i]);
            }
        }
        ESP_LOGW(TAG, "Rebooting...");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
}

/* -- Zigbee task ------------------------------------------------------------ */
static void esp_zb_task(void *pv)
{
    esp_zb_cfg_t cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    create_endpoints(ep_list);

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Start the periodic feed of the application watchdog */
    s_zb_last_feed = xTaskGetTickCount();
    esp_zb_scheduler_alarm(zb_wdt_feed_cb, 0, ZB_WDT_FEED_INTERVAL_MS);

    /* Start the ZDO keepalive towards the coordinator (island detection) */
    esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);

    esp_zb_stack_main_loop();
}

/* -- Entry point ------------------------------------------------------------ */
void app_main(void)
{
    /* -- Reset reason check ----------------------------------------------
     * Only POWERON, DEEPSLEEP and EXT guarantee the IEEE 802.15.4 radio is
     * in a clean state. Any other reset (SW_CPU from the brownout
     * handler, hardware BROWNOUT, PANIC, WDT) can leave the radio in a
     * degraded state that prevents rejoining even if the stack reports
     * success. In that case we force a 3s deep sleep: the wakeup is
     * equivalent to a power cycle (rst:0x5 DEEPSLEEP) and guarantees a
     * full radio re-initialization before attempting any Zigbee
     * connection. */
    {
        esp_reset_reason_t rr = esp_reset_reason();
        if (rr != ESP_RST_POWERON &&
            rr != ESP_RST_DEEPSLEEP &&
            rr != ESP_RST_EXT) {
            ESP_LOGW(TAG, "Unclean reset (reason=%d) - deep sleep 3s to reset the radio", (int)rr);
            esp_sleep_enable_timer_wakeup(3ULL * 1000 * 1000);
            esp_deep_sleep_start();
        }
    }

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* OTA anti-rollback */
    ota_mark_valid();

    /* Status LED */
    led_init();

    /* GPIO9 (BOOT) monitor task - Zigbee factory reset if held for 5s */
    xTaskCreate(factory_reset_task, "factory_rst", 2048, NULL, 1, NULL);

    /* Application watchdog task - reboots if the Zigbee stack hangs */
    xTaskCreate(zb_watchdog_task, "zb_wdt", 2048, NULL, 2, NULL);

    /* Power/Reset outputs (300ms pulse) - no ISR, no physical input */
    relay_init(on_reset_changed);

    /* Real PC state sense on GPIO1 (polling + 2s debounce) */
    pc_sense_init(on_pc_state_changed);

    /* Start the Zigbee stack */
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
