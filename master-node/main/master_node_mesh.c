/**
 * @file master_node_mesh.c
 * @brief BLE Mesh client model, STATUS reception and connectivity tracking.
 */

#include "master_node_mesh.h"
#include "master_node_config.h"
#include "master_node_hal.h"
#include "mesh_config.h"

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "ble_mesh_example_init.h"

static const char *TAG = "MASTER_MESH";

/* -------------------------------------------------------------------------- */
/*  Per-node context table                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Saved mesh context for each alarm node.
 *
 * Populated when a STATUS message is received from that node.
 * Used to send unicast replies back to the node if required.
 */
typedef struct {
    bool     used;          /**< true once a STATUS from this node was received */
    uint16_t addr;          /**< unicast address of the alarm node */
    esp_ble_mesh_msg_ctx_t ctx; /**< full context for sending replies */
} node_entry_t;

static node_entry_t s_node_table[MAX_ALARM_NODES];

/* -------------------------------------------------------------------------- */
/*  Per-node green LED state machine                                           */
/* -------------------------------------------------------------------------- */

typedef enum {
    GREEN_MODE_OFF = 0,  /**< Master not provisioned – LED off              */
    GREEN_MODE_SOLID,    /**< Provisioned, no/stale data – LED solid ON     */
    GREEN_MODE_BLINK,    /**< Receiving STATUS – LED toggling               */
} green_mode_t;

static green_mode_t s_green_mode[MAX_ALARM_NODES];
static bool         s_green_level[MAX_ALARM_NODES];   /* current output level */
static uint64_t     s_last_seen_us[MAX_ALARM_NODES];  /* µs timestamp per node */

/* -------------------------------------------------------------------------- */
/*  Module state                                                               */
/* -------------------------------------------------------------------------- */

static bool         s_provisioned = false;
static int64_t      s_reset_press_start_us = 0;

/* -------------------------------------------------------------------------- */
/*  Mesh stack objects                                                         */
/* -------------------------------------------------------------------------- */

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};

/*
 * Config Server – mandatory even on the master.
 * Relay DISABLED: the master node is in the security room / admin room,
 * a fixed location.  Enabling relay wastes radio time and can cause
 * interference with nodes it doesn't need to relay for.
 */
static esp_ble_mesh_cfg_srv_t s_config_server = {
    .relay        = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon       = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy   = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy   = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl      = MESH_DEFAULT_TTL,
    .net_transmit     = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

/*
 * Publication context – master may publish (e.g. group broadcast) but
 * the primary role is subscription/reception.
 */
ESP_BLE_MESH_MODEL_PUB_DEFINE(s_vnd_pub, 8, ROLE_NODE);

/*
 * Vendor CLIENT model operation table.
 *
 * The master registers receive handlers for:
 *   VND_OP_STATUS            – periodic alarm state from every alarm node
 *   VND_OP_MASTER_PKT_STATUS – echo reply from alarm node to unicast ping
 *
 * The master does NOT register VND_OP_SEND / VND_OP_MASTER_PKT here because
 * those are opcodes it *sends*, not receives.
 */
static esp_ble_mesh_model_op_t s_vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(VND_OP_STATUS,            2),
    ESP_BLE_MESH_MODEL_OP(VND_OP_MASTER_PKT_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t s_root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&s_config_server),
};

static esp_ble_mesh_model_t s_vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, VND_MODEL_ID_CLIENT,
                              s_vnd_op, &s_vnd_pub, NULL),
};

static esp_ble_mesh_elem_t s_elements[] = {
    ESP_BLE_MESH_ELEMENT(0, s_root_models, s_vnd_models),
};

static esp_ble_mesh_comp_t s_composition = {
    .cid           = CID_ESP,
    .elements      = s_elements,
    .element_count = ARRAY_SIZE(s_elements),
};

static esp_ble_mesh_prov_t s_provision = {
    .uuid = dev_uuid,
};

/* -------------------------------------------------------------------------- */
/*  Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Save the source context of an incoming STATUS message.
 *
 * Enables the master to send unicast messages back to any alarm node
 * after it has been heard at least once.
 */
static void save_node_ctx(uint8_t node_index,
                          const esp_ble_mesh_msg_ctx_t *src_ctx)
{
    if (node_index >= MAX_ALARM_NODES) {
        ESP_LOGW(TAG, "save_node_ctx: index %u out of range", node_index);
        return;
    }

    node_entry_t *e = &s_node_table[node_index];
    memcpy(&e->ctx, src_ctx, sizeof(esp_ble_mesh_msg_ctx_t));
    e->addr       = src_ctx->addr;
    e->ctx.send_ttl = MESH_DEFAULT_TTL;
    e->used       = true;

    ESP_LOGD(TAG, "Saved ctx for node %u (unicast=0x%04x net=0x%04x app=0x%04x)",
             node_index, e->addr, e->ctx.net_idx, e->ctx.app_idx);
}

/* -------------------------------------------------------------------------- */
/*  Provisioning callback                                                      */
/* -------------------------------------------------------------------------- */

static void on_prov_complete(uint16_t net_idx, uint16_t addr,
                             uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "Provisioned: net_idx=0x%04x unicast_addr=0x%04x",
             net_idx, addr);
    s_provisioned = true;

    /*
     * After provisioning, assume all nodes are "no data yet" → SOLID ON.
     * This gives a clear visual: all green LEDs ON means the master is alive
     * but no alarm nodes have been heard yet (or since last boot).
     */
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        s_green_mode[i]    = GREEN_MODE_SOLID;
        s_green_level[i]   = 1;
        s_last_seen_us[i]  = 0;
        master_hal_set_green_led(i, 1);
    }
}

static void prov_cb(esp_ble_mesh_prov_cb_event_t event,
                    esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {

    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "PROV_REGISTER_COMP err=%d",
                 param->prov_register_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "NODE_PROV_ENABLE_COMP err=%d",
                 param->node_prov_enable_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "NODE_PROV_LINK_OPEN bearer=%s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV
                 ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "NODE_PROV_LINK_CLOSE bearer=%s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV
                 ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "NODE_PROV_COMPLETE");
        on_prov_complete(param->node_prov_complete.net_idx,
                         param->node_prov_complete.addr,
                         param->node_prov_complete.flags,
                         param->node_prov_complete.iv_index);
        break;

    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "NODE_PROV_RESET");
        s_provisioned = false;
        for (int i = 0; i < MAX_ALARM_NODES; i++) {
            s_green_mode[i]   = GREEN_MODE_OFF;
            s_green_level[i]  = 0;
            s_last_seen_us[i] = 0;
            s_node_table[i].used = false;
            master_hal_set_green_led(i, 0);
            master_hal_set_red_led(i, 0);
        }
        esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV |
                                      ESP_BLE_MESH_PROV_GATT);
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/*  Configuration Server callback                                              */
/* -------------------------------------------------------------------------- */

static void cfg_srv_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                       esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) return;

    switch (param->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        ESP_LOGI(TAG, "CFG: AppKey added net=0x%04x app=0x%04x",
                 param->value.state_change.appkey_add.net_idx,
                 param->value.state_change.appkey_add.app_idx);
        ESP_LOG_BUFFER_HEX("AppKey",
                           param->value.state_change.appkey_add.app_key, 16);
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        ESP_LOGI(TAG, "CFG: AppKey bound elem=0x%04x app=0x%04x "
                      "cid=0x%04x mid=0x%04x",
                 param->value.state_change.mod_app_bind.element_addr,
                 param->value.state_change.mod_app_bind.app_idx,
                 param->value.state_change.mod_app_bind.company_id,
                 param->value.state_change.mod_app_bind.model_id);
        break;
    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/*  Vendor model callback – core of the master logic                           */
/* -------------------------------------------------------------------------- */

static void custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {

    case ESP_BLE_MESH_MODEL_OPERATION_EVT:

        if (param->model_operation.opcode == VND_OP_STATUS) {
            /*
             * ── Alarm node STATUS received ─────────────────────────────
             *
             * Payload: alarm_status_msg_t (2 bytes)
             *   [0] node_index   → which alarm node sent this
             *   [1] alarm_state  → 0 = OK, 1 = FIRE ALARM
             *
             * Actions:
             *   1. Validate length and index.
             *   2. Save sender context (enables future unicast to this node).
             *   3. Update red LED immediately (alarm state).
             *   4. Record heartbeat timestamp → switches green to BLINK mode.
             */
            if (param->model_operation.length < sizeof(alarm_status_msg_t)) {
                ESP_LOGW(TAG, "STATUS too short (%u bytes)",
                         param->model_operation.length);
                break;
            }

            uint8_t node_index  = param->model_operation.msg[0];
            uint8_t alarm_state = param->model_operation.msg[1];

            if (node_index >= MAX_ALARM_NODES) {
                ESP_LOGW(TAG, "STATUS from unknown node_index %u – ignored",
                         node_index);
                break;
            }

            ESP_LOGI(TAG, "STATUS node=%u state=%u src=0x%04x",
                     node_index, alarm_state,
                     param->model_operation.ctx->addr);

            save_node_ctx(node_index, param->model_operation.ctx);

            /* Red LED: live alarm state */
            master_hal_set_red_led(node_index, alarm_state);

            /* Green LED: mark as BLINK (node is alive) */
            s_last_seen_us[node_index] = (uint64_t)esp_timer_get_time();
            if (s_green_mode[node_index] != GREEN_MODE_BLINK) {
                ESP_LOGI(TAG, "Node %u came online → green BLINK", node_index);
            }
            s_green_mode[node_index] = GREEN_MODE_BLINK;
        }

        else if (param->model_operation.opcode == VND_OP_MASTER_PKT_STATUS) {
            /* Echo reply from an alarm node to a unicast ping – log only */
            uint16_t val = *(uint16_t *)param->model_operation.msg;
            ESP_LOGI(TAG, "MASTER_PKT_STATUS from 0x%04x val=%u",
                     param->model_operation.ctx->addr, val);
        }

        break;

    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE(TAG, "Send failed opcode=0x%06" PRIx32 " err=0x%02" PRIx32,
                     (uint32_t)param->model_send_comp.opcode,
                     (uint32_t)param->model_send_comp.err_code);
        } else {
            ESP_LOGD(TAG, "Send OK opcode=0x%06" PRIx32,
                     (uint32_t)param->model_send_comp.opcode);
        }
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/*  Public API – heartbeat tick (called from application timer)               */
/* -------------------------------------------------------------------------- */

void master_mesh_heartbeat_tick(void)
{
    if (!s_provisioned) {
        return;
    }

    uint64_t now = (uint64_t)esp_timer_get_time();

    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        switch (s_green_mode[i]) {

        case GREEN_MODE_BLINK:
            if (s_last_seen_us[i] > 0 &&
                (now - s_last_seen_us[i]) > NODE_TIMEOUT_US) {
                /*
                 * No STATUS for NODE_TIMEOUT_US → assume node offline.
                 * Switch to SOLID ON (fail-safe: last red state preserved).
                 */
                ESP_LOGW(TAG, "Node %d timed out → green SOLID (last alarm state kept)", i);
                s_green_mode[i]  = GREEN_MODE_SOLID;
                s_green_level[i] = 1;
                master_hal_set_green_led(i, 1);
            } else {
                /* Toggle to show ongoing heartbeat */
                s_green_level[i] = !s_green_level[i];
                master_hal_set_green_led(i, s_green_level[i]);
            }
            break;

        case GREEN_MODE_SOLID:
            /* Keep LED on; log if not yet set */
            if (!s_green_level[i]) {
                s_green_level[i] = 1;
                master_hal_set_green_led(i, 1);
            }
            break;

        case GREEN_MODE_OFF:
        default:
            if (s_green_level[i]) {
                s_green_level[i] = 0;
                master_hal_set_green_led(i, 0);
            }
            break;
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  Public API – reset button poll (called from application timer)            */
/* -------------------------------------------------------------------------- */

void master_mesh_reset_poll(void)
{
    if (master_hal_reset_button_pressed()) {
        if (s_reset_press_start_us == 0) {
            s_reset_press_start_us = esp_timer_get_time();
            ESP_LOGI(TAG, "Reset button pressed – timing...");
        } else {
            int64_t held_ms = (esp_timer_get_time() - s_reset_press_start_us)
                              / 1000;
            if (held_ms >= (int64_t)RESET_HOLD_MS) {
                ESP_LOGW(TAG, "Reset button held %lld ms – mesh local reset",
                         (long long)held_ms);
                esp_ble_mesh_node_local_reset();
                s_reset_press_start_us = 0; /* prevent re-trigger */
            }
        }
    } else {
        s_reset_press_start_us = 0;
    }
}

/* -------------------------------------------------------------------------- */
/*  Public API – mesh init                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t master_mesh_init(void)
{
    ble_mesh_get_dev_uuid(dev_uuid);

    memset(s_node_table,   0, sizeof(s_node_table));
    memset(s_green_mode,   0, sizeof(s_green_mode));
    memset(s_green_level,  0, sizeof(s_green_level));
    memset(s_last_seen_us, 0, sizeof(s_last_seen_us));

    esp_ble_mesh_register_prov_callback(prov_cb);
    esp_ble_mesh_register_config_server_callback(cfg_srv_cb);
    esp_ble_mesh_register_custom_model_callback(custom_model_cb);

    esp_err_t err = esp_ble_mesh_init(&s_provision, &s_composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_mesh_init failed (err=%d)", err);
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV |
                                        ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "node_prov_enable failed (err=%d)", err);
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh master node initialised – waiting for provisioning");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

bool master_mesh_is_provisioned(void)
{
    return s_provisioned;
}
