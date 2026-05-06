/**
 * @file relay_node_mesh.c
 * @brief BLE Mesh stack setup for the relay node.
 */

#include "relay_node_mesh.h"
#include "relay_node_config.h"
#include "relay_node_hal.h"
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

static const char *TAG = "RELAY_MESH";

/* -------------------------------------------------------------------------- */
/*  Module state                                                               */
/* -------------------------------------------------------------------------- */

static bool     s_provisioned      = false;
static bool     s_in_group         = false;
static uint64_t s_last_activity_us = 0;
static int64_t  s_reset_press_start_us = 0;

/* -------------------------------------------------------------------------- */
/*  Mesh stack objects                                                         */
/* -------------------------------------------------------------------------- */

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};

static esp_ble_mesh_cfg_srv_t s_config_server = {
    .relay        = ESP_BLE_MESH_RELAY_ENABLED,   /* Core relay function */
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

ESP_BLE_MESH_MODEL_PUB_DEFINE(s_vnd_pub, 8, ROLE_NODE);

static esp_ble_mesh_model_op_t s_vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(VND_OP_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t s_root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&s_config_server),
};

static esp_ble_mesh_model_t s_vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, VND_MODEL_ID_SERVER,
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
/*  Callbacks                                                                  */
/* -------------------------------------------------------------------------- */

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
        ESP_LOGI(TAG, "NODE_PROV_COMPLETE net=0x%04x addr=0x%04x",
                 param->node_prov_complete.net_idx,
                 param->node_prov_complete.addr);
        s_provisioned = true;
        s_in_group    = false;
        s_last_activity_us = 0;
        relay_hal_set_green_led(0); /* LED OFF until group traffic seen */
        break;

    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "NODE_PROV_RESET");
        s_provisioned = false;
        s_in_group    = false;
        s_last_activity_us = 0;
        relay_hal_set_green_led(0);
        esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV |
                                      ESP_BLE_MESH_PROV_GATT);
        break;

    default:
        break;
    }
}

static void cfg_srv_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                       esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) return;

    switch (param->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        ESP_LOGI(TAG, "CFG: AppKey added");
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        ESP_LOGI(TAG, "CFG: AppKey bound");
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        ESP_LOGI(TAG, "CFG: Group subscription added addr=0x%04x",
                 param->value.state_change.mod_sub_add.sub_addr);
        /* Explicitly subscribed to group → confirm in-group */
        if (param->value.state_change.mod_sub_add.company_id == CID_ESP &&
            param->value.state_change.mod_sub_add.model_id == VND_MODEL_ID_SERVER) {
            s_in_group = true;
        }
        break;
    default:
        break;
    }
}

static void custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == VND_OP_STATUS) {
            ESP_LOGI(TAG, "STATUS seen from 0x%04x → activity blink",
                     param->model_operation.ctx->addr);
            relay_mesh_note_activity();
        }
        break;
    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t relay_mesh_init(void)
{
    ble_mesh_get_dev_uuid(dev_uuid);

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

    ESP_LOGI(TAG, "BLE Mesh relay node initialised – waiting for provisioning");
    return ESP_OK;
}

bool relay_mesh_is_provisioned(void)   { return s_provisioned; }
bool relay_mesh_is_in_group(void)      { return s_in_group; }
uint64_t relay_mesh_last_activity_us(void) { return s_last_activity_us; }

void relay_mesh_note_activity(void)
{
    s_in_group         = true;
    s_last_activity_us = (uint64_t)esp_timer_get_time();
}

void relay_mesh_reset_poll(void)
{
    if (relay_hal_reset_button_pressed()) {
        if (s_reset_press_start_us == 0) {
            s_reset_press_start_us = esp_timer_get_time();
        } else {
            int64_t held_ms = (esp_timer_get_time() - s_reset_press_start_us)
                              / 1000;
            if (held_ms >= (int64_t)RESET_HOLD_MS) {
                ESP_LOGW(TAG, "Reset button held %lld ms – local reset",
                         (long long)held_ms);
                esp_ble_mesh_node_local_reset();
                s_reset_press_start_us = 0;
            }
        }
    } else {
        s_reset_press_start_us = 0;
    }
}
