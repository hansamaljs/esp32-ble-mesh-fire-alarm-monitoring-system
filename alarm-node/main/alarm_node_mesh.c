/**
 * @file alarm_node_mesh.c
 * @brief BLE Mesh stack setup and alarm status publishing for the alarm node.
 */

#include "alarm_node_mesh.h"
#include "alarm_node_config.h"
#include "alarm_node_hal.h"
#include "mesh_config.h"

#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "ble_mesh_example_init.h"

static const char *TAG = "ALARM_MESH";

/* -------------------------------------------------------------------------- */
/*  Module-private state                                                       */
/* -------------------------------------------------------------------------- */

static bool s_provisioned = false;

/* -------------------------------------------------------------------------- */
/*  BLE Mesh stack objects                                                     */
/* -------------------------------------------------------------------------- */

/*
 * dev_uuid: 16-byte UUID advertised during unprovisioned beaconing.
 * The nRF Mesh app identifies this device by UUID and lets you give it a
 * human-readable name before provisioning.
 * The first two bytes are set to {0x32, 0x10}; the rest are filled by
 * ble_mesh_get_dev_uuid() from the ESP32 MAC address.
 */
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};

/*
 * Configuration Server model — mandatory on every mesh node.
 * The provisioner (nRF Mesh app) uses it to:
 *   - Add AppKeys
 *   - Bind AppKeys to the vendor model
 *   - Set the publication address (group address) for the vendor model
 *   - Set subscriptions
 *
 * Relay is ENABLED so alarm nodes also forward mesh traffic and extend range.
 */
static esp_ble_mesh_cfg_srv_t s_config_server = {
    .relay        = ESP_BLE_MESH_RELAY_ENABLED,
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
    .default_ttl  = MESH_DEFAULT_TTL,
    /* Retransmit 3 times with 20 ms interval for reliability */
    .net_transmit     = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

/*
 * Publication context for the vendor server model.
 * Buffer size 8 bytes: 4-byte opcode + 2-byte alarm_status_msg_t payload.
 * The actual publish address is configured at run-time by the provisioner.
 */
ESP_BLE_MESH_MODEL_PUB_DEFINE(s_vnd_pub, 8, ROLE_NODE);

/*
 * Vendor model operation table.
 *
 * The alarm node's vendor SERVER model registers receive handlers for:
 *   VND_OP_MASTER_PKT_SEND  – unicast ping from master (reply back)
 *   VND_OP_MASTER_PKT       – group broadcast from master (log only)
 *   VND_OP_SEND             – kept for compatibility with base example
 *
 * It does NOT register VND_OP_STATUS here because that opcode is what
 * *this node publishes*.  The master's CLIENT model receives it.
 */
static esp_ble_mesh_model_op_t s_vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(VND_OP_SEND,            2),
    ESP_BLE_MESH_MODEL_OP(VND_OP_MASTER_PKT,      2),
    ESP_BLE_MESH_MODEL_OP(VND_OP_MASTER_PKT_SEND, 2),
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
/*  Provisioning callback                                                      */
/* -------------------------------------------------------------------------- */

static void on_prov_complete(uint16_t net_idx, uint16_t addr,
                             uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "Provisioned: net_idx=0x%04x, unicast_addr=0x%04x",
             net_idx, addr);
    s_provisioned = true;
    /* Green LED behaviour after provisioning is driven by the periodic
     * status-publish blink in the application layer (main.c). */
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
        /*
         * Fired after esp_ble_mesh_node_local_reset().
         * Reset state and re-enable provisioning so the device can be
         * re-provisioned by the nRF Mesh app.
         */
        ESP_LOGI(TAG, "NODE_PROV_RESET – re-enabling provisioning");
        s_provisioned = false;
        alarm_hal_set_red_led(0);
        alarm_hal_set_green_led(0);
        esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV |
                                      ESP_BLE_MESH_PROV_GATT);
        break;

    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "SET_UNPROV_DEV_NAME_COMP err=%d",
                 param->node_set_unprov_dev_name_comp.err_code);
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
    if (event != ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        return;
    }

    switch (param->ctx.recv_op) {

    case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        ESP_LOGI(TAG, "CFG: AppKey added net_idx=0x%04x app_idx=0x%04x",
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
/*  Custom (vendor) model callback                                             */
/* -------------------------------------------------------------------------- */

static void custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {

    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        /*
         * VND_OP_MASTER_PKT_SEND: master sent a unicast message to this node.
         * Echo it back as VND_OP_MASTER_PKT_STATUS so the master knows the
         * round-trip path is alive.
         */
        if (param->model_operation.opcode == VND_OP_MASTER_PKT_SEND) {
            uint16_t data = *(uint16_t *)param->model_operation.msg;
            ESP_LOGI(TAG, "Unicast from master: value=%u – echoing back", data);
            esp_err_t err = esp_ble_mesh_server_model_send_msg(
                &s_vnd_models[0],
                param->model_operation.ctx,
                VND_OP_MASTER_PKT_STATUS,
                sizeof(data),
                (uint8_t *)&data);
            if (err) {
                ESP_LOGE(TAG, "Echo reply failed (err=0x%02" PRIx32 ")", (uint32_t)err);
            }
        }

        /*
         * VND_OP_MASTER_PKT: master broadcast to the whole group – log only.
         */
        if (param->model_operation.opcode == VND_OP_MASTER_PKT) {
            uint16_t data = *(uint16_t *)param->model_operation.msg;
            ESP_LOGI(TAG, "Group broadcast from master: value=%u", data);
        }
        break;

    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE(TAG, "Send failed opcode=0x%06" PRIx32 " err=0x%02" PRIx32,
                     (uint32_t)param->model_send_comp.opcode,
                     (uint32_t)param->model_send_comp.err_code);
        }
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t alarm_mesh_init(void)
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

    ESP_LOGI(TAG, "BLE Mesh alarm node initialised – node_index=%d",
             ALARM_NODE_INDEX);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

void alarm_mesh_publish_status(void)
{
    if (!s_provisioned) {
        return;   /* Do nothing before provisioning is complete. */
    }

    alarm_status_msg_t msg = {
        .node_index  = (uint8_t)ALARM_NODE_INDEX,
        .alarm_state = alarm_hal_read_alarm_state(),
    };

    /* Red LED always mirrors live alarm state */
    alarm_hal_set_red_led(msg.alarm_state);

    esp_err_t err = esp_ble_mesh_model_publish(
        &s_vnd_models[0],
        VND_OP_STATUS,
        sizeof(msg),
        (uint8_t *)&msg,
        ROLE_NODE);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "model_publish failed (err=0x%02" PRIx32 ")", (uint32_t)err);
    } else {
        /* Signal the application layer to blink the green LED. */
        alarm_hal_set_green_led(1);
        /*
         * The green LED is turned off by the one-shot FreeRTOS timer
         * in main.c (green_led_off_timer) after GREEN_LED_BLINK_MS.
         * We do not call set_green_led(0) here; that is the app layer's job.
         */
        ESP_LOGD(TAG, "STATUS published: node=%u alarm=%u",
                 msg.node_index, msg.alarm_state);
    }
}

/* -------------------------------------------------------------------------- */

bool alarm_mesh_is_provisioned(void)
{
    return s_provisioned;
}
