/* main.c - Relay Node for Alarm Mesh System
 *
 * Relay indicator LED behavior (GPIO_LED_GREEN):
 *  - Before provisioning: OFF
 *  - After provisioning but before any STATUS seen: OFF
 *  - After first STATUS received (i.e. node participates in group traffic):
 *       - SOLID ON when idle (no recent traffic)
 *       - BLINK for a short window after each STATUS
 *
 * Relay itself is done by BLE Mesh stack (config_server.relay = ENABLED).
 * Reset button: hold 5s -> esp_ble_mesh_node_local_reset().
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

#define TAG "RELAY_NODE"

/* -------------------------------------------------------------------------- */
/*                          Basic Mesh / Vendor Setup                         */
/* -------------------------------------------------------------------------- */

#define CID_ESP     0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

/* Pub context (we don't actually publish, but it's fine to define it) */
ESP_BLE_MESH_MODEL_PUB_DEFINE(vnd_pub, 8, ROLE_NODE);

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

/* Config Server with relay ENABLED */
static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
                              vnd_op, &vnd_pub, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

/* -------------------------------------------------------------------------- */
/*                         Relay-node specific state                          */
/* -------------------------------------------------------------------------- */

/* IMPORTANT:
 * GPIO_LED_GREEN must be the actual LED pin for the relay indicator,
 * and must NOT be the same as LED_G from board.h.
 */
#define GPIO_LED_GREEN          GPIO_NUM_25
#define GPIO_RESET_BUTTON       GPIO_NUM_0    /* Reset button, active-low */

#define LED_ACTIVITY_WINDOW_US  (1000000ULL)  /* 1s blink window after packet */
#define LED_TICK_PERIOD_MS      100           /* LED state tick period */
#define RESET_HOLD_MS           5000          /* 5 seconds to reset */

static bool node_provisioned  = false;
static bool model_subscribed  = false;  /* logical "I'm part of the group now" */

/* LED state */
static bool activity_blink    = false;  /* true while in blink window */
static bool green_level       = false;  /* current output level */
static uint64_t last_activity_us = 0;

static esp_timer_handle_t led_timer;
static esp_timer_handle_t reset_timer;
static int64_t reset_press_start_us = 0;

/* -------------------------------------------------------------------------- */
/*                               GPIO Helpers                                 */
/* -------------------------------------------------------------------------- */

static inline bool reset_button_pressed(void)
{
    return (gpio_get_level(GPIO_RESET_BUTTON) == 0); /* active-low */
}

static void relay_gpio_init(void)
{
    gpio_config_t io_conf = {0};

    /* Relay green LED as output */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_LED_GREEN);
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    /* Reset button as input with pull-up */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_RESET_BUTTON);
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    /* Initial LED state: OFF before provisioning */
    green_level = 0;
    gpio_set_level(GPIO_LED_GREEN, green_level);

    ESP_LOGI(TAG, "Relay GPIOs: green=%d, reset=%d",
             GPIO_LED_GREEN, GPIO_RESET_BUTTON);
}

/* -------------------------------------------------------------------------- */
/*                         Timer Callback Functions                           */
/* -------------------------------------------------------------------------- */

static void led_timer_cb(void *arg)
{
    uint64_t now = (uint64_t)esp_timer_get_time();

    /* Before provisioning: LED must be OFF */
    if (!node_provisioned) {
        if (green_level) {
            green_level = 0;
            gpio_set_level(GPIO_LED_GREEN, green_level);
        }
        activity_blink = false;
        return;
    }

    /* After provisioning, but before we know we're in the group:
     * keep it OFF until model_subscribed becomes true.
     */
    if (!model_subscribed) {
        if (green_level) {
            green_level = 0;
            gpio_set_level(GPIO_LED_GREEN, green_level);
        }
        activity_blink = false;
        return;
    }

    /* After we know we participate in group traffic:
     * - If in activity window: blink
     * - Otherwise: solid ON
     */
    if (activity_blink && (now - last_activity_us < LED_ACTIVITY_WINDOW_US)) {
        green_level = !green_level;
        gpio_set_level(GPIO_LED_GREEN, green_level);
    } else {
        activity_blink = false;
        if (!green_level) {
            green_level = 1;
            gpio_set_level(GPIO_LED_GREEN, green_level);
        }
    }
}

static void reset_timer_cb(void *arg)
{
    bool pressed = reset_button_pressed();

    if (pressed) {
        if (reset_press_start_us == 0) {
            reset_press_start_us = esp_timer_get_time();
            ESP_LOGI(TAG, "Reset button pressed, timing...");
        } else {
            int64_t held_ms = (esp_timer_get_time() - reset_press_start_us) / 1000;
            if (held_ms >= RESET_HOLD_MS) {
                ESP_LOGW(TAG, "Reset button held %lld ms, doing mesh local reset",
                         (long long)held_ms);
                esp_ble_mesh_node_local_reset();
            }
        }
    } else {
        if (reset_press_start_us != 0) {
            ESP_LOGI(TAG, "Reset button released before timeout");
        }
        reset_press_start_us = 0;
    }
}

static void create_timers(void)
{
    esp_timer_create_args_t led_args = {
        .callback = led_timer_cb,
        .name = "relay_led_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&led_args, &led_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(led_timer,
                                             LED_TICK_PERIOD_MS * 1000));

    esp_timer_create_args_t reset_args = {
        .callback = reset_timer_cb,
        .name = "relay_reset_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&reset_args, &reset_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(reset_timer, 50 * 1000)); /* 50 ms */

    ESP_LOGI(TAG, "Relay timers created and started");
}

/* -------------------------------------------------------------------------- */
/*                        Mesh Callback Implementations                       */
/* -------------------------------------------------------------------------- */

static void prov_complete(uint16_t net_idx, uint16_t addr,
                          uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "Provisioned: net_idx=0x%03x, addr=0x%04x",
             net_idx, addr);
    ESP_LOGI(TAG, "flags=0x%02x, iv_index=0x%08x",
             flags, iv_index);

    node_provisioned = true;
    model_subscribed = false;
    activity_blink = false;

    /* Keep relay LED OFF until we know we're part of the group */
    green_level = 0;
    gpio_set_level(GPIO_LED_GREEN, green_level);

    /* Optional: board status LED OFF to show provisioning complete */
    board_led_operation(LED_G, LED_OFF);

    ESP_LOGI(TAG, "Relay node provisioned; waiting for traffic/subscription");
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "PROV_REGISTER_COMP, err_code %d",
                 param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "NODE_PROV_ENABLE_COMP, err_code %d",
                 param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "NODE_PROV_LINK_OPEN, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ?
            "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "NODE_PROV_LINK_CLOSE, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ?
            "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "NODE_PROV_COMPLETE");
        prov_complete(param->node_prov_complete.net_idx,
                      param->node_prov_complete.addr,
                      param->node_prov_complete.flags,
                      param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "NODE_PROV_RESET_EVT");
        node_provisioned = false;
        model_subscribed = false;
        activity_blink = false;
        green_level = 0;
        gpio_set_level(GPIO_LED_GREEN, green_level);
        board_led_operation(LED_G, LED_OFF);
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "SET_UNPROV_DEV_NAME_COMP, err_code %d",
                 param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "APP_KEY_ADD: net_idx 0x%04x, app_idx 0x%04x",
                     param->value.state_change.appkey_add.net_idx,
                     param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey",
                               param->value.state_change.appkey_add.app_key,
                               16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "MODEL_APP_BIND: elem_addr 0x%04x, app_idx 0x%04x, "
                          "cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_app_bind.element_addr,
                     param->value.state_change.mod_app_bind.app_idx,
                     param->value.state_change.mod_app_bind.company_id,
                     param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "MODEL_SUB_ADD: elem_addr 0x%04x, sub_addr 0x%04x, "
                          "cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_sub_add.element_addr,
                     param->value.state_change.mod_sub_add.sub_addr,
                     param->value.state_change.mod_sub_add.company_id,
                     param->value.state_change.mod_sub_add.model_id);
            if (param->value.state_change.mod_sub_add.company_id == CID_ESP &&
                param->value.state_change.mod_sub_add.model_id == ESP_BLE_MESH_VND_MODEL_ID_SERVER) {
                /* If subscription is explicitly set for our vendor model,
                 * we know we're in the group.
                 */
                model_subscribed = true;
                ESP_LOGI(TAG, "Vendor model subscribed via config; relay LED will show activity.");
            }
            break;
        default:
            break;
        }
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_STATUS) {
            if (param->model_operation.length < 2) {
                ESP_LOGW(TAG, "STATUS msg too short (%d)", param->model_operation.length);
                return;
            }

            /* Any time we receive STATUS here, we know we're part of that traffic.
             * Ensure model_subscribed is true, then start activity blink window.
             */
            model_subscribed = true;
            activity_blink = true;
            last_activity_us = (uint64_t)esp_timer_get_time();

            ESP_LOGI(TAG, "STATUS relayed/received from 0x%04x (len=%d) -> blink",
                     param->model_operation.ctx->addr,
                     param->model_operation.length);
        }
        break;

    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        ESP_LOGD(TAG, "MODEL_SEND_COMP_EVT, opcode 0x%06x, err 0x%02x",
                 param->model_send_comp.opcode,
                 param->model_send_comp.err_code);
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/*                             Mesh Init Wrapper                              */
/* -------------------------------------------------------------------------- */

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV |
                                        ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }

    /* Optional: board status LED ON while unprovisioned */
    board_led_operation(LED_G, LED_ON);

    ESP_LOGI(TAG, "BLE Mesh Relay Node initialized, waiting for provisioning");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                                   main                                     */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Relay node starting...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS has no free pages or new version, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Board init (status LED) */
    ESP_ERROR_CHECK(board_init());
    board_led_operation(LED_G, LED_OFF);

    /* Relay GPIOs: relay indicator LED & reset button */
    relay_gpio_init();

    /* Bluetooth + Mesh base init */
    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);
    ESP_LOG_BUFFER_HEX("dev_uuid", dev_uuid, ESP_BLE_MESH_OCTET16_LEN);

    /* Timers: LED state machine + reset polling */
    create_timers();

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}

