/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "driver/adc.h"

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

#include "freertos/timers.h"

#define TAG "EXAMPLE"

#define CID_ESP     0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND                  ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS                ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_SEND    ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_STATUS  ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET         ESP_BLE_MESH_MODEL_OP_3(0x04, CID_ESP)

/* Publication context for vendor server model (2-byte payload) */
ESP_BLE_MESH_MODEL_PUB_DEFINE(vnd_pub, 8, ROLE_NODE);

/* --------- Alarm-node specific definitions -------------------------------- */


/* Alarm node index: 0..7 → Alarm Node 01..08 */
#ifndef ALARM_NODE_INDEX
#define ALARM_NODE_INDEX    1
#endif

/* GPIOs for alarm node (change to match the board) */
#define GPIO_ALARM_INPUT            GPIO_NUM_12   /* alarm digital input */
#define GPIO_ALARM_INPUT_ANALOG     GPIO_NUM_35   /* alarm ANALOG input */
#define GPIO_LED_RED                GPIO_NUM_25  /* red LED: alarm state */
#define GPIO_LED_GREEN              GPIO_NUM_23  /* green LED: heartbeat */
#define GPIO_RESET_BUTTON           GPIO_NUM_0   /* button: hold 5s to reset */

/* Timing (ms) */
#define ALARM_STATUS_PERIOD_MS  2000   /* send status every 2 s */
#define RESET_HOLD_MS           5000   /* 5 second press to reset mesh */

/* Alarm status payload:
 *  node_index : 0..7  (Alarm Node 01..08)
 *  alarm_state: 0 = OFF, 1 = ON
 */
typedef struct __attribute__((packed)) {
    uint8_t node_index;
    uint8_t alarm_state;
    //uint16_t alarm_state_analog;
} alarm_status_msg_t;



/* -------------------------------------------------------------------------- */

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

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
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_SEND, 2),

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



/* ----------------- Alarm node state & timers ------------------------------- */

static bool node_provisioned = false;

static esp_timer_handle_t status_timer;

static TimerHandle_t green_led_off_timer = NULL;
static TimerHandle_t local_reset_timer = NULL;


/* ----------------- Function Prototypes ------------------------------- */


static void send_alarm_status(void);
static inline bool get_reset_button_state(void);
static void IRAM_ATTR reset_button_isr_handler(void *arg);


/* ----------------- Alarm node GPIO setup ----------------------------------- */

static void alarm_gpio_init(void)
{
    gpio_config_t io_conf = {0};

    // Alarm input
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_ALARM_INPUT);
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // LEDs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_LED_RED) | (1ULL << GPIO_LED_GREEN);
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Reset button
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_RESET_BUTTON);
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_RESET_BUTTON, reset_button_isr_handler, NULL));

    // ESP_LOGI(TAG, "Reset button interrupt initialized on GPIO %d", GPIO_RESET_BUTTON);

    // Initial LED states:
    //  - Red OFF (no alarm)
    // Green LED OFF before provisioning
    gpio_set_level(GPIO_LED_RED, 0);
    gpio_set_level(GPIO_LED_GREEN, 0);

    ESP_LOGI(TAG, "Alarm GPIOs in=%d, red=%d, green=%d, reset=%d",
             GPIO_ALARM_INPUT, GPIO_LED_RED, GPIO_LED_GREEN, GPIO_RESET_BUTTON);
}


/* --------------  Timer callbacks  -------------------- */

static void status_timer_cb(void *arg)
{
    if (!node_provisioned) {
        return;
    }
    
    send_alarm_status();
}

//excecutes when 300ms is gone.
static void green_led_off_timer_cb(TimerHandle_t xTimer)
{
    gpio_set_level(GPIO_LED_GREEN, 0);
}

//do local reset after 5s
static void local_reset_timer_cb(TimerHandle_t xTimer)
{
    if(get_reset_button_state() == 1)
    {
        ESP_LOGW(TAG, "Reset button held for %d ms, doing local reset", RESET_HOLD_MS);
        esp_ble_mesh_node_local_reset();
    }
    
}


/*  -------------------     Create Timers      ---------------------- */

static void create_timers(void)
{
    esp_timer_create_args_t status_args = {
        .callback = status_timer_cb,
        .name = "status_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&status_args, &status_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(status_timer, ALARM_STATUS_PERIOD_MS * 1000));


    //create one time timer for green led blink
    green_led_off_timer = xTimerCreate(
        "green_led_off",
        pdMS_TO_TICKS(300),
        pdFALSE,
        NULL,
        green_led_off_timer_cb );

    if ( green_led_off_timer == NULL )
    {
        ESP_LOGE(TAG, "faild to create green led off timer");
    }


    //create one time timer for local reset
    local_reset_timer = xTimerCreate(
        "local_reset",
        pdMS_TO_TICKS(RESET_HOLD_MS),
        pdFALSE,
        NULL,
        local_reset_timer_cb );
    
    if ( local_reset_timer == NULL )
    {
        ESP_LOGE(TAG, "faild to create local reset timer");
    }
}

/* --------------------  User Defined Functions    ---------------- */


// read alarm digital state
static uint8_t read_alarm_state(void)
{
    return gpio_get_level(GPIO_ALARM_INPUT) ? 1 : 0;
}

//active low helper (reset button is active low)
static inline bool get_reset_button_state(void)
{
    return (gpio_get_level(GPIO_RESET_BUTTON) == 0);
}

//eneable inturrupts
static void reset_button_interrupt_init(void)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_RESET_BUTTON, reset_button_isr_handler, NULL));

    ESP_LOGI(TAG, "Reset button interrupt initialized on GPIO %d", GPIO_RESET_BUTTON);
}


//alarm status send
static void send_alarm_status(void)
{
    alarm_status_msg_t msg;
    msg.node_index = ALARM_NODE_INDEX;
    msg.alarm_state = read_alarm_state();

    // Reflect alarm state on red LED
    gpio_set_level(GPIO_LED_RED, msg.alarm_state ? 1 : 0);

    // Use model publish; publish address is set in nRF Mesh app
    esp_err_t err = esp_ble_mesh_model_publish(&vnd_models[0],
                                               ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                                               sizeof(msg),
                                               (uint8_t *)&msg,
                                               ROLE_NODE);

    if (err) {
        ESP_LOGE(TAG, "esp_ble_mesh_model_publish failed (err 0x%02x)", err);
    }
    
    else
    {
        //blink the green LED per publish
        gpio_set_level(GPIO_LED_GREEN, 1);
        
        if(green_led_off_timer != NULL)
        {
            xTimerReset(green_led_off_timer, 0);
        }
    }
}



/* ----------------- Provisioning & model callbacks -------------------------- */

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%" PRIu16 ", addr 0x%" PRIu16 "", net_idx, addr);
    //ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);
   
    // Alarm-node behavior: start blinking green LED
    node_provisioned = true;
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        node_provisioned = false;

        gpio_set_level(GPIO_LED_RED, 0);
        gpio_set_level(GPIO_LED_GREEN, 0);
        esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);

        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
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
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);

            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
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
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_SEND) {
            // Original example behavior still here (not really used for alarm node)
            uint16_t data = *(uint16_t *)param->model_operation.msg;
            ESP_LOGI(TAG, "Received unicast random number %u from master ", data);
            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_STATUS,
                    sizeof(data), (uint8_t *)&data);
            if (err) {
                ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }

        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET) {
            ESP_LOGI(TAG,"DATA FROM MASTER");
            ESP_LOGI(TAG,"DATA FROM MASTER %u",*(uint16_t *)param->model_operation.msg);
        }


        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            //ESP_LOGE(TAG, "Failed to send message 0x%06x", param->model_send_comp.opcode);
            break;
        }
        //ESP_LOGI(TAG, "Send 0x%06x", param->model_send_comp.opcode);
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    board_init();

    // Alarm-specific GPIOs
    alarm_gpio_init();

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    // Start timers (status publish, blink, reset button scanning)
    create_timers();
    reset_button_interrupt_init();

}


/*  ------------------   GPIO ISRs   ----------------------  */

static void IRAM_ATTR reset_button_isr_handler(void *arg)
{   
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ( get_reset_button_state() == 1 ){

        if (local_reset_timer != NULL) 
        {
            xTimerResetFromISR(local_reset_timer, &xHigherPriorityTaskWoken);
        }

    } else {

        if (local_reset_timer != NULL) 
        {
            xTimerStopFromISR(local_reset_timer, &xHigherPriorityTaskWoken);
        }
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


