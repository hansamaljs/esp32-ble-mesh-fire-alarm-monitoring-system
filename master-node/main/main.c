/* main.c - Master Node (Vendor Client for Alarm System)
 *
 * - BLE Mesh Node with vendor client model (CID 0x02E5, MID 0x0000)
 * - Subscribes to a group where alarm nodes publish STATUS messages
 * - Each alarm node (0..7) has:
 *    - 1 red LED = alarm state (ON/OFF)
 *    - 1 green LED = heartbeat/timeout
 * - Behavior:
 *    - Before provisioning: all 8 green LEDs OFF
 *    - After provisioning:
 *         - all green LEDs SOLID ON (no data / timeout)
 *         - when receiving periodic STATUS from node i: green[i] BLINKS
 *         - if no STATUS for node i for 10s: green[i] returns to SOLID ON
 *    - Red LED[i] ON when alarm_state=1, OFF when alarm_state=0
 * - Reset button (5s) calls esp_ble_mesh_node_local_reset()
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

#define TAG "MASTER"

/* -------------------------------------------------------------------------- */
/*                         BLE Mesh Vendor Model Setup                        */
/* -------------------------------------------------------------------------- */

/* Company ID and Vendor Model IDs (same as alarm node project) */
#define CID_ESP     0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND                  ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS                ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_SEND    ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_STATUS  ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET         ESP_BLE_MESH_MODEL_OP_3(0x04, CID_ESP)

/* Alarm status payload:
 *  node_index : 0..7  (Alarm Node 01..08)
 *  alarm_state: 0 = OFF, 1 = ON
 */

#define MAX_ALARM_NODES   8

typedef struct __attribute__((packed)) {
    uint8_t node_index;
    uint8_t alarm_state;
} alarm_status_msg_t;

typedef struct {
    bool used;
    uint8_t alarm_index;
    uint16_t addr;                              // Primary address
    esp_ble_mesh_msg_ctx_t ctx;                 // Full ctx
} node_entry_t;


/* Allocate enough buffer for opcode + payload (8 bytes is safe) */
ESP_BLE_MESH_MODEL_PUB_DEFINE(vnd_pub, 8, ROLE_NODE);

/* -------------------------------------------------------------------------- */
/*                         Master-specific definitions                        */
/* -------------------------------------------------------------------------- */


/* Change these GPIO pins to match your hardware */
static const gpio_num_t red_led_pins[MAX_ALARM_NODES] = {
    GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15,
    GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19
};

static const gpio_num_t green_led_pins[MAX_ALARM_NODES] = {
    GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_25,
    GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33
};

/* Reset button: active low with pull-up */
#define GPIO_RESET_BUTTON   GPIO_NUM_0

/* Heartbeat / timeout */
#define NODE_TIMEOUT_US         (2ULL * 1000000ULL)   /* 2 seconds */
#define HEARTBEAT_PERIOD_MS     200                   /* 200 ms tick */
#define MASTER_SEND_PERIOD_MS   1000                  /* 1s tick */

/* Reset timing */
#define RESET_HOLD_MS        5000                  /* 5 seconds */

/* Green LED mode per node */
typedef enum {
    GREEN_MODE_OFF = 0,    /* LED off (master not provisioned)    */
    GREEN_MODE_SOLID,      /* LED solid ON (no data / timeout)    */
    GREEN_MODE_BLINK       /* LED blinking (receiving data)       */
} green_mode_t;

/* -------------------------------------------------------------------------- */
/*                               Mesh Objects                                 */
/* -------------------------------------------------------------------------- */

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
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

// static esp_ble_mesh_client_t config_client;

// static const esp_ble_mesh_client_op_pair_t vnd_op_pair[] = {
//     { ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_SEND, ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_STATUS },
// };

// static esp_ble_mesh_client_t vendor_client = {
//     .op_pair_size = ARRAY_SIZE(vnd_op_pair),
//     .op_pair = vnd_op_pair,
// };

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

/* Master only needs to receive STATUS messages from alarm nodes */
static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_CLIENT,
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

/* Node provisioning structure (for BLE Mesh Node role) */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

/* -------------------------------------------------------------------------- */
/*                           Master Node App State                            */
/* -------------------------------------------------------------------------- */

static bool node_provisioned = false;

static green_mode_t green_mode[MAX_ALARM_NODES];
static bool green_level[MAX_ALARM_NODES];                 /* current output level */
static uint64_t last_seen_us[MAX_ALARM_NODES];            /* last STATUS time per node */
//static uint16_t node_addresses[MAX_ALARM_NODES] = {0};    /*connected node addresses*/
static node_entry_t node_table[MAX_ALARM_NODES] = {0};

static esp_timer_handle_t heartbeat_timer;
static esp_timer_handle_t reset_timer;
static esp_timer_handle_t master_send;
static esp_timer_handle_t master_send_to_specific_node_timer;

static int64_t reset_press_start_us = 0;

/* -------------------------------------------------------------------------- */
/*                             GPIO Helper Functions                          */
/* -------------------------------------------------------------------------- */

static inline bool reset_button_pressed(void)
{
    return (gpio_get_level(GPIO_RESET_BUTTON) == 0); /* active low */
}

static void master_gpio_init(void)
{
    gpio_config_t io_conf = {0};

    /* Configure red LEDs */
    uint64_t red_mask = 0;
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        red_mask |= (1ULL << red_led_pins[i]);
    }
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = red_mask;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    /* Configure green LEDs */
    uint64_t green_mask = 0;
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        green_mask |= (1ULL << green_led_pins[i]);
    }
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = green_mask;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    /* Configure reset button */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_RESET_BUTTON);
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    /* Initial LED states:
     *  - All red OFF
     *  - All green OFF (master not provisioned yet)
     */
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        gpio_set_level(red_led_pins[i], 0);
        gpio_set_level(green_led_pins[i], 0);
        green_level[i] = 0;
        green_mode[i] = GREEN_MODE_OFF;
        last_seen_us[i] = 0;
    }

    ESP_LOGI(TAG, "Master GPIOs configured");
}


/* -------------------------------------------------------------------------- */
/*                                  funtions                                  */
/* -------------------------------------------------------------------------- */

static void save_node_ctx(uint8_t node_index, const esp_ble_mesh_msg_ctx_t *src_ctx){

    if (node_index >= MAX_ALARM_NODES) {
        ESP_LOGW(TAG, "Node index out of range");
        return;
    }

    node_entry_t *e = &node_table[node_index];

    e->alarm_index = node_index;
    e->addr = src_ctx->addr;
    //e->ctx = *src_ctx;
    e->used = true;

    // Deep copy ctx content
    memcpy(&e->ctx, src_ctx, sizeof(esp_ble_mesh_msg_ctx_t));

    // ensure proper fields for unicast communication
    e->ctx.send_rel = false;
    e->ctx.send_ttl = 7;         // reasonable TTL
    e->ctx.net_idx  = src_ctx->net_idx;
    e->ctx.app_idx  = src_ctx->app_idx;

    ESP_LOGI(TAG, "Saved ctx for node %u (addr=0x%04X) net idx %04x net--idx %04x ", node_index, e->addr,src_ctx->net_idx,  e->ctx.net_idx);
}

static void send_to_alarm_node(uint8_t alarm_index,const uint8_t *data, uint16_t data_len)
{   
    // esp_ble_mesh_client_common_param_t common = {0};
    uint data_new = alarm_index;

    

    if (alarm_index >= MAX_ALARM_NODES) {
        ESP_LOGW(TAG, "Invalid alarm index %u", alarm_index);
        return;
    }

    node_entry_t *e = &node_table[alarm_index];

    if (!e->used) {
        ESP_LOGW(TAG, "No saved ctx for alarm node %u", alarm_index);
        return;
    }

    ESP_LOGI(TAG, "Sending to node %u, addr 0x%04X, net_idx 0x%04x, app_idx 0x%04x",
         alarm_index, e->ctx.addr, e->ctx.net_idx, e->ctx.app_idx);

    esp_ble_mesh_msg_ctx_t ctx = e->ctx;
    ctx.addr = e->addr;       // ensure correct destination address

    // common.opcode = ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_SEND;
    // common.model = &vendor_client.model;
    // common.ctx.net_idx = ctx.net_idx;
    // common.ctx.app_idx = ctx.app_idx;
    // common.ctx.addr = ctx.addr;
    // common.ctx.send_ttl = ctx.send_ttl;
    // common.ctx.send_rel = ctx.send_rel;
    // common.msg_timeout = 0;
    // common.msg_role = ROLE_NODE;




    esp_err_t err = esp_ble_mesh_server_model_send_msg(
                                        &vnd_models[0],
                                        &ctx,
                                        ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET_SEND,
                                        sizeof(data_new),
                                        (uint8_t *)&data_new);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send to node %u (0x%04X), err=%d",
                 alarm_index, e->addr, err);
    } else {
        ESP_LOGI(TAG, "Sent %d bytes to node %u (0x%04X)",
                 data_len, alarm_index, e->addr);
    }

}




/* -------------------------------------------------------------------------- */
/*                         Timer Callback Functions                           */
/* -------------------------------------------------------------------------- */

static void heartbeat_timer_cb(void *arg)
{  // ESP_LOGI(TAG, "call back triggering");
    if (!node_provisioned) {
        /* While not provisioned, green LEDs remain OFF: do nothing */
        ESP_LOGI(TAG, "NOT provisioned");

        return;
    }

    uint64_t now = (uint64_t)esp_timer_get_time();
    ESP_LOGI(TAG, "provisioned");

    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        if (green_mode[i] == GREEN_MODE_BLINK) {
            /* Check timeout for node i */
            
            if ((last_seen_us[i] > 0) && (now - last_seen_us[i] > NODE_TIMEOUT_US)) {
                /* No data for 10 s -> SOLID ON */
                green_mode[i] = GREEN_MODE_SOLID;
                ESP_LOGI(TAG, "Node %d timeout, green LED solid ON", i);
                gpio_set_level(green_led_pins[i], 1);
                green_level[i] = 1;
                
            } else {
                /* Toggle green LED to indicate heartbeat */
                ESP_LOGI(TAG, "Alarm Node %u SENDING DATA (last seen %llu us)",i, last_seen_us[i]);
                green_level[i] = !green_level[i];
                gpio_set_level(green_led_pins[i], green_level[i]);
            }
        } else if (green_mode[i] == GREEN_MODE_SOLID) {
            /* Ensure LED is solid ON */
            ESP_LOGW(TAG, "Alarm Node %d is NOT SENDING DATA", i);
            if (!green_level[i]) {
                green_level[i] = 1;
                gpio_set_level(green_led_pins[i], 1);
                ESP_LOGW(TAG, "Alarm Node %d is NOT SENDING DATA", i);
            }
        } else { /* GREEN_MODE_OFF */
            ESP_LOGW(TAG, "Alarm Node %d DISCONNECTED.", i);
            /* Ensure LED is OFF */
            if (green_level[i]) {
                green_level[i] = 0;
                gpio_set_level(green_led_pins[i], 0);
                ESP_LOGW(TAG, "Alarm Node %d DISCONNECTED.", i);
            }
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
                ESP_LOGW(TAG, "Reset button held %lld ms, performing mesh local reset",
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

static void master_send_cb(void *arg){
    ESP_LOGI(TAG, "master send data");
    uint16_t msg = random();
    // Use model publish; publish address is set in nRF Mesh app
    esp_err_t err = esp_ble_mesh_model_publish(&vnd_models[0],
                                               ESP_BLE_MESH_VND_MODEL_OP_MASTER_PACKET,
                                               sizeof(msg),
                                               (uint8_t *)&msg,
                                               ROLE_NODE);
    if (err) {
        ESP_LOGE(TAG, "master_esp_ble_mesh_model_publish failed (err 0x%02x)", err);
    }
}

static void master_send_to_specific_node_timer_cb(void *arg){

    uint16_t msg = random();
    //uint8_t i = 4;
    //send_to_alarm_node(i,&msg,sizeof(msg));
    for(uint8_t i = 0; i < MAX_ALARM_NODES; i++){
        send_to_alarm_node(i,&msg,sizeof(msg));
        
        
    }


}

static void create_timers(void)
{
    esp_timer_create_args_t heartbeat_args = {
        .callback = heartbeat_timer_cb,
        .name = "heartbeat_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&heartbeat_args, &heartbeat_timer));
    //ESP_ERROR_CHECK(esp_timer_start_periodic(heartbeat_timer,
    //                                         HEARTBEAT_PERIOD_MS * 1000));

    esp_timer_create_args_t reset_args = {
        .callback = reset_timer_cb,
        .name = "reset_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&reset_args, &reset_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(reset_timer, 50 * 1000)); /* 50ms */


    esp_timer_create_args_t master_send_args = {
        .callback = master_send_cb,
        .name = "master_send",
    };
    ESP_ERROR_CHECK(esp_timer_create(&master_send_args, &master_send));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(master_send,
    //                                          MASTER_SEND_PERIOD_MS * 1000));

                                             
    esp_timer_create_args_t master_send_to_specific_node_args = {
        .callback = master_send_to_specific_node_timer_cb,
        .name = "master_send_to_specific_node_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&master_send_to_specific_node_args, &master_send_to_specific_node_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(master_send_to_specific_node_timer,
                                             MASTER_SEND_PERIOD_MS * 1000));                                         

    ESP_LOGI(TAG, "Timers created and started");
}




/* -------------------------------------------------------------------------- */
/*                        Mesh Callback Implementations                       */
/* -------------------------------------------------------------------------- */

static void prov_complete(uint16_t net_idx, uint16_t addr,
                          uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "Provisioned: net_idx=0x%03x, addr=0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags=0x%02x, iv_index=0x%08x", flags, iv_index);

    node_provisioned = true;

    /* Status LED (on-board) ON to indicate provisioned */
    board_led_operation(LED_G, LED_ON);

    /* After provisioning, green LEDs indicate "no data / timeout" as SOLID ON */
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        green_mode[i] = GREEN_MODE_SOLID;
        green_level[i] = 1;
        gpio_set_level(green_led_pins[i], 1);
        last_seen_us[i] = 0;  /* updated when data arrives */
    }

    ESP_LOGI(TAG, "Master is provisioned: green LEDs now solid ON (no data yet)");
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
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "NODE_PROV_LINK_CLOSE, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
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

        /* Back to unprovisioned: status LED OFF, all app LEDs OFF */
        board_led_operation(LED_G, LED_OFF);
        for (int i = 0; i < MAX_ALARM_NODES; i++) {
            green_mode[i] = GREEN_MODE_OFF;
            green_level[i] = 0;
            gpio_set_level(green_led_pins[i], 0);
            gpio_set_level(red_led_pins[i], 0);
            last_seen_us[i] = 0;
        }
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
            /* Incoming STATUS from an alarm node */
            if (param->model_operation.length < 2) {
                ESP_LOGW(TAG, "STATUS msg too short (%d)", param->model_operation.length);
                return;
            }

            uint8_t node_index = param->model_operation.msg[0];
            uint8_t alarm_state = param->model_operation.msg[1];

            ESP_LOGI(TAG, "STATUS from addr 0x%04x: idx=%u state=%u net_idx 0x%04x, app_idx 0x%04x",
                     param->model_operation.ctx->addr,
                     node_index, alarm_state,param->model_operation.ctx->net_idx,param->model_operation.ctx->app_idx);
            
            
            //node_addresses[node_index] = param->model_operation.ctx->addr;
            save_node_ctx(node_index,param->model_operation.ctx);
            ESP_LOGW(TAG, "printed structure %04x",node_table[4].ctx.net_idx);

            if (node_index >= MAX_ALARM_NODES) {
                ESP_LOGW(TAG, "Node index %u out of range, ignoring", node_index);
                return;
            }

            /* Update red LED (alarm state) */
            gpio_set_level(red_led_pins[node_index],
                           alarm_state ? 1 : 0);

            /* Update heartbeat tracking for green LED */
            last_seen_us[node_index] = (uint64_t)esp_timer_get_time();
            green_mode[node_index] = GREEN_MODE_BLINK;
            /* Blink timer will toggle the level; keep current as starting point */
            ESP_LOGI(TAG, "Node %u heartbeat active -> green LED blinking", node_index);
        }
        break;

    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE(TAG, "MODEL_SEND_COMP: failed, opcode 0x%06x, err=0x%02x",
                     param->model_send_comp.opcode,
                     param->model_send_comp.err_code);
        } else {
            ESP_LOGI(TAG, "MODEL_SEND_COMP: success, opcode 0x%06x",
                     param->model_send_comp.opcode);
        }
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

    /* While unprovisioned: status LED OFF (board_init did this already) */
    ESP_LOGI(TAG, "BLE Mesh Master Node initialized, waiting for provisioning");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                                   main                                     */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Master node starting...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS has no free pages or new version, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Initialize status LED */
    ESP_ERROR_CHECK(board_init());
    board_led_operation(LED_G, LED_OFF);   /* not provisioned yet */

    /* Initialize application GPIOs */
    master_gpio_init();

    /* Bluetooth + Mesh base init */
    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);
    ESP_LOG_BUFFER_HEX("dev_uuid", dev_uuid, ESP_BLE_MESH_OCTET16_LEN);

    /* Timers: heartbeat and reset polling */
    create_timers();

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}
