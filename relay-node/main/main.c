/**
 * @file main.c
 * @brief Relay Node – Application layer (entry point and LED state machine).
 *
 * ╔═══════════════════════════════════════════════════════════╗
 * ║             Relay Node – Layered Architecture             ║
 * ╠═══════════════════════════════════════╦═══════════════════╣
 * ║  Layer                                ║  Files            ║
 * ╠═══════════════════════════════════════╬═══════════════════╣
 * ║  Application (timers, LED FSM)        ║  main.c           ║
 * ║  Mesh / Transport                     ║  relay_node_mesh  ║
 * ║  HAL (GPIO abstraction)               ║  relay_node_hal   ║
 * ║  Configuration (pins, timing)         ║  relay_node_config║
 * ║  Protocol (opcodes, shared types)     ║  mesh_config.h    ║
 * ╚═══════════════════════════════════════╩═══════════════════╝
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "ble_mesh_example_init.h"
#include "relay_node_config.h"
#include "relay_node_hal.h"
#include "relay_node_mesh.h"

static const char *TAG = "RELAY_APP";

/* -------------------------------------------------------------------------- */
/*  LED state machine                                                          */
/* -------------------------------------------------------------------------- */

static bool s_led_level = false;   /* current LED output state */

/**
 * @brief Activity LED finite-state machine, called every LED_TICK_PERIOD_MS.
 *
 * States:
 *   Not provisioned         → LED off
 *   Provisioned, not in group → LED off (waiting for subscription traffic)
 *   In group, activity recent  → LED BLINK (toggle each tick)
 *   In group, activity stale   → LED SOLID ON
 */
static void led_fsm_tick(void)
{
    if (!relay_mesh_is_provisioned() || !relay_mesh_is_in_group()) {
        if (s_led_level) {
            s_led_level = false;
            relay_hal_set_green_led(0);
        }
        return;
    }

    uint64_t now    = (uint64_t)esp_timer_get_time();
    uint64_t last   = relay_mesh_last_activity_us();
    bool in_window  = (last > 0) && ((now - last) < LED_ACTIVITY_WINDOW_US);

    if (in_window) {
        /* Blink during activity window */
        s_led_level = !s_led_level;
        relay_hal_set_green_led(s_led_level ? 1 : 0);
    } else {
        /* Solid ON when idle but in group */
        if (!s_led_level) {
            s_led_level = true;
            relay_hal_set_green_led(1);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  Timer callbacks                                                            */
/* -------------------------------------------------------------------------- */

static void led_timer_cb(void *arg)
{
    led_fsm_tick();
}

static void reset_poll_timer_cb(void *arg)
{
    relay_mesh_reset_poll();
}

/* -------------------------------------------------------------------------- */
/*  Timer creation                                                             */
/* -------------------------------------------------------------------------- */

static esp_timer_handle_t s_led_timer;
static esp_timer_handle_t s_reset_poll_timer;

static void create_timers(void)
{
    esp_timer_create_args_t led_args = {
        .callback = led_timer_cb,
        .name     = "relay_led",
    };
    ESP_ERROR_CHECK(esp_timer_create(&led_args, &s_led_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_led_timer,
                                             LED_TICK_PERIOD_MS * 1000ULL));

    esp_timer_create_args_t rst_args = {
        .callback = reset_poll_timer_cb,
        .name     = "relay_reset",
    };
    ESP_ERROR_CHECK(esp_timer_create(&rst_args, &s_reset_poll_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_reset_poll_timer,
                                             RESET_POLL_PERIOD_MS * 1000ULL));

    ESP_LOGI(TAG, "Timers created: led=%ums, reset_poll=%ums",
             LED_TICK_PERIOD_MS, RESET_POLL_PERIOD_MS);
}

/* -------------------------------------------------------------------------- */
/*  app_main                                                                   */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "Relay node starting...");

    /* 1. NVS flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS full/version mismatch – erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* 2. HAL */
    relay_hal_gpio_init();

    /* 3. Bluetooth controller + host */
    err = bluetooth_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluetooth_init failed (err=%d)", err);
        return;
    }

    /* 4. Timers */
    create_timers();

    /* 5. BLE Mesh */
    err = relay_mesh_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "relay_mesh_init failed (err=%d)", err);
        return;
    }

    ESP_LOGI(TAG, "Relay node ready – waiting for provisioning via nRF Mesh");
}
