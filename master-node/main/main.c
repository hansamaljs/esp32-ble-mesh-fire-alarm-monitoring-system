/**
 * @file main.c
 * @brief Master Node – Application layer (entry point and timer orchestration).
 *
 * ╔═══════════════════════════════════════════════════════════╗
 * ║             Master Node – Layered Architecture            ║
 * ╠═══════════════════════════════════════╦═══════════════════╣
 * ║  Layer                                ║  Files            ║
 * ╠═══════════════════════════════════════╬═══════════════════╣
 * ║  Application (orchestration, timers)  ║  main.c           ║
 * ║  Mesh / Transport                     ║  master_node_mesh ║
 * ║  HAL (GPIO abstraction)               ║  master_node_hal  ║
 * ║  Configuration (pins, timing)         ║  master_node_config║
 * ║  Protocol (opcodes, shared types)     ║  mesh_config.h    ║
 * ╚═══════════════════════════════════════╩═══════════════════╝
 *
 * The application layer only:
 *   - Initialises NVS, Bluetooth, HAL, and the mesh layer.
 *   - Creates two periodic timers and routes their ticks to the mesh layer.
 *   - Does NOT touch GPIO or mesh objects directly.
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "ble_mesh_example_init.h"
#include "master_node_config.h"
#include "master_node_hal.h"
#include "master_node_mesh.h"

static const char *TAG = "MASTER_APP";

/* -------------------------------------------------------------------------- */
/*  Timer handles                                                              */
/* -------------------------------------------------------------------------- */

/** Periodic: drives green LED blink/solid logic and timeout detection. */
static esp_timer_handle_t s_heartbeat_timer;

/** Periodic: polls reset button every RESET_POLL_PERIOD_MS. */
static esp_timer_handle_t s_reset_poll_timer;

/* -------------------------------------------------------------------------- */
/*  Timer callbacks                                                            */
/* -------------------------------------------------------------------------- */

static void heartbeat_timer_cb(void *arg)
{
    master_mesh_heartbeat_tick();
}

static void reset_poll_timer_cb(void *arg)
{
    master_mesh_reset_poll();
}

/* -------------------------------------------------------------------------- */
/*  Timer creation                                                             */
/* -------------------------------------------------------------------------- */

static void create_timers(void)
{
    esp_timer_create_args_t hb_args = {
        .callback = heartbeat_timer_cb,
        .name     = "master_heartbeat",
    };
    ESP_ERROR_CHECK(esp_timer_create(&hb_args, &s_heartbeat_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_heartbeat_timer,
                                             HEARTBEAT_PERIOD_MS * 1000ULL));

    esp_timer_create_args_t rst_args = {
        .callback = reset_poll_timer_cb,
        .name     = "master_reset_poll",
    };
    ESP_ERROR_CHECK(esp_timer_create(&rst_args, &s_reset_poll_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_reset_poll_timer,
                                             RESET_POLL_PERIOD_MS * 1000ULL));

    ESP_LOGI(TAG, "Timers created: heartbeat=%ums, reset_poll=%ums",
             HEARTBEAT_PERIOD_MS, RESET_POLL_PERIOD_MS);
}

/* -------------------------------------------------------------------------- */
/*  app_main                                                                   */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "Master node starting...");

    /* 1. NVS flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS full/version mismatch – erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* 2. HAL – configure all 16 LEDs and reset button */
    master_hal_gpio_init();

    /* 3. Bluetooth controller + host */
    err = bluetooth_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluetooth_init failed (err=%d)", err);
        return;
    }

    /* 4. Application timers */
    create_timers();

    /* 5. BLE Mesh stack */
    err = master_mesh_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "master_mesh_init failed (err=%d)", err);
        return;
    }

    ESP_LOGI(TAG, "Master node ready – waiting for provisioning via nRF Mesh");
}
