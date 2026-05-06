/**
 * @file main.c
 * @brief Alarm Node – Application layer (entry point and timer orchestration).
 *
 * ╔═══════════════════════════════════════════════════════════╗
 * ║             Alarm Node – Layered Architecture             ║
 * ╠═══════════════════════════════════════╦═══════════════════╣
 * ║  Layer                                ║  Files            ║
 * ╠═══════════════════════════════════════╬═══════════════════╣
 * ║  Application (orchestration, timers)  ║  main.c           ║
 * ║  Mesh / Transport                     ║  alarm_node_mesh  ║
 * ║  HAL (GPIO abstraction)               ║  alarm_node_hal   ║
 * ║  Configuration (pins, timing)         ║  alarm_node_config║
 * ║  Protocol (opcodes, shared types)     ║  mesh_config.h    ║
 * ╚═══════════════════════════════════════╩═══════════════════╝
 *
 * Behaviour summary:
 *  • Before provisioning  : LEDs off, no messages sent.
 *  • After provisioning   :
 *      – Every ALARM_STATUS_PERIOD_MS, publishes alarm_status_msg_t to the
 *        configured group address using VND_OP_STATUS.
 *      – Green LED blinks for GREEN_LED_BLINK_MS after each successful publish.
 *      – Red LED mirrors the live alarm input state.
 *      – Hold reset button RESET_HOLD_MS → esp_ble_mesh_node_local_reset().
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "ble_mesh_example_init.h"
#include "esp_ble_mesh_networking_api.h"

#include "alarm_node_config.h"
#include "alarm_node_hal.h"
#include "alarm_node_mesh.h"

static const char *TAG = "ALARM_APP";

/* -------------------------------------------------------------------------- */
/*  Timer handles                                                              */
/* -------------------------------------------------------------------------- */

/** Periodic esp_timer: fires every ALARM_STATUS_PERIOD_MS. */
static esp_timer_handle_t s_status_timer;

/**
 * One-shot FreeRTOS timer: turns the green LED off GREEN_LED_BLINK_MS
 * after it was switched on by the mesh layer following a successful publish.
 */
static TimerHandle_t s_green_led_off_timer;

/**
 * One-shot FreeRTOS timer: started on button-press, fires after
 * RESET_HOLD_MS to perform the local mesh reset.
 */
static TimerHandle_t s_local_reset_timer;

/* -------------------------------------------------------------------------- */
/*  Timer callbacks                                                            */
/* -------------------------------------------------------------------------- */

/** Called every ALARM_STATUS_PERIOD_MS from the esp_timer ISR context. */
static void status_timer_cb(void *arg)
{
    alarm_mesh_publish_status();
}

/** Called RESET_HOLD_MS after the reset button was pressed. */
static void local_reset_timer_cb(TimerHandle_t xTimer)
{
    if (alarm_hal_reset_button_pressed()) {
        ESP_LOGW(TAG, "Reset button held – triggering local mesh reset");
        esp_ble_mesh_node_local_reset();
    }
}

/** Called GREEN_LED_BLINK_MS after green LED was lit. */
static void green_led_off_timer_cb(TimerHandle_t xTimer)
{
    alarm_hal_set_green_led(0);
}

/* -------------------------------------------------------------------------- */
/*  Reset button ISR (IRAM-safe)                                               */
/* -------------------------------------------------------------------------- */

static void IRAM_ATTR reset_button_isr(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;

    if (alarm_hal_reset_button_pressed()) {
        /* Button just pressed: start countdown */
        if (s_local_reset_timer != NULL) {
            xTimerResetFromISR(s_local_reset_timer, &higher_prio_woken);
        }
    } else {
        /* Button released before timeout: cancel */
        if (s_local_reset_timer != NULL) {
            xTimerStopFromISR(s_local_reset_timer, &higher_prio_woken);
        }
    }

    if (higher_prio_woken) {
        portYIELD_FROM_ISR();
    }
}

/* -------------------------------------------------------------------------- */
/*  Timer creation                                                             */
/* -------------------------------------------------------------------------- */

static void create_timers(void)
{
    /* Status publish timer – esp_timer (high-resolution, ISR context) */
    esp_timer_create_args_t status_args = {
        .callback = status_timer_cb,
        .name     = "alarm_status",
    };
    ESP_ERROR_CHECK(esp_timer_create(&status_args, &s_status_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_status_timer,
                                             ALARM_STATUS_PERIOD_MS * 1000ULL));

    /* Green LED off timer – one-shot FreeRTOS timer */
    s_green_led_off_timer = xTimerCreate(
        "green_off",
        pdMS_TO_TICKS(GREEN_LED_BLINK_MS),
        pdFALSE,        /* one-shot */
        NULL,
        green_led_off_timer_cb);
    if (s_green_led_off_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create green_led_off_timer");
    }

    /* Local reset timer – one-shot FreeRTOS timer */
    s_local_reset_timer = xTimerCreate(
        "local_reset",
        pdMS_TO_TICKS(RESET_HOLD_MS),
        pdFALSE,        /* one-shot */
        NULL,
        local_reset_timer_cb);
    if (s_local_reset_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create local_reset_timer");
    }

    ESP_LOGI(TAG, "Timers created: status=%ums, green_blink=%ums, reset=%ums",
             ALARM_STATUS_PERIOD_MS, GREEN_LED_BLINK_MS, RESET_HOLD_MS);
}

/* -------------------------------------------------------------------------- */
/*  app_main                                                                   */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "Alarm node starting (node_index=%d)...", ALARM_NODE_INDEX);

    /* 1. NVS flash – required by BLE Mesh to persist provisioning data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS full/version mismatch – erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* 2. HAL – configure all GPIOs */
    alarm_hal_gpio_init();

    /* 3. Bluetooth controller + host (Bluedroid / NimBLE) */
    err = bluetooth_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluetooth_init failed (err=%d)", err);
        return;
    }

    /* 4. BLE Mesh stack */
    err = alarm_mesh_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "alarm_mesh_init failed (err=%d)", err);
        return;
    }

    /* 5. Application timers */
    create_timers();

    /* 6. Reset-button interrupt */
    alarm_hal_install_reset_isr(reset_button_isr);

    ESP_LOGI(TAG, "Alarm node ready – waiting for provisioning via nRF Mesh");
}
