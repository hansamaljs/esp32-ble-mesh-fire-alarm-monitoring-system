/**
 * @file alarm_node_hal.c
 * @brief HAL implementation – GPIO init, read, and write for the alarm node.
 */

#include "alarm_node_hal.h"
#include "alarm_node_config.h"

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ALARM_HAL";

/* -------------------------------------------------------------------------- */

void alarm_hal_gpio_init(void)
{
    gpio_config_t io = {0};

    /* ── Alarm input: active HIGH, internal pull-up ───────────────────── */
    io.intr_type    = GPIO_INTR_DISABLE;
    io.mode         = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << GPIO_ALARM_INPUT);
    io.pull_up_en   = 1;
    io.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io));

    /* ── LED outputs ──────────────────────────────────────────────────── */
    io.intr_type    = GPIO_INTR_DISABLE;
    io.mode         = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << GPIO_LED_RED) | (1ULL << GPIO_LED_GREEN);
    io.pull_up_en   = 0;
    io.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io));

    /* ── Reset button: active LOW, internal pull-up, edge interrupt ───── */
    io.intr_type    = GPIO_INTR_ANYEDGE;
    io.mode         = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << GPIO_RESET_BUTTON);
    io.pull_up_en   = 1;
    io.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io));

    /* Initial output state: both LEDs off */
    gpio_set_level(GPIO_LED_RED,   0);
    gpio_set_level(GPIO_LED_GREEN, 0);

    ESP_LOGI(TAG, "GPIO init: alarm_in=%d, red=%d, green=%d, reset_btn=%d",
             GPIO_ALARM_INPUT, GPIO_LED_RED, GPIO_LED_GREEN, GPIO_RESET_BUTTON);
}

/* -------------------------------------------------------------------------- */

uint8_t alarm_hal_read_alarm_state(void)
{
    /* Active-HIGH: pin HIGH → alarm present */
    return (uint8_t)(gpio_get_level(GPIO_ALARM_INPUT) ? 1 : 0);
}

/* -------------------------------------------------------------------------- */

void alarm_hal_set_red_led(uint8_t on)
{
    gpio_set_level(GPIO_LED_RED, on ? 1 : 0);
}

/* -------------------------------------------------------------------------- */

void alarm_hal_set_green_led(uint8_t on)
{
    gpio_set_level(GPIO_LED_GREEN, on ? 1 : 0);
}

/* -------------------------------------------------------------------------- */

bool alarm_hal_reset_button_pressed(void)
{
    return (gpio_get_level(GPIO_RESET_BUTTON) == 0);
}

/* -------------------------------------------------------------------------- */

void alarm_hal_install_reset_isr(void (*isr_fn)(void *arg))
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_RESET_BUTTON, isr_fn, NULL));
    ESP_LOGI(TAG, "Reset-button ISR installed on GPIO %d", GPIO_RESET_BUTTON);
}
