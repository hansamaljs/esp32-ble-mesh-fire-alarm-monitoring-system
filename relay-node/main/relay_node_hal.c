/**
 * @file relay_node_hal.c
 * @brief HAL implementation for relay node GPIO.
 */

#include "relay_node_hal.h"
#include "relay_node_config.h"

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "RELAY_HAL";

void relay_hal_gpio_init(void)
{
    gpio_config_t io = {0};

    /* Green activity LED */
    io.intr_type    = GPIO_INTR_DISABLE;
    io.mode         = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << GPIO_LED_GREEN);
    io.pull_up_en   = 0;
    io.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io));

    /* Reset button */
    io.intr_type    = GPIO_INTR_DISABLE;
    io.mode         = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << GPIO_RESET_BUTTON);
    io.pull_up_en   = 1;
    io.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_level(GPIO_LED_GREEN, 0);
    ESP_LOGI(TAG, "Relay GPIOs: green=%d, reset=%d",
             GPIO_LED_GREEN, GPIO_RESET_BUTTON);
}

void relay_hal_set_green_led(uint8_t on)
{
    gpio_set_level(GPIO_LED_GREEN, on ? 1 : 0);
}

bool relay_hal_reset_button_pressed(void)
{
    return (gpio_get_level(GPIO_RESET_BUTTON) == 0);
}
