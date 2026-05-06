/**
 * @file master_node_hal.c
 * @brief HAL implementation for master node GPIO.
 */

#include "master_node_hal.h"
#include "master_node_config.h"

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MASTER_HAL";

/* -------------------------------------------------------------------------- */
/*  Pin tables (indexed by alarm node 0..7)                                    */
/* -------------------------------------------------------------------------- */

static const gpio_num_t s_red_pins[MAX_ALARM_NODES]   = MASTER_RED_LEDS;
static const gpio_num_t s_green_pins[MAX_ALARM_NODES] = MASTER_GREEN_LEDS;

/* -------------------------------------------------------------------------- */

void master_hal_gpio_init(void)
{
    gpio_config_t io = {0};

    /* ── Red LED outputs ──────────────────────────────────────────────── */
    uint64_t red_mask = 0;
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        red_mask |= (1ULL << s_red_pins[i]);
    }
    io.intr_type    = GPIO_INTR_DISABLE;
    io.mode         = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = red_mask;
    io.pull_up_en   = 0;
    io.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io));

    /* ── Green LED outputs ────────────────────────────────────────────── */
    uint64_t green_mask = 0;
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        green_mask |= (1ULL << s_green_pins[i]);
    }
    io.pin_bit_mask = green_mask;
    ESP_ERROR_CHECK(gpio_config(&io));

    /* ── Reset button input ───────────────────────────────────────────── */
    io.intr_type    = GPIO_INTR_DISABLE;
    io.mode         = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << GPIO_RESET_BUTTON);
    io.pull_up_en   = 1;
    io.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io));

    /* All LEDs off at start */
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        gpio_set_level(s_red_pins[i],   0);
        gpio_set_level(s_green_pins[i], 0);
    }

    ESP_LOGI(TAG, "Master GPIOs configured (%d red + %d green LEDs)",
             MAX_ALARM_NODES, MAX_ALARM_NODES);
}

/* -------------------------------------------------------------------------- */

void master_hal_set_red_led(uint8_t node_index, uint8_t on)
{
    if (node_index >= MAX_ALARM_NODES) return;
    gpio_set_level(s_red_pins[node_index], on ? 1 : 0);
}

/* -------------------------------------------------------------------------- */

void master_hal_set_green_led(uint8_t node_index, uint8_t on)
{
    if (node_index >= MAX_ALARM_NODES) return;
    gpio_set_level(s_green_pins[node_index], on ? 1 : 0);
}

/* -------------------------------------------------------------------------- */

void master_hal_set_all_red(uint8_t on)
{
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        gpio_set_level(s_red_pins[i], on ? 1 : 0);
    }
}

/* -------------------------------------------------------------------------- */

void master_hal_set_all_green(uint8_t on)
{
    for (int i = 0; i < MAX_ALARM_NODES; i++) {
        gpio_set_level(s_green_pins[i], on ? 1 : 0);
    }
}

/* -------------------------------------------------------------------------- */

bool master_hal_reset_button_pressed(void)
{
    return (gpio_get_level(GPIO_RESET_BUTTON) == 0);
}
