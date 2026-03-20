#include "board.h"
#include "driver/gpio.h"

esp_err_t board_init(void)
{
    gpio_config_t io_conf = {0};

    /* Configure status LED pin as output */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_G);
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    /* Start with LED OFF (unprovisioned / idle state) */
    gpio_set_level(LED_G, LED_OFF);

    return ESP_OK;
}

void board_led_operation(gpio_num_t led, uint8_t onoff)
{
    gpio_set_level(led, onoff ? 1 : 0);
}
