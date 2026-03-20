#ifndef _BOARD_H_
#define _BOARD_H_

#include "esp_err.h"
#include "driver/gpio.h"

/*
 * Simple board abstraction:
 *  - One status LED (LED_G)
 *  - Two helper functions: board_init() and board_led_operation()
 *
 * Change LED_G to match your board’s GPIO.
 */
#define LED_G   GPIO_NUM_2   /* Example: on many dev boards, GPIO2 has an LED */

#define LED_ON  1
#define LED_OFF 0

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the board status LED (LED_G).
 *
 * Configures LED_G as a push-pull output and turns it OFF.
 */
esp_err_t board_init(void);

/**
 * @brief Turn the given LED on or off.
 *
 * @param led   GPIO pin (e.g., LED_G)
 * @param onoff LED_ON (1) or LED_OFF (0)
 */
void board_led_operation(gpio_num_t led, uint8_t onoff);

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H_ */
