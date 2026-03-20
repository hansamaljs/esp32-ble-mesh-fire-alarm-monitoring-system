#ifndef _BOARD_H_
#define _BOARD_H_

#include "esp_err.h"
#include "driver/gpio.h"

/* Simple board LED abstraction
 * This is a single status LED (not the 8x2 master LEDs)
 * Change GPIO if needed.
 */
#define LED_G   GPIO_NUM_2    /* On-board status LED (example) */

#define LED_ON  1
#define LED_OFF 0

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the board status LED */
esp_err_t board_init(void);

/* Turn the status LED on/off */
void board_led_operation(gpio_num_t led, uint8_t onoff);

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H_ */
