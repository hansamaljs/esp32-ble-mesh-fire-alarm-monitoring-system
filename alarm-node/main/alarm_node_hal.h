/**
 * @file alarm_node_hal.h
 * @brief Hardware Abstraction Layer for the alarm node GPIO peripherals.
 *
 * Layered architecture role:
 *   HAL layer – abstracts all GPIO reads and writes behind named functions.
 *   The application layer (main.c) never calls gpio_get_level / gpio_set_level
 *   directly; it only calls these HAL functions.
 *
 * Dependency:
 *   alarm_node_config.h  (for GPIO pin constants)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialise all alarm-node GPIO pins.
 *
 * Must be called once from app_main() before any other HAL call.
 * - Alarm input   : input, pull-up, no interrupt (polling).
 * - Red LED       : output, initially OFF.
 * - Green LED     : output, initially OFF.
 * - Reset button  : input, pull-up, ANYEDGE interrupt (attached separately).
 */
void alarm_hal_gpio_init(void);

/**
 * @brief Read the current alarm state from the digital input GPIO.
 * @return 1 if the alarm detector is asserting the line (fire / alarm), 0 otherwise.
 *
 * The fire-alarm detector wired to GPIO_ALARM_INPUT is assumed active-HIGH
 * (the GPIO pull-up keeps the pin LOW when no alarm is present).
 */
uint8_t alarm_hal_read_alarm_state(void);

/**
 * @brief Drive the red LED.
 * @param on  1 to illuminate, 0 to extinguish.
 */
void alarm_hal_set_red_led(uint8_t on);

/**
 * @brief Drive the green LED.
 * @param on  1 to illuminate, 0 to extinguish.
 */
void alarm_hal_set_green_led(uint8_t on);

/**
 * @brief Read the reset button (active-LOW).
 * @return true if the button is currently pressed, false otherwise.
 */
bool alarm_hal_reset_button_pressed(void);

/**
 * @brief Attach the ISR for the reset button GPIO.
 *
 * Installs the GPIO ISR service if not already installed and adds the
 * reset-button handler.  Separated from alarm_hal_gpio_init() so the
 * application can install the ISR after all other peripherals are ready.
 *
 * @param isr_fn  Pointer to the IRAM_ATTR ISR function.
 */
void alarm_hal_install_reset_isr(void (*isr_fn)(void *arg));
