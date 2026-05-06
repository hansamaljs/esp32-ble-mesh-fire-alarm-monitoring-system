/**
 * @file master_node_hal.h
 * @brief HAL for master node LEDs and reset button GPIO.
 *
 * Layered architecture role:
 *   HAL layer – hides all gpio_set_level / gpio_get_level calls from the
 *   application and mesh layers.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "mesh_config.h"   /* for MAX_ALARM_NODES */

/**
 * @brief Initialise all master-node GPIOs.
 *
 * Configures 8 red LEDs, 8 green LEDs (all outputs, initially OFF), and
 * the reset button (input, pull-up).  Must be called once from app_main().
 */
void master_hal_gpio_init(void);

/**
 * @brief Set the red indicator LED for alarm node i.
 * @param node_index  0..MAX_ALARM_NODES-1
 * @param on          1 = illuminate, 0 = extinguish
 */
void master_hal_set_red_led(uint8_t node_index, uint8_t on);

/**
 * @brief Set the green indicator LED for alarm node i.
 * @param node_index  0..MAX_ALARM_NODES-1
 * @param on          1 = illuminate, 0 = extinguish
 */
void master_hal_set_green_led(uint8_t node_index, uint8_t on);

/**
 * @brief Set ALL red LEDs to the given state.
 * @param on  1 = all on, 0 = all off
 */
void master_hal_set_all_red(uint8_t on);

/**
 * @brief Set ALL green LEDs to the given state.
 * @param on  1 = all on, 0 = all off
 */
void master_hal_set_all_green(uint8_t on);

/**
 * @brief Read the reset button.
 * @return true if button is pressed (active-LOW).
 */
bool master_hal_reset_button_pressed(void);
