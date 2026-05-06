/**
 * @file relay_node_hal.h
 * @brief HAL for relay node GPIO (single LED + reset button).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Initialise relay node GPIOs.  Call once from app_main(). */
void relay_hal_gpio_init(void);

/** Set the relay activity LED. @param on 1 = ON, 0 = OFF */
void relay_hal_set_green_led(uint8_t on);

/** @return true if reset button is currently pressed (active-LOW). */
bool relay_hal_reset_button_pressed(void);
