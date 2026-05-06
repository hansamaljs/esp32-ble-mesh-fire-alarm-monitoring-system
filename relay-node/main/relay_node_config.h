/**
 * @file relay_node_config.h
 * @brief Hardware and timing configuration for the relay node.
 *
 * The relay node's primary role is mesh range extension: the BLE Mesh stack
 * automatically rebroadcasts packets it receives (because relay=ENABLED in
 * the Config Server).  The application code only needs to drive one status LED
 * and handle the reset button.
 *
 * Layered architecture role:
 *   HAL / Configuration layer – no logic, only constants.
 */

#pragma once

#include "driver/gpio.h"

/* -------------------------------------------------------------------------- */
/*  GPIOs                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Green LED – relay activity indicator.
 *   OFF           – not provisioned
 *   OFF           – provisioned but no group traffic seen yet
 *   SOLID ON      – provisioned and in group, no recent traffic
 *   BLINKING      – group traffic being received/relayed actively
 */
#define GPIO_LED_GREEN      GPIO_NUM_25

/** Reset button (active LOW, internal pull-up). */
#define GPIO_RESET_BUTTON   GPIO_NUM_0

/* -------------------------------------------------------------------------- */
/*  Timing constants                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief After receiving group traffic, how long to stay in blink mode (µs).
 * After this window with no new traffic the LED returns to SOLID ON.
 */
#define LED_ACTIVITY_WINDOW_US  (1000000ULL)   /* 1 second */

/** LED state-machine tick period (ms). */
#define LED_TICK_PERIOD_MS      100U

/** Reset button polling period (ms). */
#define RESET_POLL_PERIOD_MS    50U

/** How long (ms) to hold reset button to trigger local reset. */
#define RESET_HOLD_MS           5000U
