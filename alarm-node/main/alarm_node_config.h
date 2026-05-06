/**
 * @file alarm_node_config.h
 * @brief Hardware pin assignments and timing constants for the alarm node.
 *
 * Change the GPIO definitions here to match your physical board wiring.
 * The node index (0-based, 0..7) identifies which alarm this board represents
 * and is compiled in via ALARM_NODE_INDEX.  Set it in the build system or
 * sdkconfig rather than editing this file directly.
 *
 * Layered architecture role:
 *   HAL / Configuration layer – no logic, only constants.
 */

#pragma once

#include "driver/gpio.h"

/* -------------------------------------------------------------------------- */
/*  Node Identity                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Zero-based index of this alarm node (0..MAX_ALARM_NODES-1).
 *
 * Each physical board must be flashed with a unique ALARM_NODE_INDEX.
 * This value is embedded in every STATUS message so the master knows
 * which alarm fired.
 *
 * Override at build time:
 *   idf.py build -DALARM_NODE_INDEX=3
 * or set CONFIG_ALARM_NODE_INDEX in Kconfig.projbuild.
 */
#ifndef ALARM_NODE_INDEX
#define ALARM_NODE_INDEX    0U
#endif

/* -------------------------------------------------------------------------- */
/*  GPIO Pins                                                                  */
/* -------------------------------------------------------------------------- */

/** Digital alarm input from the fire-alarm detector (active HIGH = alarm). */
#define GPIO_ALARM_INPUT        GPIO_NUM_12

/**
 * @brief Analog alarm input pin (ADC channel).
 * Kept for future use / analogue threshold detection.
 * Currently not used in the digital-only implementation.
 */
#define GPIO_ALARM_INPUT_ANALOG GPIO_NUM_35

/** Red LED – mirrors the current alarm_state (ON = alarm active). */
#define GPIO_LED_RED            GPIO_NUM_25

/**
 * @brief Green LED – heartbeat indicator.
 * Blinks briefly (300 ms) every time a STATUS message is published.
 * OFF before provisioning; confirms the mesh publish is working.
 */
#define GPIO_LED_GREEN          GPIO_NUM_23

/**
 * @brief Reset button (active LOW, internal pull-up enabled).
 * Hold for RESET_HOLD_MS to trigger esp_ble_mesh_node_local_reset().
 */
#define GPIO_RESET_BUTTON       GPIO_NUM_0

/* -------------------------------------------------------------------------- */
/*  Timing Constants (milliseconds unless noted)                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief How often (ms) the alarm node publishes a STATUS message.
 *
 * This is also the heartbeat interval that the master node expects.
 * The master's timeout (NODE_TIMEOUT_US in master config) must be
 * larger than this value.
 */
#define ALARM_STATUS_PERIOD_MS  2000U

/**
 * @brief How long (ms) the green LED stays ON after a successful publish.
 * Short blink so it's visible without being distracting.
 */
#define GREEN_LED_BLINK_MS      300U

/**
 * @brief How long (ms) the reset button must be held to trigger a local reset.
 */
#define RESET_HOLD_MS           5000U
