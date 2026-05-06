/**
 * @file master_node_config.h
 * @brief Hardware pin assignments and timing constants for the master node.
 *
 * The master node drives 16 LEDs (8 red + 8 green) to represent 8 alarm nodes:
 *
 *   Red LED [i]   → alarm_state of alarm node i (ON = fire alarm active)
 *   Green LED [i] → connectivity status of alarm node i:
 *                     SOLID ON  = provisioned but no STATUS received yet,
 *                                 OR last STATUS was > NODE_TIMEOUT_US ago
 *                                 (connection likely lost — last known state shown on red)
 *                     BLINKING  = STATUS being received regularly (node alive)
 *                     OFF       = master not provisioned
 *
 * Layered architecture role:
 *   HAL / Configuration layer – no logic, only constants.
 */

#pragma once

#include "driver/gpio.h"

/* -------------------------------------------------------------------------- */
/*  Indicator LED GPIO arrays (index = alarm node index 0..7)                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Red LEDs – one per alarm node.
 * Driven directly by received alarm_state (1 = alarm, 0 = OK).
 * If the node goes offline the last received state is preserved (fail-safe).
 */
#define MASTER_RED_LEDS  \
    {GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15, \
     GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19}

/**
 * @brief Green LEDs – one per alarm node (connectivity indicators).
 */
#define MASTER_GREEN_LEDS \
    {GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_25, \
     GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33}

/* -------------------------------------------------------------------------- */
/*  Reset button                                                               */
/* -------------------------------------------------------------------------- */

/** Reset button (active LOW, internal pull-up). Hold RESET_HOLD_MS to reset. */
#define GPIO_RESET_BUTTON       GPIO_NUM_0

/* -------------------------------------------------------------------------- */
/*  Timing constants                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Inactivity timeout per alarm node (microseconds).
 *
 * If no STATUS is received from alarm node i for this duration,
 * green LED [i] switches from BLINKING to SOLID ON to indicate
 * possible loss of connectivity.  The last received alarm_state on the
 * red LED is preserved so the security room still sees the last known state.
 *
 * Must be > ALARM_STATUS_PERIOD_MS × 1000 (alarm nodes send every 2 s).
 * Set to 2× the alarm period to allow one missed packet before flagging.
 */
#define NODE_TIMEOUT_US         (4ULL * 1000000ULL)   /* 4 seconds */

/**
 * @brief Period of the heartbeat/connectivity LED tick timer (ms).
 * Drives green LED toggling for nodes in BLINK mode.
 */
#define HEARTBEAT_PERIOD_MS     200U

/**
 * @brief Reset button polling period (ms).
 */
#define RESET_POLL_PERIOD_MS    50U

/**
 * @brief How long (ms) to hold the reset button for a local mesh reset.
 */
#define RESET_HOLD_MS           5000U
