/**
 * @file mesh_config.h
 * @brief BLE Mesh vendor model opcodes, company ID, and shared message types.
 *
 * This header is shared in concept across all node types. Each node project
 * copies it so they stay independent IDF components. All opcodes, the CID,
 * and the on-wire payload struct live here so the protocol is defined in one
 * place per firmware.
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │                   Vendor Model Protocol                     │
 * │                                                             │
 * │  Opcode 0x00 (OP_SEND)            – reserved / generic      │
 * │  Opcode 0x01 (OP_STATUS)          – alarm node → group      │
 * │  Opcode 0x02 (OP_MASTER_PKT_SEND) – master → alarm (unicast)│
 * │  Opcode 0x03 (OP_MASTER_PKT_STATUS)– alarm → master reply   │
 * │  Opcode 0x04 (OP_MASTER_PKT)      – master → group publish  │
 * └─────────────────────────────────────────────────────────────┘
 *
 * Payload for OP_STATUS (2 bytes):
 *   [0] node_index  – 0..7, identifies which alarm node sent this
 *   [1] alarm_state – 0 = OK, 1 = ALARM
 */

#pragma once

#include <stdint.h>
#include "esp_ble_mesh_defs.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* -------------------------------------------------------------------------- */
/*  Company & Model IDs                                                        */
/* -------------------------------------------------------------------------- */

/** Espressif company ID used in all vendor models of this project. */
#define CID_ESP                             0x02E5U

/** Vendor Model ID used on alarm nodes and relay nodes (sender side). */
#define VND_MODEL_ID_SERVER                 0x0001U

/** Vendor Model ID used on the master node (receiver / client side). */
#define VND_MODEL_ID_CLIENT                 0x0000U

/* -------------------------------------------------------------------------- */
/*  Vendor Opcodes  (3-byte opcode: 0xCx + CID_ESP)                           */
/* -------------------------------------------------------------------------- */

/** Generic send opcode – kept for compatibility with base example. */
#define VND_OP_SEND             ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)

/** Alarm node publishes its status using this opcode every 2 s. */
#define VND_OP_STATUS           ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

/** Master sends a unicast ping/data packet to a specific alarm node. */
#define VND_OP_MASTER_PKT_SEND  ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)

/** Alarm node replies to a master unicast ping. */
#define VND_OP_MASTER_PKT_STATUS ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)

/** Master publishes a packet to the group (broadcast). */
#define VND_OP_MASTER_PKT       ESP_BLE_MESH_MODEL_OP_3(0x04, CID_ESP)

/* -------------------------------------------------------------------------- */
/*  On-wire payload struct                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Status message sent by every alarm node every ALARM_STATUS_PERIOD_MS.
 *
 * Packed so it maps directly onto the raw BLE Mesh payload bytes.
 *   Byte 0 → node_index  (0-based, 0..7)
 *   Byte 1 → alarm_state (0 = OK, 1 = ALARM)
 */
typedef struct __attribute__((packed)) {
    uint8_t node_index;   /**< Which alarm node sent this (0..MAX_ALARM_NODES-1). */
    uint8_t alarm_state;  /**< 0 = no alarm, 1 = alarm detected. */
} alarm_status_msg_t;

/* -------------------------------------------------------------------------- */
/*  System-wide constants                                                      */
/* -------------------------------------------------------------------------- */

/** Total number of alarm nodes in the system. */
#define MAX_ALARM_NODES     8U

/** Default BLE Mesh TTL for all outgoing messages. */
#define MESH_DEFAULT_TTL    7U
