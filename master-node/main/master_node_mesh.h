/**
 * @file master_node_mesh.h
 * @brief BLE Mesh layer for the master node.
 *
 * Layered architecture role:
 *   Mesh / Transport layer – owns the vendor CLIENT model, receives STATUS
 *   messages from alarm nodes, stores per-node context for unicast replies,
 *   and drives the HAL to update indicator LEDs.
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                Why CLIENT model on the master?                   │
 * │                                                                  │
 * │  BLE Mesh model naming from the spec:                            │
 * │    • Server model  → holds state, responds to GET/SET, publishes │
 * │    • Client model  → subscribes, receives published messages     │
 * │                                                                  │
 * │  The master needs to SUBSCRIBE to the group address and RECEIVE  │
 * │  alarm status updates.  It never holds alarm state itself.       │
 * │  Therefore, a CLIENT model with VND_MODEL_ID_CLIENT (0x0000) is  │
 * │  the correct choice.                                             │
 * │                                                                  │
 * │  Additionally, using model ID 0x0000 on the master and 0x0001    │
 * │  on alarm nodes makes it possible to subscribe both to the same  │
 * │  group while keeping them distinguishable in the provisioner.    │
 * │                                                                  │
 * │  The master is NOT a provisioner.  It is provisioned by the same │
 * │  nRF Mesh app as every alarm node, then subscribed to the same   │
 * │  group.  Incoming STATUS messages from that group are received   │
 * │  via the OPERATION_EVT callback.                                 │
 * │                                                                  │
 * │  Why not a server model on the master?                           │
 * │  A server model CAN receive messages, but its intended use is to │
 * │  hold device state and serve GET/SET requests.  Using a client   │
 * │  model on the subscriber side is semantically correct and avoids │
 * │  confusion when using nRF Mesh (which shows client/server         │
 * │  distinctly in its UI).                                          │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │           Real-time Connectivity Detection                       │
 * │                                                                  │
 * │  The master actively monitors the heartbeat of each alarm node  │
 * │  using a per-node timestamp (last_seen_us[]).                    │
 * │                                                                  │
 * │  Every 2 s each alarm node publishes a STATUS message.          │
 * │  The master records esp_timer_get_time() whenever it receives   │
 * │  a STATUS from node i.                                           │
 * │                                                                  │
 * │  A periodic heartbeat timer (HEARTBEAT_PERIOD_MS) checks every  │
 * │  node:                                                           │
 * │    if now - last_seen_us[i] > NODE_TIMEOUT_US                   │
 * │       → green LED[i] = SOLID ON  (node silent / offline)        │
 * │    else                                                          │
 * │       → green LED[i] = BLINK    (node alive and reporting)      │
 * │                                                                  │
 * │  The red LED[i] is NEVER cleared by a timeout – it keeps the    │
 * │  last received alarm_state.  This is intentional: if a node     │
 * │  goes offline while its alarm was active, the master must still  │
 * │  show the alarm (fail-safe for a critical fire-alarm system).    │
 * └─────────────────────────────────────────────────────────────────┘
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialise the BLE Mesh stack for the master node role.
 *
 * Registers all callbacks, calls esp_ble_mesh_init(), and enables
 * provisioning advertisement.  Must be called after bluetooth_init().
 *
 * @return ESP_OK on success, or an esp_err_t error code.
 */
esp_err_t master_mesh_init(void);

/**
 * @brief Tick function – call from heartbeat timer.
 *
 * Checks each alarm node's last_seen timestamp and updates green LEDs:
 *   GREEN_MODE_BLINK   → node is reporting (toggle LED each tick)
 *   GREEN_MODE_SOLID   → node timed out (LED solid ON, last alarm state kept)
 *   GREEN_MODE_OFF     → master not provisioned (LED off)
 */
void master_mesh_heartbeat_tick(void);

/**
 * @brief Poll reset button – call from reset-poll timer.
 *
 * Tracks press duration.  Calls esp_ble_mesh_node_local_reset() when
 * the button has been held for RESET_HOLD_MS.
 */
void master_mesh_reset_poll(void);

/**
 * @brief Return whether the master node has been provisioned.
 */
bool master_mesh_is_provisioned(void);
