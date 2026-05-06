/**
 * @file relay_node_mesh.h
 * @brief BLE Mesh initialisation for the relay node.
 *
 * The relay node is the simplest node in the system:
 *   • The BLE Mesh stack handles all packet rebroadcasting automatically
 *     (relay=ENABLED in Config Server).
 *   • The application code only tracks whether the node has been provisioned
 *     and whether group traffic has been seen, then drives one LED accordingly.
 *
 * The relay node uses a vendor SERVER model so it can be subscribed to
 * the same group as the alarm nodes by the nRF Mesh provisioner.
 * Once subscribed, whenever a STATUS packet is received the
 * OPERATION_EVT fires and the application can note the activity.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

/** Initialise BLE Mesh for relay node role.  Call after bluetooth_init(). */
esp_err_t relay_mesh_init(void);

/** @return true when provisioning is complete. */
bool relay_mesh_is_provisioned(void);

/** @return true when the node has been confirmed as part of the group (received traffic). */
bool relay_mesh_is_in_group(void);

/**
 * @brief Called on every group STATUS received.
 * Records last-activity timestamp and sets in-group flag.
 * Call from the mesh callback via relay_mesh_note_activity().
 */
void relay_mesh_note_activity(void);

/** @return timestamp (µs) of last received group activity, 0 if none. */
uint64_t relay_mesh_last_activity_us(void);

/**
 * @brief Poll reset button.  Call from a periodic timer.
 * Triggers esp_ble_mesh_node_local_reset() after RESET_HOLD_MS.
 */
void relay_mesh_reset_poll(void);
