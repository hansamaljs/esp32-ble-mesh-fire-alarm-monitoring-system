/**
 * @file alarm_node_mesh.h
 * @brief BLE Mesh initialisation and message sending for the alarm node.
 *
 * Layered architecture role:
 *   Mesh / Transport layer – owns the BLE Mesh stack objects (composition,
 *   provision struct, model registrations) and the single outgoing message
 *   function used by the application layer.
 *
 * Why only a Vendor SERVER model on the alarm node?
 * ─────────────────────────────────────────────────
 * In BLE Mesh, a *server* model holds state and can both publish and respond
 * to messages.  A *client* model is designed to *initiate* transactions to
 * known server addresses.
 *
 * The alarm node:
 *   • Must PUBLISH status to a group address (not to a specific peer).
 *   • Is provisioned externally by the nRF Mesh app; the alarm node never
 *     needs to discover or address other nodes autonomously.
 *   • Therefore only a server model is needed — the publish address is
 *     configured by the nRF Mesh provisioner and stored in the mesh stack.
 *
 * The master node uses a *client* model ID (0x0000) so it can SUBSCRIBE to
 * the same group address and receive the published STATUS messages.
 *
 * This is exactly the Publish/Subscribe pattern from the Bluetooth Mesh
 * specification:
 *   Alarm (server, pub→group) → [mesh] → Master (client, sub←group)
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
/**
 * @brief Initialise the BLE Mesh stack for the alarm node role.
 *
 * Registers all callbacks, calls esp_ble_mesh_init(), and enables
 * provisioning advertisement.  Must be called after bluetooth_init().
 *
 * @return ESP_OK on success, or an esp_err_t error code.
 */
esp_err_t alarm_mesh_init(void);

/**
 * @brief Publish the current alarm status to the configured group address.
 *
 * Reads the alarm GPIO via the HAL, builds an alarm_status_msg_t,
 * updates the red LED to mirror alarm_state, and calls
 * esp_ble_mesh_model_publish().  The green LED is blinked by the
 * application layer on success.
 *
 * This function is called by the periodic status timer in main.c.
 * It is a no-op if the node is not yet provisioned.
 */
void alarm_mesh_publish_status(void);

/**
 * @brief Return whether the node has been successfully provisioned.
 * @return true once ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT has fired.
 */
bool alarm_mesh_is_provisioned(void);
