# ESP32 BLE Mesh Fire Alarm Monitoring System

A distributed fire alarm monitoring system built using ESP32 and Bluetooth Low Energy Mesh.

## Repository Structure

- `alarm-node/` - alarm sensing and status publishing node
- `relay-node/` - BLE Mesh relay node
- `master-node/` - central monitoring node

## Features

- BLE Mesh based communication
- Vendor model messaging
- Alarm status monitoring
- LED status indication
- Long-press local reset
- Multi-node architecture