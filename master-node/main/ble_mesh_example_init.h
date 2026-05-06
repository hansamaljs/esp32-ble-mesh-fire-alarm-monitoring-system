#pragma once

#include <stdint.h>
#include "esp_err.h"

esp_err_t bluetooth_init(void);
void ble_mesh_get_dev_uuid(uint8_t *dev_uuid);
