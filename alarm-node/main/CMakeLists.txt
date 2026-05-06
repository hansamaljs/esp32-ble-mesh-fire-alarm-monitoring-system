set(srcs
    "main.c"
    "alarm_node_hal.c"
    "alarm_node_mesh.c"
    "ble_mesh_example_init.c"
)

idf_component_register(
    SRCS            "${srcs}"
    INCLUDE_DIRS    "."
    REQUIRES bt nvs_flash driver
)
