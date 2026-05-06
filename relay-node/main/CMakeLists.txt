set(srcs
    "main.c"
    "relay_node_hal.c"
    "relay_node_mesh.c"
    "ble_mesh_example_init.c"
)

idf_component_register(
    SRCS            "${srcs}"
    INCLUDE_DIRS    "."
    REQUIRES bt nvs_flash driver
)
