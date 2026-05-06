set(srcs
    "main.c"
    "master_node_hal.c"
    "master_node_mesh.c"
    "ble_mesh_example_init.c"
)

idf_component_register(
    SRCS            "${srcs}"
    INCLUDE_DIRS    "."
    REQUIRES bt nvs_flash driver
)
