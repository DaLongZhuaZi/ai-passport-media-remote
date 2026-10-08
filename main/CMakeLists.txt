idf_component_register(
    SRCS "main.c"
         "hid_remote.c"
    INCLUDE_DIRS "."
    REQUIRES bsp bt nvs_flash esp_timer
)
