idf_component_register(
    SRCS
        "main.c"
        "relay.c"
        "pc_sense.c"
        "ota.c"
        "led.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        driver
        nvs_flash
        app_update
        espressif__esp-zigbee-lib
        espressif__esp-zboss-lib
)
