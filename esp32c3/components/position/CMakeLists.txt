idf_component_register(
    SRCS "position.c"
    PRIV_REQUIRES 
        "esp_driver_i2c"
        "sensor"
        "pins"
    INCLUDE_DIRS ".")