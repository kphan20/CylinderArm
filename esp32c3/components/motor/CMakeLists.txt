idf_component_register(
    SRCS "motor.c"
    PRIV_REQUIRES 
        "esp_driver_ledc"
        "esp_driver_gpio"
        "pins"
        "pid"
    INCLUDE_DIRS ".")