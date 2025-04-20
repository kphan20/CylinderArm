idf_component_register(
    SRCS "distance.c"
    PRIV_REQUIRES 
        "driver"
        "sensor"
        "pins"
        "spline"
        "esp_timer"
    INCLUDE_DIRS ".")