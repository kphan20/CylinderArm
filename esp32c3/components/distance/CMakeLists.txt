idf_component_register(
    SRCS "distance.c"
    PRIV_REQUIRES 
        "driver"
        "sensor"
        "pins"
    INCLUDE_DIRS ".")