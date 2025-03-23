if(CONFIG_IS_HUB)
    message(STATUS "USING HUB DEVICE IMPLEMENTATION")
    set(SRC_FILE "hub_main.c")
    set(CYL_ARM_REQUIRES spi_flash esp_driver_spi esp_driver_gpio esp_wifi)
    set(CYL_ARM_INCLUDES)
else()
    message(STATUS "USING DISTRIBUTED DEVICE IMPLEMENTATION")
    set(SRC_FILE "distributed_main.c")
    set(CYL_ARM_REQUIRES)
    set(CYL_ARM_INCLUDES "../components/pins" "../components/distance" "../components/position")
endif()

idf_component_register(SRCS "${SRC_FILE}"
                    PRIV_REQUIRES "${CYL_ARM_REQUIRES}"
                    INCLUDE_DIRS "${CYL_ARM_INCLUDES}")