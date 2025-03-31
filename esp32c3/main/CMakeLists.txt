if(CONFIG_IS_HUB)
    message(STATUS "USING HUB DEVICE IMPLEMENTATION")
    set(SRC_FILE "hub_main.c")
    set(CYL_ARM_REQUIRES spi_flash esp_driver_spi esp_driver_gpio esp_wifi)
    set(CYL_ARM_INCLUDES)
else()
    message(STATUS "USING DISTRIBUTED DEVICE IMPLEMENTATION")
    set(SRC_FILE "distributed_main.c")
    set(CYL_ARM_REQUIRES)
    set(CYL_ARM_INCLUDES 
            "../components/pins"   
            "../components/esp_now_setup"
            "../components/pid"
            "../components/motor")
    if(CONFIG_I2C_SENSOR)
        list(APPEND CYL_ARM_INCLUDES "../components/position" "../components/sensor")
    elseif(CONFIG_PWM_SENSOR)
        list(APPEND CYL_ARM_INCLUDES "../components/distance" "../components/sensor")
    endif()
endif()

idf_component_register(SRCS "${SRC_FILE}"
                    PRIV_REQUIRES "${CYL_ARM_REQUIRES}"
                    INCLUDE_DIRS "${CYL_ARM_INCLUDES}")