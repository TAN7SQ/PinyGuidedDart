# 组件要求
set(
    REQUIRES 
    console
    driver
    vfs
    sdmmc
    fatfs
    nvs_flash
)

# 源文件
set(SOURCES
            "main.cpp"
            "Sensors/drivers/spi_bus.cpp"
            "Sensors/device_cpp/bmi088.cpp"
            "Sensors/device_cpp/ms5611.cpp"
            "Sensors/drivers/i2c_bus.cpp"
            "Device/WS2812/ws2812.cpp"
            "Device/Beeper/beeper.cpp"
            "App/application.cpp"
            "Device/TFCard/tfcard.cpp"

)



# 包含目录
set(
    INCLUDE_DIRS
    "Sensors/drivers"
    "Sensors/device_cpp"
    "mLog"
    "Device/WS2812"
    "Device/Beeper"
    "App/include"
    "Device/TFCard"
)

idf_component_register(SRCS ${SOURCES}
                    PRIV_REQUIRES ${REQUIRES}
                    INCLUDE_DIRS ${INCLUDE_DIRS})
