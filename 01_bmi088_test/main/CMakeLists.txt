# 组件要求
set(
    my_requires
    esp_driver_i2c
    esp_driver_spi
    esp_driver_gpio
)

# 源文件
set(
    SOURCES
    "main.cpp"
    # "Sensors/device_c/bmi088.c"
    "Sensors/drivers/spi_bus.cpp"
    "Sensors/device_cpp/bmi088.cpp"
    "Sensors/device_cpp/ms5611.cpp"
    "Sensors/drivers/i2c_bus.cpp"
    #"Sensors/MS5611/ms5611.c"
)



# 包含目录
set(
    INCLUDE_DIRS
    # "Sensors/device_c"
    "Sensors/drivers"
    "Sensors/device_cpp"
    #"Sensors/MS5611"
)

idf_component_register(SRCS ${SOURCES}
                    PRIV_REQUIRES ${my_requires}
                    INCLUDE_DIRS ${INCLUDE_DIRS})
