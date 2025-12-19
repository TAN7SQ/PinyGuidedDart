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
    "Sensors/BMI088/bmi088.c"
    "Sensors/MS5611/ms5611.c"
)



# 包含目录
set(
    INCLUDE_DIRS
    "Sensors/BMI088"
    "Sensors/MS5611"
)

idf_component_register(SRCS ${SOURCES}
                    PRIV_REQUIRES ${my_requires}
                    INCLUDE_DIRS ${INCLUDE_DIRS})
