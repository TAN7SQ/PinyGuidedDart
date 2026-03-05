
# 组件依赖要求
set(REQUIRES 
    console
    driver
    vfs
    
    sdmmc
    fatfs
    nvs_flash

    esp_driver_ledc

    esp_event
    esp_wifi
    lwip
    esp_netif
)

# ========== 按模块分类 ==========
# Sensor模块源文件
set(SENSOR_SOURCES
    "Sensors/drivers/spi_bus.cpp"
    "Sensors/device_cpp/bmi088/bmi088.cpp"
    "Sensors/device_cpp/ms5611/ms5611.cpp"
    "Sensors/drivers/i2c_bus.cpp"
    "Sensors/drivers/uart.cpp"
)

# Device模块源文件
set(DEVICE_SOURCES
    "Device/WS2812/ws2812.cpp"
    "Device/Beeper/beeper.cpp"
    "Device/TFCard/tfcard.cpp"
    "Device/Servo/servo.cpp"
)

# Protocol模块源文件
set(PROTOCOL_SOURCES
    "Protocol/wifi_udp_client.cpp"
)

# App模块源文件
set(APP_SOURCES
    "App/application.cpp"
    "App/control.cpp"
    "App/hostpc.cpp"

)

# Algorithm模块源文件
set(ALGORITHM_SOURCES
    "Algorithm/kalman6asix.cpp"
    "Algorithm/AuxiliaryMath.cpp"
    # "Algorithm/complementary6asix.cpp"
    "Algorithm/Calibrate/calibrate.cpp"
    "Algorithm/kalmanHeightVelocity.hpp"
    "Algorithm/kalman1D.hpp"
)



# 合并所有源文件
set(SOURCES
    "main.cpp"
    ${SENSOR_SOURCES}
    ${DEVICE_SOURCES}
    ${APP_SOURCES}
    ${ALGORITHM_SOURCES}
    ${PROTOCOL_SOURCES}
)

# ========== 按模块分类 ==========
# Sensor模块包含目录
set(SENSOR_INCLUDE_DIRS
    "Sensors/drivers"
    "Sensors/device_cpp/bmi088"
    "Sensors/device_cpp/ms5611"
)

# Device模块包含目录
set(DEVICE_INCLUDE_DIRS
    "Device/WS2812"
    "Device/Beeper"
    "Device/TFCard"
    "Device/Servo"
)

# App模块包含目录
set(APP_INCLUDE_DIRS
    "App/include"
    "mLog"  # mLog归类到App模块，若需独立可单独定义
    "Protocol"
    "Algorithm"
    "Algorithm/Calibrate"
)

# 合并所有包含目录
set(INCLUDE_DIRS
    ${SENSOR_INCLUDE_DIRS}
    ${DEVICE_INCLUDE_DIRS}
    ${APP_INCLUDE_DIRS}
)

# 注册ESP-IDF组件
idf_component_register(
    SRCS ${SOURCES}
    PRIV_REQUIRES ${REQUIRES}
    INCLUDE_DIRS ${INCLUDE_DIRS}
)