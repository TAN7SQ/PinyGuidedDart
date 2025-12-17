set(
    my_requires
    esp_driver_spi
    esp_driver_gpio
)

idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES "${my_requires}"
                    INCLUDE_DIRS "")
