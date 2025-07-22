set(srcs "main.c")

set(priv_requires
        unity
        esp_driver_cam
        esp_psram
        sensor_init
        esp_lcd
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES ${priv_requires}
                       WHOLE_ARCHIVE TRUE)
