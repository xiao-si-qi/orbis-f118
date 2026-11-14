idf_component_register(SRCS "esp_lcd_st77916.c" "esp_lcd_st77916_spi.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_lcd driver esp_driver_spi esp_driver_gpio)
