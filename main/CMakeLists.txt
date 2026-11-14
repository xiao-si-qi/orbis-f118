idf_component_register(SRCS "main.c" "net.c" "ha_client.c" "app_settings.c" "lv_font_cjk16.c"
                       INCLUDE_DIRS "."
                       REQUIRES lvgl st77916 esp_lcd esp_driver_spi esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_timer heap
                                mqtt esp_wifi esp_netif esp_event nvs_flash cjson esp_http_client esp_adc)
