idf_component_register(SRCS "main.c" "net.c" "ha_client.c" "app_settings.c" "ui_home.c" "log_buffer.c" "bsp_shim.c" "ota.c" "bg_image.c" "bg_cache.c" "layout.c" "lv_font_cjk16.c"
                                "voice_audio.c" "xiaozhi_client.c"
                       INCLUDE_DIRS "."
                       REQUIRES lvgl st77916 esp_lcd esp_driver_spi esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_i2s esp_timer heap
                                mqtt esp_wifi esp_netif esp_event nvs_flash cjson esp_http_client esp_adc
                                esp_jpeg wear_levelling fatfs vfs app_update bootloader_support esp_https_ota
                                esp_websocket_client esp_audio_codec)
