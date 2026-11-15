# lv_mem_psram.c 提供 liblvgl 需要的 lv_mem_init / lv_malloc_core / ... （自定义分配器）。
# 静态库按依赖顺序链接，liblvgl 的 lv_init.c.obj 是在 main 之后才被拉进来的，
# 普通链接会漏掉这个 .o；WHOLE_ARCHIVE 强制包含 main 的全部目标文件
# （ESP-IDF 给 esp_phy 等组件也是这么做的）。
idf_component_register(SRCS "main.c" "net.c" "web.c" "ha_client.c" "ha_ws.c" "app_settings.c" "ui_home.c" "assistant_ui.c" "log_buffer.c" "bsp_shim.c" "ota.c" "bg_image.c" "bg_cache.c" "layout.c" "lv_font_cjk16.c"
                                "voice_audio.c" "xiaozhi_client.c" "lv_mem_psram.c" "bt_link.c"
                       INCLUDE_DIRS "."
                       REQUIRES lvgl st77916 esp_lcd esp_driver_spi esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_i2s esp_timer heap esp_mm
                                mqtt esp_wifi esp_netif esp_event nvs_flash cjson esp_http_client esp_http_server esp_adc
                                esp_driver_jpeg esp_jpeg wear_levelling fatfs vfs app_update bootloader_support esp_https_ota
                                esp_websocket_client esp_audio_codec mbedtls
                                bt
                       WHOLE_ARCHIVE)
