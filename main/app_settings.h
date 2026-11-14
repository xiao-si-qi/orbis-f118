#pragma once

#include <stdbool.h>
#include "esp_err.h"

#define APP_HA_URL_MAX   128
#define APP_HA_TOKEN_MAX 256

typedef struct {
    char ha_url[APP_HA_URL_MAX];
    char ha_token[APP_HA_TOKEN_MAX];
} app_settings_t;

const app_settings_t *app_settings_get(void);
bool app_settings_has_ha(void);
