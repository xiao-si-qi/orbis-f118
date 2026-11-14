#include <string.h>
#include <stdio.h>
#include "app_settings.h"
#include "secrets.h"

static app_settings_t s_cfg;
static bool s_ready;

const app_settings_t *app_settings_get(void)
{
    if (!s_ready) {
        snprintf(s_cfg.ha_url, sizeof(s_cfg.ha_url), "%s", APP_SECRET_HA_URL);
        snprintf(s_cfg.ha_token, sizeof(s_cfg.ha_token), "%s", APP_SECRET_HA_TOKEN);
        s_ready = true;
    }
    return &s_cfg;
}

bool app_settings_has_ha(void)
{
    return APP_SECRET_HA_URL[0] != '\0' && APP_SECRET_HA_TOKEN[0] != '\0';
}
