#include "svc/config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

SystemConfig sysCfg;

void loadConfig() {
    nvs_handle_t h;
    if (nvs_open("runner", NVS_READONLY, &h) != ESP_OK) return;

    int32_t i32;
    uint8_t u8;
    float   f32;
    size_t  len = sizeof(f32);

    if (nvs_get_blob(h, "freq",  &f32, &len) == ESP_OK)   sysCfg.record_freq = f32;
    if (nvs_get_u8(h,   "track", &u8)  == ESP_OK)          sysCfg.draw_track   = (bool)u8;
    if (nvs_get_i32(h,  "soff",  &i32) == ESP_OK)          sysCfg.screen_off  = (int)i32;
    if (nvs_get_i32(h,  "stor",  &i32) == ESP_OK)          sysCfg.storage_track = (int)i32;
    if (nvs_get_u8(h,   "mcol",  &u8)  == ESP_OK)          sysCfg.en_multycol = (bool)u8;
    if (nvs_get_i32(h,  "cont",  &i32) == ESP_OK)          sysCfg.contrast    = (int)i32;
    if (nvs_get_u8(h,   "aslp",  &u8)  == ESP_OK)          sysCfg.auto_sleep  = (bool)u8;

    nvs_close(h);
}

void saveConfig() {
    nvs_handle_t h;
    if (nvs_open("runner", NVS_READWRITE, &h) != ESP_OK) return;

    nvs_set_blob(h, "freq",  &sysCfg.record_freq, sizeof(float));
    nvs_set_u8(h,   "track", (uint8_t)sysCfg.draw_track);
    nvs_set_i32(h,  "soff",  (int32_t)sysCfg.screen_off);
    nvs_set_i32(h,  "stor",  (int32_t)sysCfg.storage_track);
    nvs_set_u8(h,   "mcol",  (uint8_t)sysCfg.en_multycol);
    nvs_set_i32(h,  "cont",  (int32_t)sysCfg.contrast);
    nvs_set_u8(h,   "aslp",  (uint8_t)sysCfg.auto_sleep);
    nvs_commit(h);
    nvs_close(h);
}
