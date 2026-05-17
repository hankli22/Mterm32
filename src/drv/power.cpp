#include "power.h"
#include "display.h"
#include "board/board.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static adc_oneshot_unit_handle_t adcHandle;

void Power::init() {
    gpio_config_t outCfg = {};
    outCfg.mode = GPIO_MODE_OUTPUT;
    outCfg.pin_bit_mask = (1ULL << GPS_PWR_MOSFET) | (1ULL << OLED_PWR) | (1ULL << OLED_RST);
    gpio_config(&outCfg);

    gpio_set_level(GPIO_NUM_20, 1);  // GPS_PWR_MOSFET high (GPS on)
    gpio_set_level(GPIO_NUM_19, 0);  // OLED_PWR low
    gpio_set_level(GPIO_NUM_15, 0);  // OLED_RST low
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(GPIO_NUM_19, 1);  // OLED_PWR high
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(GPIO_NUM_15, 1);  // OLED_RST high
    vTaskDelay(pdMS_TO_TICKS(50));

    // ADC for battery
    adc_oneshot_unit_init_cfg_t adcInit = {};
    adcInit.unit_id = ADC_UNIT_1;
    adc_oneshot_new_unit(&adcInit, &adcHandle);
    adc_oneshot_chan_cfg_t chCfg = {};
    chCfg.atten = ADC_ATTEN_DB_12;
    chCfg.bitwidth = ADC_BITWIDTH_12;
    adc_oneshot_config_channel(adcHandle, ADC_CHANNEL_0, &chCfg);
}

int Power::getBatteryPercent() {
    int raw = 0;
    adc_oneshot_read(adcHandle, ADC_CHANNEL_0, &raw);
    int analogVolts = raw * 3300 / 4095;
    int batMv = analogVolts * 2;  // 2:1 voltage divider
    int pct = (batMv - 3300) / 9;
    if (pct > 100) pct = 100;
    if (pct < 0)   pct = 0;
    return pct;
}

void Power::sleepDevice() {
    auto* u = Display::get();
    u8g2_ClearBuffer(u);
    u8g2_SetFont(u, u8g2_font_6x12_tr);
    u8g2_DrawStr(u, 15, 30, "Release Button");
    u8g2_DrawStr(u, 15, 45, "to Power Off...");
    u8g2_SendBuffer(u);

    const int btns[] = { BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT };
    for (;;) {
        bool any = false;
        for (int i = 0; i < 4; i++)
            if (gpio_get_level((gpio_num_t)btns[i]) == 0) any = true;
        if (!any) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(300));

    u8g2_ClearBuffer(u);
    u8g2_SendBuffer(u);

    gpio_set_level(GPIO_NUM_19, 0);  // OLED_PWR off
    gpio_set_level(GPIO_NUM_15, 0);  // OLED_RST low
    gpio_set_level(GPIO_NUM_20, 0);  // GPS_PWR_MOSFET low (cut GPS power in sleep)

    const uint8_t wakeBtns[] = { BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT };
    uint64_t mask = 0;
    for (int i = 0; i < 4; i++) {
        gpio_sleep_set_direction((gpio_num_t)wakeBtns[i], GPIO_MODE_INPUT);
        gpio_sleep_set_pull_mode((gpio_num_t)wakeBtns[i], GPIO_PULLUP_ONLY);
        mask |= 1ULL << wakeBtns[i];
    }
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_deep_sleep_enable_gpio_wakeup(mask, ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
}
