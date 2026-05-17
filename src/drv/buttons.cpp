#include "buttons.h"
#include "compat/compat.h"
#include "driver/gpio.h"
#include "esp_timer.h"

BtnEvent Buttons::lastEvent = BTN_NONE;
bool     Buttons::lastState[4] = { true, true, true, true };
uint32_t Buttons::nextRepeat[2] = { 0, 0 };

void Buttons::init() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << BTN_UP) | (1ULL << BTN_DOWN) |
                       (1ULL << BTN_LEFT) | (1ULL << BTN_RIGHT);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&cfg);
}

void Buttons::update() {
    bool current[4] = {
        (gpio_get_level(GPIO_NUM_4) == 1),   // BTN_UP
        (gpio_get_level(GPIO_NUM_5) == 1),   // BTN_DOWN
        (gpio_get_level(GPIO_NUM_6) == 1),   // BTN_LEFT
        (gpio_get_level(GPIO_NUM_7) == 1)    // BTN_RIGHT
    };
    lastEvent = BTN_NONE;
    uint32_t now = millis();

    for (int i = 0; i < 4; i++) {
        if (lastState[i] == true && current[i] == false) {
            if (i == 0) { lastEvent = BTN_UP_PRESSED;    nextRepeat[0] = now + 400; }
            if (i == 1) { lastEvent = BTN_DOWN_PRESSED;  nextRepeat[1] = now + 400; }
            if (i == 2) { lastEvent = BTN_LEFT_PRESSED;  }
            if (i == 3) { lastEvent = BTN_RIGHT_PRESSED; }
        } else if (lastState[i] == false && current[i] == false) {
            if (i == 0 && now >= nextRepeat[0]) { lastEvent = BTN_UP_PRESSED;   nextRepeat[0] = now + 80; }
            if (i == 1 && now >= nextRepeat[1]) { lastEvent = BTN_DOWN_PRESSED; nextRepeat[1] = now + 80; }
        }
        lastState[i] = current[i];
    }
}

BtnEvent Buttons::getEvent() {
    BtnEvent evt = lastEvent;
    lastEvent = BTN_NONE;
    return evt;
}
