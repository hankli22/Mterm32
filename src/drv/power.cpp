#include "power.h"
#include "display.h"
#include "board/board.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_sleep.h>

void Power::init() {
  pinMode(GPS_PWR_MOSFET, OUTPUT);
  digitalWrite(GPS_PWR_MOSFET, LOW);

  pinMode(OLED_PWR, OUTPUT);
  pinMode(OLED_RST, OUTPUT);

  digitalWrite(OLED_PWR, LOW);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_PWR, HIGH);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  delay(50);
}

int Power::getBatteryPercent() {
  int analogVolts = analogReadMilliVolts(BAT_ADC);
  int batMv = analogVolts * 2;
  int pct = (batMv - 3300) / 9;
  if (pct > 100) pct = 100;
  if (pct < 0)  pct = 0;
  return pct;
}

void Power::sleepDevice() {
  auto u8g2 = Display::get();
  u8g2->clearBuffer();
  u8g2->setFont(u8g2_font_6x12_tr);
  u8g2->enableUTF8Print();
  u8g2->drawStr(15, 30, "Release Button");
  u8g2->drawStr(15, 45, "to Power Off...");
  u8g2->sendBuffer();

  while (digitalRead(BTN_UP)   == LOW ||
         digitalRead(BTN_DOWN) == LOW ||
         digitalRead(BTN_LEFT)  == LOW ||
         digitalRead(BTN_RIGHT) == LOW) {
    delay(50);
  }
  delay(300);

  u8g2->clearBuffer();
  u8g2->sendBuffer();

  digitalWrite(OLED_PWR, LOW);
  digitalWrite(OLED_RST, LOW);
  digitalWrite(GPS_PWR_MOSFET, HIGH);

  const uint8_t btns[] = { BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT };
  uint64_t mask = 0;
  for (int i = 0; i < 4; i++) {
    gpio_sleep_set_direction((gpio_num_t)btns[i], GPIO_MODE_INPUT);
    gpio_sleep_set_pull_mode((gpio_num_t)btns[i], GPIO_PULLUP_ONLY);
    mask |= 1ULL << btns[i];
  }
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_deep_sleep_enable_gpio_wakeup(mask, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}
