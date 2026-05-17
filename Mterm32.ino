#include <Arduino.h>
#include "drv/display.h"
#include "drv/buttons.h"
#include "drv/power.h"
#include "svc/gps.h"
#include "app/menu.h"
#include "svc/config.h"
#pragma GCC optimize("O2")


TaskHandle_t uiTaskHandle;
TaskHandle_t logicTaskHandle;

void uiTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    Buttons::update();
    MenuManager::handleInput();
    MenuManager::update();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(16));
  }
}

void logicTask(void* pvParameters) {
  for (;;) {
    GPSCalc::process();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(1024);
  loadConfig();
  Power::init();
  Display::init();
  Buttons::init();
  GPSCalc::init();

  xTaskCreatePinnedToCore(uiTask, "UI", 8192, NULL, 1, &uiTaskHandle, 0);
  xTaskCreatePinnedToCore(logicTask, "Logic", 4096, NULL, 2, &logicTaskHandle, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}