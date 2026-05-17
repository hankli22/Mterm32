#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "drv/display.h"
#include "drv/buttons.h"
#include "drv/power.h"
#include "svc/gps.h"
#include "svc/config.h"
#include "app/menu.h"
#include "compat/uart_hal.h"
#include "driver/gpio.h"

static UsbCdc pcSerial;

TaskHandle_t uiTaskHandle;
TaskHandle_t logicTaskHandle;

static void uiTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        Buttons::update();
        MenuManager::handleInput();
        MenuManager::update();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(16));
    }
}

static void logicTask(void* pvParameters) {
    for (;;) {
        GPSCalc::process();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main() {
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // TEST: Toggle OLED_PWR (GPIO 19) to verify app_main() runs.
    // REMOVE after confirming program reaches this point.
    // If OLED powers on immediately after reset, app_main IS entered.
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_19, 1);

    nvs_flash_init();
    pcSerial.begin(115200);

    loadConfig();
    Power::init();
    Display::init();
    Buttons::init();
    GPSCalc::init();

    xTaskCreatePinnedToCore(uiTask, "UI", 8192, NULL, 1, &uiTaskHandle, 0);
    xTaskCreatePinnedToCore(logicTask, "Logic", 4096, NULL, 2, &logicTaskHandle, 0);

    // app_main must not return
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}
