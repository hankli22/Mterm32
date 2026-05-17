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
#include "esp_rom_sys.h"

static UsbCdc pcSerial;

TaskHandle_t uiTaskHandle;
TaskHandle_t logicTaskHandle;

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// TEST: Diagnostic macros to trace init sequence via GPIO & ROM printf.
// GPIO 19 = OLED_PWR (indicator LED), GPIO 20 = GPS_PWR_MOSFET (2nd LED)
// Watch the LEDs / serial monitor on USB-JTAG (COM24, 115200 8N1).
// REMOVE this entire block after pinpointing the crash location.
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define TRACE_INIT(step) do {                              \
    esp_rom_printf("[INIT] " #step "\r\n");                \
    gpio_set_level(GPIO_NUM_20, ((step) & 1) ? 1 : 0);    \
    for (volatile int _i = 0; _i < 300000; _i++) {}        \
} while(0)

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
    // --- TEST: early GPIO setup so LEDs work immediately ---
    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_20, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_19, 1);
    gpio_set_level(GPIO_NUM_20, 0);
    TRACE_INIT(1);  // app_main entered

    nvs_flash_init();
    TRACE_INIT(2);  // nvs done

    pcSerial.begin(115200);
    TRACE_INIT(3);  // usb-serial-jtag ready

    loadConfig();
    TRACE_INIT(4);  // config loaded

    Power::init();
    TRACE_INIT(5);  // power init done

    Display::init();
    TRACE_INIT(6);  // display init done

    Buttons::init();
    TRACE_INIT(7);  // buttons init done

    GPSCalc::init();
    TRACE_INIT(8);  // gps init done

    xTaskCreatePinnedToCore(uiTask, "UI", 8192, NULL, 1, &uiTaskHandle, 0);
    xTaskCreatePinnedToCore(logicTask, "Logic", 4096, NULL, 2, &logicTaskHandle, 0);
    TRACE_INIT(9);  // tasks created, entering main loop

    // app_main must not return
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}
