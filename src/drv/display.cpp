#include "display.h"
#include "board/board.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static spi_device_handle_t spiHandle;
u8g2_t Display::u8g2;

static uint8_t u8g2_spi_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_SEND: {
            spi_transaction_t t = {};
            t.length = arg_int * 8;
            t.tx_buffer = arg_ptr;
            spi_device_transmit(spiHandle, &t);
            break;
        }
        case U8X8_MSG_BYTE_INIT:
            break;
        case U8X8_MSG_BYTE_SET_DC:
            gpio_set_level(GPIO_NUM_21, arg_int);
            break;
    }
    return 0;
}

static uint8_t u8g2_gpio_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT: {
            gpio_config_t cfg = {};
            cfg.pin_bit_mask = (1ULL << 21);
            cfg.mode = GPIO_MODE_OUTPUT;
            cfg.pull_up_en = GPIO_PULLUP_DISABLE;
            cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            cfg.intr_type = GPIO_INTR_DISABLE;
            gpio_config(&cfg);
            break;
        }
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
        case U8X8_MSG_GPIO_DC:
            gpio_set_level(GPIO_NUM_21, arg_int);
            break;
        case U8X8_MSG_GPIO_RESET:
            // OLED_RST = 15, but we handle that in Power::init()
            break;
    }
    return 0;
}

void Display::init() {
    spi_bus_config_t busCfg = {};
    busCfg.mosi_io_num = 22;
    busCfg.miso_io_num = -1;
    busCfg.sclk_io_num = 23;
    busCfg.quadwp_io_num = -1;
    busCfg.quadhd_io_num = -1;
    spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devCfg = {};
    devCfg.clock_speed_hz = 40000000;
    devCfg.mode = 0;
    devCfg.spics_io_num = -1;
    devCfg.queue_size = 1;
    spi_bus_add_device(SPI2_HOST, &devCfg, &spiHandle);

    u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8g2_spi_cb, u8g2_gpio_cb);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
}

u8g2_t* Display::get()   { return &u8g2; }
void    Display::setContrast(uint8_t c)  { u8g2_SetContrast(&u8g2, c); }
void    Display::setPowerSave(uint8_t en) { u8g2_SetPowerSave(&u8g2, en); }
