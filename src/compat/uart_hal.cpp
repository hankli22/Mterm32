#include "uart_hal.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include <string.h>

// ---- UartHal (hardware UART, GPS on Serial1) ----

void UartHal::begin(unsigned long baud, uint32_t cfg, int8_t rxPin, int8_t txPin) {
    if (installed_) end();

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = (int)baud;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity   = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    uartNum_ = 1;
    uart_param_config((uart_port_t)uartNum_, &uart_cfg);
    uart_set_pin((uart_port_t)uartNum_, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install((uart_port_t)uartNum_, 1024, 256, 0, nullptr, 0);
    installed_ = true;
}

void UartHal::end() {
    if (installed_) {
        uart_driver_delete((uart_port_t)uartNum_);
        installed_ = false;
    }
}

int UartHal::available() {
    if (!installed_) return 0;
    size_t len = 0;
    uart_get_buffered_data_len((uart_port_t)uartNum_, &len);
    return (int)len;
}

int UartHal::read() {
    if (!installed_) return -1;
    uint8_t byte;
    int n = uart_read_bytes((uart_port_t)uartNum_, &byte, 1, 0);
    return (n > 0) ? (int)byte : -1;
}

size_t UartHal::write(uint8_t byte) {
    if (!installed_) return 0;
    return uart_write_bytes((uart_port_t)uartNum_, &byte, 1);
}

size_t UartHal::println(const char* str) {
    if (!installed_) return 0;
    size_t n = uart_write_bytes((uart_port_t)uartNum_, str, strlen(str));
    n += uart_write_bytes((uart_port_t)uartNum_, "\r\n", 2);
    return n;
}

int UartHal::availableForWrite() {
    if (!installed_) return 0;
    size_t len = 0;
    uart_get_tx_buffer_free_size((uart_port_t)uartNum_, &len);
    return (int)len;
}

void UartHal::setRxBufferSize(size_t size) {
    // buffer size is set at begin() time; re-install if needed
    if (installed_) {
        uart_driver_delete((uart_port_t)uartNum_);
        uart_driver_install((uart_port_t)uartNum_, size, 256, 0, nullptr, 0);
    }
}

void UartHal::flush() {
    if (installed_) uart_wait_tx_done((uart_port_t)uartNum_, pdMS_TO_TICKS(100));
}

// ---- UsbCdc (USB Serial/JTAG → PC) ----

void UsbCdc::begin(unsigned long /*baud*/) {
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&cfg);
}

int UsbCdc::available() {
    if (hasPeek_) return 1;
    int n = usb_serial_jtag_read_bytes(&peekChar_, 1, 0);
    if (n > 0) { hasPeek_ = true; return 1; }
    return 0;
}

int UsbCdc::read() {
    if (hasPeek_) { hasPeek_ = false; return (int)peekChar_; }
    uint8_t b;
    int n = usb_serial_jtag_read_bytes(&b, 1, 0);
    return (n > 0) ? (int)b : -1;
}

size_t UsbCdc::write(uint8_t byte) {
    return usb_serial_jtag_write_bytes(&byte, 1, 0);
}

void UsbCdc::setRxBufferSize(size_t /*size*/) {}
