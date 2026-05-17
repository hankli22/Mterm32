#pragma once

#include <stdint.h>
#include <stddef.h>

// ESP-IDF UART config: 8N1 etc.
#define UART_CFG_8N1  (0x800001c)   // matches SERIAL_8N1

class UartHal {
public:
    // hardware UART (GPS on pins RX_GPS/TX_GPS)
    void begin(unsigned long baud, uint32_t cfg, int8_t rxPin, int8_t txPin);
    void end();
    int  available();
    int  read();
    size_t write(uint8_t byte);
    size_t println(const char* str);
    int  availableForWrite();
    void setRxBufferSize(size_t size);
    void flush();

private:
    int uartNum_;
    bool installed_;
};

// USB CDC — PC communication via USB Serial/JTAG
class UsbCdc {
public:
    void begin(unsigned long baud);
    int  available();
    int  read();
    size_t write(uint8_t byte);
    void setRxBufferSize(size_t size);
};
