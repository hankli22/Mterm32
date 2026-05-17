#ifndef BOARD_H
#define BOARD_H

// ============================================================
// Board pin definitions — change only this file to port
// ============================================================

// Buttons (active low, INPUT_PULLUP)
#define BTN_UP     4
#define BTN_DOWN   5
#define BTN_LEFT   6
#define BTN_RIGHT  7

// Power control
#define OLED_PWR        19
#define GPS_PWR_MOSFET  20  // A1SHB Gate
#define OLED_RST        15

// ADC
#define BAT_ADC         0

// GPS UART (Serial1)
#define RX_GPS          17
#define TX_GPS          18

// Button event enum — used by drv/buttons.h and app
enum BtnEvent {
  BTN_NONE,
  BTN_UP_PRESSED,
  BTN_DOWN_PRESSED,
  BTN_LEFT_PRESSED,
  BTN_RIGHT_PRESSED
};

#endif
