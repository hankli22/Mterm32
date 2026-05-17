#include "display.h"
#include "board/board.h"
#include <SPI.h>

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI Display::u8g2(
    U8G2_R0, U8X8_PIN_NONE, 21, U8X8_PIN_NONE);

void Display::init() {
  SPI.begin(23, -1, 22, -1);
  u8g2.begin();
  u8g2.setBusClock(40000000);
  u8g2.enableUTF8Print();
  analogReadResolution(12);
}

U8G2* Display::get()   { return &u8g2; }
void  Display::setContrast(uint8_t c)  { u8g2.setContrast(c); }
void  Display::setPowerSave(uint8_t en) { u8g2.setPowerSave(en); }
