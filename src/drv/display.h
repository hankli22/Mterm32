#ifndef DRV_DISPLAY_H
#define DRV_DISPLAY_H

#include <U8g2lib.h>

class Display {
public:
  static void init();
  static U8G2* get();
  static void setContrast(uint8_t contrast);
  static void setPowerSave(uint8_t enable);

private:
  static U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2;
};

#endif
