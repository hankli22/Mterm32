#ifndef DRV_DISPLAY_H
#define DRV_DISPLAY_H

#include "clib/u8g2.h"

class Display {
public:
  static void init();
  static u8g2_t* get();
  static void setContrast(uint8_t contrast);
  static void setPowerSave(uint8_t enable);

private:
  static u8g2_t u8g2;
};

#endif
