#ifndef HARDWARELAYER_H
#define HARDWARELAYER_H

#include <U8g2lib.h>
#include "board/board.h"

class HAL {
public:
  static void init();
  static void updateButtons();
  static BtnEvent getEvent();
  static U8G2* getDisplay();
  static void sleepDevice();
  static int getBatteryPercent();
private:
  static BtnEvent lastEvent;
};

#endif
