#ifndef DRV_BUTTONS_H
#define DRV_BUTTONS_H

#include "board/board.h"

class Buttons {
public:
  static void init();
  static void update();
  static BtnEvent getEvent();

private:
  static BtnEvent lastEvent;
  static bool lastState[4];
  static uint32_t nextRepeat[2];
};

#endif
