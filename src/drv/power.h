#ifndef DRV_POWER_H
#define DRV_POWER_H

class Power {
public:
  static void init();              // set up power & reset pins
  static int  getBatteryPercent(); // returns 0–100
  static void sleepDevice();       // deep sleep with button wake
};

#endif
