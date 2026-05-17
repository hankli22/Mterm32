#include "buttons.h"
#include <Arduino.h>
#include <driver/gpio.h>

BtnEvent Buttons::lastEvent = BTN_NONE;
bool     Buttons::lastState[4] = { true, true, true, true };
uint32_t Buttons::nextRepeat[2] = { 0, 0 };

void Buttons::init() {
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);
  pinMode(BTN_LEFT,  INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
}

void Buttons::update() {
  bool current[4] = {
    (digitalRead(BTN_UP)    == HIGH),
    (digitalRead(BTN_DOWN)  == HIGH),
    (digitalRead(BTN_LEFT)  == HIGH),
    (digitalRead(BTN_RIGHT) == HIGH)
  };
  lastEvent = BTN_NONE;
  uint32_t now = millis();

  for (int i = 0; i < 4; i++) {
    if (lastState[i] == true && current[i] == false) {
      if (i == 0) { lastEvent = BTN_UP_PRESSED;    nextRepeat[0] = now + 400; }
      if (i == 1) { lastEvent = BTN_DOWN_PRESSED;  nextRepeat[1] = now + 400; }
      if (i == 2) { lastEvent = BTN_LEFT_PRESSED;  }
      if (i == 3) { lastEvent = BTN_RIGHT_PRESSED; }
    } else if (lastState[i] == false && current[i] == false) {
      if (i == 0 && now >= nextRepeat[0]) { lastEvent = BTN_UP_PRESSED;   nextRepeat[0] = now + 80; }
      if (i == 1 && now >= nextRepeat[1]) { lastEvent = BTN_DOWN_PRESSED; nextRepeat[1] = now + 80; }
    }
    lastState[i] = current[i];
  }
}

BtnEvent Buttons::getEvent() {
  BtnEvent evt = lastEvent;
  lastEvent = BTN_NONE;
  return evt;
}
