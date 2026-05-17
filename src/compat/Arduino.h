// Stub Arduino.h for ESP-IDF — provides types TinyGPS++ expects
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <limits.h>

// TinyGPS++ uses PROGMEM for string storage on AVR. ESP32 has flat address
// space, so these are no-ops.
#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(addr)   (*(const uint8_t *)(addr))
#define pgm_read_word(addr)   (*(const uint16_t *)(addr))
#define strlen_P strlen
#define strcmp_P strcmp
#define strncmp_P strncmp
#define memcpy_P memcpy

// Type aliases Arduino provides but TinyGPS++ expects
using byte = uint8_t;
