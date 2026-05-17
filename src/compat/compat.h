#pragma once

#include <stdint.h>
#include <stdio.h>
#include <algorithm>
#include "esp_timer.h"

// ---- Arduino compatibility macros ----

#define HIGH 1
#define LOW  0
#define PI  3.14159265358979323846
#define TWO_PI (PI * 2.0)

using std::max;
using std::min;

inline uint32_t millis() {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

#ifndef constrain
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#define radians(deg) ((deg) * (PI / 180.0))
#define degrees(rad) ((rad) * (180.0 / PI))
#define sq(x)        ((x) * (x))

inline char* dtostrf(double val, int width, int prec, char* buf) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
    snprintf(buf, (size_t)(width + 2), "%.*f", prec, val);
#pragma GCC diagnostic pop
    return buf;
}
