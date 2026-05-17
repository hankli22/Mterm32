#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
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
    // NB: "width" = minimum field width (Arduino semantics), NOT buffer size.
    // Format into temp buffer then copy — all callers allocate >= 16 bytes.
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%*.*f", width, prec, val);
    size_t n = strlen(tmp);
    if (n > (size_t)(width + prec + 7)) n = (size_t)(width + prec + 7);
    memcpy(buf, tmp, n);
    buf[n] = '\0';
    return buf;
}
