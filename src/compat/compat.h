#pragma once

#include <stdint.h>
#include "esp_timer.h"

// ---- Arduino compatibility macros ----

#define HIGH 1
#define LOW  0

inline uint32_t millis() {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

#ifndef constrain
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
