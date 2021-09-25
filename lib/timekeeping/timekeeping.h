#ifndef _TIMEKEEPING_H
#define _TIMEKEEPING_H

#include <stdint.h>

typedef struct stime_t {
    uint16_t milliseconds;
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t days;
    uint8_t months;
    uint8_t years;
} stime_t;

void srtc_update(volatile stime_t* time);

#endif