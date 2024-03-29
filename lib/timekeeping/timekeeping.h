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
    uint8_t bytes[6];
} stime_t;

void srtc_update(volatile stime_t* time);
void srtc_update_from_bytes(volatile stime_t* time);

#endif