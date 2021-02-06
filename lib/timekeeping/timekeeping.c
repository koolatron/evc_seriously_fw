#include "timekeeping.h"

void srtc_update(stime_t* time) {
    if ( time->milliseconds >= 1000 ) {
        time->milliseconds %= 1000;
        time->seconds++;
    }

    if ( time->seconds >= 60 ) {
        time->seconds %= 60;
        time->minutes++;
    }

    if ( time->minutes >= 60 ) {
        time->minutes %= 60;
        time->hours++;
    }

    if ( time->hours >= 24 ) {
        time->hours %= 24;
        time->days++;
    }

    // ignore month, year for now
}