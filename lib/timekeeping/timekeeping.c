#include "timekeeping.h"

void srtc_update(volatile stime_t* time) {
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
    }

    time->bytes[5] = time->seconds % 10;
    time->bytes[4] = (time->seconds / 10) % 10;
    time->bytes[3] = time->minutes % 10;
    time->bytes[2] = (time->minutes / 10) % 10;
    time->bytes[1] = time->hours % 10;
    time->bytes[0] = (time->hours / 10) % 10;

    // TODO: implement day, month, year
}