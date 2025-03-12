/* SRAD Avionics Flight Software for AIAA-UH
 *
 * Copyright (c) 2024 Nathan Samuell + Dedah + Thanh! (www.github.com/nathansamuell, www.github.com/UH-AIAA)
 *
 * More information on the MIT license as well as a complete copy
 * of the license can be found here: https://choosealicense.com/licenses/mit/
 *
 * All above text must be included in any redistribution.
 */

#include "SRAD_PHX.h"


void FLIGHT::calculateState() {
    switch(STATE) {
        case(STATES::PRE_NO_CAL):
            if(calibrate()) {
                STATE = STATES::PRE_CAL;
            }
            break;

        case(STATES::PRE_CAL):
            if(isAscent()) {
                STATE = STATES::FLIGHT_ASCENT;
            }
            break;

        case STATES::FLIGHT_ASCENT:
            if(isDescent()) {
                STATE = STATES::FLIGHT_DESCENT;
            }
            break;

        case STATES::FLIGHT_DESCENT:
            if(isLanded()) {
                STATE = STATES::POST_LANDED;
            }
            break;
    }
}

bool FLIGHT::isCal() {
    return calibrated;
}
bool FLIGHT::isAscent() {
    static uint32_t liftoff_timer;
    
    if(output.lsm_acc.z > accel_liftoff_threshold) {
        // increment liftoff timer with delta time when completed

        if(liftoff_timer > accel_liftoff_time_threshold) {
            return true;
        }
    } else {
        liftoff_timer = 0;
    }
    return false;
}
bool FLIGHT::isDescent() {

}
bool FLIGHT::isLanded() {

}

bool FLIGHT::calibrate() {
    
}