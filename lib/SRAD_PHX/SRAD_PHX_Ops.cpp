/* SRAD Avionics Flight Software for AIAA-UH
 *
 * Copyright (c) 2025 Nathan Samuell + Dedah + Thanh! (www.github.com/nathansamuell, www.github.com/UH-AIAA)
 *
 * More information on the MIT license as well as a complete copy
 * of the license can be found here: https://choosealicense.com/licenses/mit/
 *
 * All above text must be included in any redistribution.
 */

#include "SRAD_PHX.h"

void FLIGHT::incrementTime() {
    static uint64_t newRunningTime_ms = millis();
    deltaTime_ms = newRunningTime_ms - runningTime_ms;
    output.totalTime_ms = newRunningTime_ms;
    runningTime_ms += deltaTime_ms;
}
void FLIGHT::writeSD(bool headers, File& outputFile) {
    if(headers) {
        // write headers
    }

}
void FLIGHT::writeXBEE(bool headers) {
    if(headers) {

    }
}
