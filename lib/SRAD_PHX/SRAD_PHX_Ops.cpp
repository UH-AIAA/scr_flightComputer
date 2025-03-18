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
    runningTime_ms = newRunningTime_ms;
    output.totalTime_ms = newRunningTime_ms;
}

void FLIGHT::writeSD(bool headers, File& outputFile) {
    if(headers) {
        outputFile.println(data_header);
        outputFile.flush();
        return;
    }

    outputFile.print(output.totalTime_ms); outputFile.print(", ");
    if(last_gps.fix) {
        outputFile.print(last_gps.latitudeDegrees, 6); outputFile.print(", ");
        outputFile.print(last_gps.longitudeDegrees, 6); outputFile.print(",");
        outputFile.print((int32_t)last_gps.satellites); outputFile.print(",");
        outputFile.print(last_gps.speed, 3); outputFile.print(",");
        outputFile.print(last_gps.angle, 3); outputFile.print(",");
        outputFile.print(last_gps.altitude, 3); outputFile.print(",");
    } else {
        outputFile.print("-1,No fix,-1,No fix,0,-1,-1,-1,");
    }
    outputFile.print(output.bno_orientation.w, 5); outputFile.print(",");
    outputFile.print(output.bno_orientation.x, 5); outputFile.print(",");
    outputFile.print(output.bno_orientation.y, 5); outputFile.print(",");
    outputFile.print(output.bno_orientation.z, 5); outputFile.print(",");
    outputFile.print(output.bno_acc.x, 4); outputFile.print(",");
    outputFile.print(output.bno_acc.y, 4); outputFile.print(",");
    outputFile.print(output.bno_acc.z, 4); outputFile.print(",");
    outputFile.print(output.adxl_acc.x, 2); outputFile.print(",");
    outputFile.print(output.adxl_acc.y, 2); outputFile.print(",");
    outputFile.print(output.adxl_acc.z, 2); outputFile.print(",");
    outputFile.print(output.bmp_press, 6); outputFile.print(",");
    outputFile.print(output.bmp_alt, 4); outputFile.print(",");
    outputFile.print(output.lsm_temp, 2); outputFile.print(",");
    outputFile.print(output.adxl_temp, 2); outputFile.print(",");
    outputFile.print(output.bno_temp, 2); outputFile.print(",");
    outputFile.print(output.bmp_temp, 2); outputFile.println();
    outputFile.flush();

    return;
}

void FLIGHT::writeSERIAL(bool headers, HardwareSerial &Serial1) {
    if(headers) {
        Serial1.println(data_header);
        Serial1.flush();
        return;
    }

    Serial1.print(output.totalTime_ms); Serial1.print(", ");
    if(last_gps.fix) {
        Serial1.print(last_gps.latitudeDegrees, 6); Serial1.print(", ");
        Serial1.print(last_gps.longitudeDegrees, 6); Serial1.print(",");
        Serial1.print((int32_t)last_gps.satellites); Serial1.print(",");
        Serial1.print(last_gps.speed, 3); Serial1.print(",");
        Serial1.print(last_gps.angle, 3); Serial1.print(",");
        Serial1.print(last_gps.altitude, 3); Serial1.print(",");
    } else {
        Serial1.print("-1,No fix,-1,No fix,0,-1,-1,-1,");
    }
    Serial1.print(output.bno_orientation.w, 5); Serial1.print(",");
    Serial1.print(output.bno_orientation.x, 5); Serial1.print(",");
    Serial1.print(output.bno_orientation.y, 5); Serial1.print(",");
    Serial1.print(output.bno_orientation.z, 5); Serial1.print(",");
    Serial1.print(output.bno_acc.x, 4); Serial1.print(",");
    Serial1.print(output.bno_acc.y, 4); Serial1.print(",");
    Serial1.print(output.bno_acc.z, 4); Serial1.print(",");
    Serial1.print(output.adxl_acc.x, 2); Serial1.print(",");
    Serial1.print(output.adxl_acc.y, 2); Serial1.print(",");
    Serial1.print(output.adxl_acc.z, 2); Serial1.print(",");
    Serial1.print(output.bmp_press, 6); Serial1.print(",");
    Serial1.print(output.bmp_alt, 4); Serial1.print(",");
    Serial1.print(output.lsm_temp, 2); Serial1.print(",");
    Serial1.print(output.adxl_temp, 2); Serial1.print(",");
    Serial1.print(output.bno_temp, 2); Serial1.print(",");
    Serial1.print(output.bmp_temp, 2); Serial1.println();
    Serial1.flush();

    return;
}
