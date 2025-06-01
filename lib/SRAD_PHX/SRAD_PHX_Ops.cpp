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

/** 
 * @brief tracks time during flight
 * 
 * This function calls the millis() function from the
 * Arduino library to track milliseconds since the board
 * was powered on. It updates three variables: 
 * 
 * 1. `deltaTime_ms`
 * 2. `runningTime_ms`
 * 3. `output.totalTime_ms`
 */
void FLIGHT::incrementTime() {
    uint64_t newRunningTime_ms = millis();
    deltaTime_ms = newRunningTime_ms - runningTime_ms;
    runningTime_ms = newRunningTime_ms;
    output.totalTime_ms = newRunningTime_ms;
}

/**
 * @brief writes data stored in `output` to file
 * @param headers If true, function will only right headers and return early
 * @param File A reference to Arduino file type from SD.h
 * 
 * This function can write data headers or current data to SD card.
 */
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
    outputFile.print(output.bno_gyro.x, 5); outputFile.print(",");
    outputFile.print(output.bno_gyro.y, 5); outputFile.print(",");
    outputFile.print(output.bno_gyro.z, 5); outputFile.print(",");
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
    outputFile.print(output.sensorStatus.test(0)); outputFile.print(", ");
    outputFile.print(output.sensorStatus.test(1)); outputFile.print(", ");
    outputFile.print(output.sensorStatus.test(2)); outputFile.print(", ");
    outputFile.print(output.sensorStatus.test(3)); outputFile.print(", ");
    outputFile.print(output.sensorStatus.test(4)); outputFile.println();
    outputFile.flush();

    return;
}

/**
 * @brief writes data stored in `output` to a serial port
 * @param headers If true, function will only right headers and return early
 * @param Serial1 The serial port to write data to
 * 
 * This function can write data headers or current data to a serial port.
 */
void FLIGHT::writeSERIAL(bool headers, Stream& outputSerial) {
    if(headers) {
        outputSerial.println(data_header);
        outputSerial.flush();
        return;
    }

    outputSerial.print(output.totalTime_ms); outputSerial.print(",");
    if(last_gps.fix) {
        outputSerial.print(last_gps.latitudeDegrees, 6); outputSerial.print(",");
        outputSerial.print(last_gps.longitudeDegrees, 6); outputSerial.print(",");
        outputSerial.print((int32_t)last_gps.satellites); outputSerial.print(",");
        outputSerial.print(last_gps.speed, 3); outputSerial.print(",");
        outputSerial.print(last_gps.angle, 3); outputSerial.print(",");
        outputSerial.print(last_gps.altitude, 3); outputSerial.print(",");
    } else {
        outputSerial.print("-1,No fix,-1,No fix,0,-1,-1,-1,");
    }
    outputSerial.print(output.lsm_gyro.x, 5); outputSerial.print(",");
    outputSerial.print(output.lsm_gyro.y, 5); outputSerial.print(",");
    outputSerial.print(output.lsm_gyro.z, 5); outputSerial.print(",");
    outputSerial.print(output.bno_orientation.w, 5); outputSerial.print(",");
    outputSerial.print(output.bno_orientation.x, 5); outputSerial.print(",");
    outputSerial.print(output.bno_orientation.y, 5); outputSerial.print(",");
    outputSerial.print(output.bno_orientation.z, 5); outputSerial.print(",");
    outputSerial.print(output.bno_gyro.x, 5); outputSerial.print(",");
    outputSerial.print(output.bno_gyro.y, 5); outputSerial.print(",");
    outputSerial.print(output.bno_gyro.z, 5); outputSerial.print(",");
    outputSerial.print(output.bno_acc.x, 4); outputSerial.print(",");
    outputSerial.print(output.bno_acc.y, 4); outputSerial.print(",");
    outputSerial.print(output.bno_acc.z, 4); outputSerial.print(",");
    outputSerial.print(output.adxl_acc.x, 2); outputSerial.print(",");
    outputSerial.print(output.adxl_acc.y, 2); outputSerial.print(",");
    outputSerial.print(output.adxl_acc.z, 2); outputSerial.print(",");
    outputSerial.print(output.bmp_press, 6); outputSerial.print(",");
    outputSerial.print(output.bmp_alt, 4); outputSerial.print(",");
    outputSerial.print(output.lsm_temp, 2); outputSerial.print(",");
    outputSerial.print(output.adxl_temp, 2); outputSerial.print(",");
    outputSerial.print(output.bno_temp, 2); outputSerial.print(",");
    outputSerial.print(output.bmp_temp, 2); outputSerial.println();
    outputSerial.print(output.sensorStatus.test(0)); outputSerial.print(", ");
    outputSerial.print(output.sensorStatus.test(1)); outputSerial.print(", ");
    outputSerial.print(output.sensorStatus.test(2)); outputSerial.print(", ");
    outputSerial.print(output.sensorStatus.test(3)); outputSerial.print(", ");
    outputSerial.print(output.sensorStatus.test(4)); outputSerial.println();
    outputSerial.flush();

    return;
}

void FLIGHT::writeDEBUG(bool headers, Stream &outputSerial) {
    if(headers) {
        outputSerial.println(data_header);
        outputSerial.flush();
        return;
    }

    outputSerial.print("Uptime (ms): ");outputSerial.print(output.totalTime_ms); outputSerial.print(", \n");
    outputSerial.print("State: "); outputSerial.println(STATE); outputSerial.println("\n");
    if(last_gps.fix) {
        outputSerial.print("GPS Latitude Degrees: ");outputSerial.print(last_gps.latitudeDegrees, 6); outputSerial.println(", ");
        outputSerial.print("GPS Longitude Degrees: ");outputSerial.print(last_gps.longitudeDegrees, 6); outputSerial.println(",");
        outputSerial.print("GPS satellites: ");outputSerial.print((int32_t)last_gps.satellites); outputSerial.print(",");
        outputSerial.print("GPS speed: ");outputSerial.print(last_gps.speed, 3); outputSerial.print(",");
        outputSerial.print("GPS angle: ");outputSerial.print(last_gps.angle, 3); outputSerial.print(",");
        outputSerial.print("GPS altitude: ");outputSerial.println(last_gps.altitude, 3); outputSerial.println();
    } else {
        outputSerial.println("-1,No fix,-1,No fix,0,-1,-1,-1,\n");
    }
    // LSM data
    outputSerial.print("LSM Gyro X: "); outputSerial.print(output.lsm_gyro.x, 5); outputSerial.print(",");
    outputSerial.print("LSM Gyro Y: "); outputSerial.print(output.lsm_gyro.y, 5); outputSerial.print(",");
    outputSerial.print("LSM Gyro Z: "); outputSerial.print(output.lsm_gyro.z, 5); outputSerial.println(",");

    outputSerial.print("LSM Acc X: "); outputSerial.print(output.lsm_acc.x, 5); outputSerial.print(",");
    outputSerial.print("LSM Acc Y: "); outputSerial.print(output.lsm_acc.y, 5); outputSerial.print(",");
    outputSerial.print("LSM Acc Z: "); outputSerial.print(output.lsm_acc.z, 5); outputSerial.println(",");

    //BNO data
        //orientation
    outputSerial.print("BNO W-Orientation: ");outputSerial.print(output.bno_orientation.w, 5); outputSerial.print(",");
    outputSerial.print("BNO X-Orientation: ");outputSerial.print(output.bno_orientation.x, 5); outputSerial.print(",");
    outputSerial.print("BNO Y-Orientation: ");outputSerial.print(output.bno_orientation.y, 5); outputSerial.print(",");
    outputSerial.print("BNO Z-Orientation: ");outputSerial.print(output.bno_orientation.z, 5); outputSerial.println(",");
        //gyro
    outputSerial.print("BNO X-Gyro: ");outputSerial.print(output.bno_gyro.x, 5); outputSerial.print(",");
    outputSerial.print("BNO Y-Gyro: ");outputSerial.print(output.bno_gyro.y, 5); outputSerial.print(",");
    outputSerial.print("BNO Z-Gyro: ");outputSerial.print(output.bno_gyro.z, 5); outputSerial.println(",");
        //Accel
    outputSerial.print("BNO X-Accel: ");outputSerial.print(output.bno_acc.x, 4); outputSerial.print(",");
    outputSerial.print("BNO Y-Accel: ");outputSerial.print(output.bno_acc.y, 4); outputSerial.print(",");
    outputSerial.print("BNO Z-Accel: ");outputSerial.print(output.bno_acc.z, 4); outputSerial.println(",");

    //ADXL data
    outputSerial.print("ADXL X_Accel: ");outputSerial.print(output.adxl_acc.x, 2); outputSerial.print(",");
    outputSerial.print("ADXL Y_Accel: ");outputSerial.print(output.adxl_acc.y, 2); outputSerial.print(",");
    outputSerial.print("ADXL Z_Accel: ");outputSerial.print(output.adxl_acc.z, 2); outputSerial.println(",");

    //BMP data
    outputSerial.print("BMP Pressure: ");outputSerial.print(output.bmp_press, 6); outputSerial.print(",");
    outputSerial.print("BMP Altitude: ");outputSerial.print(output.bmp_alt, 4); outputSerial.println(",");

    //Temperature data
    outputSerial.print("LSM Temp: ");outputSerial.print(output.lsm_temp, 2); outputSerial.print(",");
    outputSerial.print("ADXL Temp: ");outputSerial.print(output.adxl_temp, 2); outputSerial.print(",");
    outputSerial.print("BNO Temp: ");outputSerial.print(output.bno_temp, 2); outputSerial.print(",");
    outputSerial.print("BMP Temp: ");outputSerial.print(output.bmp_temp, 2); outputSerial.println("\n");

    //Sensor status
    outputSerial.println("Sensor Status:");
    outputSerial.print(output.sensorStatus.test(0)); outputSerial.print(", ");
    outputSerial.print(output.sensorStatus.test(1)); outputSerial.print(", ");
    outputSerial.print(output.sensorStatus.test(2)); outputSerial.print(", ");
    outputSerial.print(output.sensorStatus.test(3)); outputSerial.print(", ");
    outputSerial.print(output.sensorStatus.test(4)); outputSerial.println("\n");
    outputSerial.flush();

    return;
}

void FLIGHT::writeDataToTeensy() {
    txData.lsm_gyro_x = output.lsm_gyro.x;
    txData.lsm_gyro_y = output.lsm_gyro.y;
    txData.lsm_gyro_z = output.lsm_gyro.z;

    txData.lsm_acc_x = output.lsm_acc.x;
    txData.lsm_acc_y = output.lsm_acc.y;
    txData.lsm_acc_z = output.lsm_acc.z;

    txData.adxl_acc_x = output.adxl_acc.x;
    txData.adxl_acc_y = output.adxl_acc.y;
    txData.adxl_acc_z = output.adxl_acc.z;

    txData.bno_gyro_x = output.bno_gyro.x;
    txData.bno_gyro_y = output.bno_gyro.y;
    txData.bno_gyro_z = output.bno_gyro.z;

    txData.bno_acc_x = output.bno_acc.x;
    txData.bno_acc_y = output.bno_acc.y;
    txData.bno_acc_z = output.bno_acc.z;

    txData.bno_mag_x = output.bno_mag.x;
    txData.bno_mag_y = output.bno_mag.y;
    txData.bno_mag_z = output.bno_mag.z;

    txData.bno_ori_w = output.bno_orientation.w;
    txData.bno_ori_x = output.bno_orientation.x;
    txData.bno_ori_y = output.bno_orientation.y;
    txData.bno_ori_z = output.bno_orientation.z;

    txData.lsm_temp = output.lsm_temp;
    txData.adxl_temp = output.adxl_temp;
    txData.bno_temp = output.bno_temp;
    txData.bmp_temp = output.bmp_temp;
    txData.bmp_press = output.bmp_press;
    txData.bmp_alt = output.bmp_alt;

    for (int i = 0; i < 5; i++) { // Fixed bit sized. Bit was declared 5 in FlightData struct. 
                                      //4 might cause data missing
        txData.sensor_status[i] = output.sensorStatus[i];
    }

    ET.sendData();
}

void FLIGHT::readDataFromTeensy() {
    if (ET.receiveData()) {
        output.lsm_gyro.x = rxData.lsm_gyro_x;
        output.lsm_gyro.y = rxData.lsm_gyro_y;
        output.lsm_gyro.z = rxData.lsm_gyro_z;

        output.lsm_acc.x = rxData.lsm_acc_x;
        output.lsm_acc.y = rxData.lsm_acc_y;
        output.lsm_acc.z = rxData.lsm_acc_z;

        output.adxl_acc.x = rxData.adxl_acc_x;
        output.adxl_acc.y = rxData.adxl_acc_y;
        output.adxl_acc.z = rxData.adxl_acc_z;

        output.bno_gyro.x = rxData.bno_gyro_x;
        output.bno_gyro.y = rxData.bno_gyro_y;
        output.bno_gyro.z = rxData.bno_gyro_z;

        output.bno_acc.x = rxData.bno_acc_x;
        output.bno_acc.y = rxData.bno_acc_y;
        output.bno_acc.z = rxData.bno_acc_z;

        output.bno_mag.x = rxData.bno_mag_x;
        output.bno_mag.y = rxData.bno_mag_y;
        output.bno_mag.z = rxData.bno_mag_z;

        output.bno_orientation.w = rxData.bno_ori_w;
        output.bno_orientation.x = rxData.bno_ori_x;
        output.bno_orientation.y = rxData.bno_ori_y;
        output.bno_orientation.z = rxData.bno_ori_z;

        output.lsm_temp = rxData.lsm_temp;
        output.adxl_temp = rxData.adxl_temp;
        output.bno_temp = rxData.bno_temp;
        output.bmp_temp = rxData.bmp_temp;
        output.bmp_press = rxData.bmp_press;
        output.bmp_alt = rxData.bmp_alt;

        for (int i = 0; i < 5; i++) { // Fixed bit sized. Bit was declared 5 in FlightData struct. 
                                      //4 might cause data missing
            output.sensorStatus[i] = rxData.sensor_status[i];
        }
    }
}

void FLIGHT::initTransferSerial(Stream &transferSerial) {
    ET.begin(details(txData), &transferSerial);
}

// FlightData FLIGHT::decodeTransmission(TransmitFlightData s) {
//     return {
//         s.lsm_gyro, s.lsm_acc,
//         s.adxl_acc,
//         s.bno_gyro, s.bno_acc, s.bno_mag,
//         s.bno_orientation,
//         s.lsm_temp, s.adxl_temp, s.bno_temp,
//         s.bmp_temp, s.bmp_press, s.bmp_alt,
//         s.sensorStatus
//     };
// }

// TransmitFlightData FLIGHT::prepareToTransmit(FlightData s) {
//     return {
//         s.lsm_gyro, s.lsm_acc,
//         s.adxl_acc,
//         s.bno_gyro, s.bno_acc, s.bno_mag,
//         s.bno_orientation,
//         s.lsm_temp, s.adxl_temp, s.bno_temp,
//         s.bmp_temp, s.bmp_press, s.bmp_alt,
//         s.sensorStatus
//     };
// }

void FLIGHT::printRate() {
    Serial.print("Cycle Time: "); Serial.println(deltaTime_ms);
    Serial.print("Cycle Rate: "); Serial.println(1000.0/float(deltaTime_ms));
}
