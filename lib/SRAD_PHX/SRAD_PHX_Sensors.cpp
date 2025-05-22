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

/**
 * Reads the Adafruit LSM6DS032 6 DoF Accelerometer/Gyroscope.
 * It's index in the sensorStatus is 0.
 * @param LSM Referenece to initialized sensor instance
 * @returns Returns `true` if the operation succeeds, False if the operation fails
 */
uint8_t FLIGHT::read_LSM(Adafruit_LSM6DSO32 &LSM) {
    sensors_event_t accel, gyro, temp;

    // Attempt to read sensor data
    if(!LSM.getEvent(&accel, &gyro, &temp))
    {
        output.sensorStatus.set(0);
        return 1;  // Return true if read fails
    }

    // Store gyroscope data
    output.lsm_gyro.x = gyro.gyro.x;
    output.lsm_gyro.y = gyro.gyro.y;
    output.lsm_gyro.z = gyro.gyro.z;

    // Store accelerometer data
    output.lsm_acc.x = accel.acceleration.x;
    output.lsm_acc.y = accel.acceleration.y;
    output.lsm_acc.z = accel.acceleration.z;

    // Store temperature data
    output.lsm_temp = float(temp.temperature);

    output.sensorStatus.reset(0);
    return 0;  // Return false if read succeeds
}

/**
 * Reads the Adafruit BMP388 Precision Barometer and Altimeter
 * It's index in sensorStatus is 1.
 * @param BMP Reference to initialized sensor instance\
 * @return Returns `true` if operation succeeds
 */
uint8_t FLIGHT::read_BMP(Adafruit_BMP3XX &BMP) {
    if (!BMP.performReading()) {
        output.sensorStatus.set(1);
        return 1;
    }
    output.bmp_temp = BMP.temperature;
    output.bmp_press = BMP.pressure;

    output.bmp_alt = BMP.readAltitude(1013.25) - alt_offset;

    if(STATE < STATES::FLIGHT_ASCENT) {
        output.bmp_alt = BMP.readAltitude(1013.25);   //uncalibrated/true altitude
    } else {
        output.bmp_alt = BMP.readAltitude(1013.25) - alt_offset;    //sea level can fluctuate under +/- 7 
                                                                    // depends on the data of the day. 
                                                                    //But 1013.25 is an acceptable value.
    }
    
    if(++altReadings_ind == 10) {
        altReadings_ind = 0;
    }
    altReadings[altReadings_ind] = output.bmp_alt;

    output.sensorStatus.reset(1);
    return 0;
}

/**
 * Reads the Adafruit ADXL_375 High-G Accelerometer
 * It's index in sensor status is 2.
 * @param ADXL Reference to initialized sensor instance
 * @return Returns `true`if operation succeeds
 */
uint8_t FLIGHT::read_ADXL(Adafruit_ADXL375 &ADXL) {
    sensors_event_t event;
    if (!ADXL.getEvent(&event)) {
        output.sensorStatus.set(2);
        return 1;
    }
    output.adxl_acc.x = event.acceleration.x;
    output.adxl_acc.y = event.acceleration.y;
    output.adxl_acc.z = event.acceleration.z;

    output.adxl_temp = float(event.temperature);

    output.sensorStatus.reset(2);
    return 0;
}

/**
 * Returns Adafruit BNO055 Absolute Orientation Sensor
 * It's index in sensorStatus is 3.
 * @param BNO Initialized sensor instance
 * @return Returns `true` if operation succeeds
 */
uint8_t FLIGHT::read_BNO(Adafruit_BNO055 &BNO) {
    sensors_event_t orientationData, angVelocityData, magnetometerData, accelerometerData;

    if (!BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
        output.sensorStatus.set(3);
        return 1;
    }
    if (!BNO.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
        output.sensorStatus.set(3);
        return 1;
    }
    if (!BNO.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER)) {
        output.sensorStatus.set(3);
        return 1;
    }
    if (!BNO.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER)) {
        output.sensorStatus.set(3);
        return 1;
    }

    imu::Quaternion quat = BNO.getQuat();
    output.bno_orientation.w = quat.w();
    output.bno_orientation.x = quat.x();
    output.bno_orientation.y = quat.y();
    output.bno_orientation.z = quat.z();

    output.bno_gyro.x = angVelocityData.gyro.x;
    output.bno_gyro.y = angVelocityData.gyro.y;
    output.bno_gyro.z = angVelocityData.gyro.z;

    output.bno_acc.x = accelerometerData.acceleration.x;
    output.bno_acc.y = accelerometerData.acceleration.y;
    output.bno_acc.z = accelerometerData.acceleration.z;

    output.bno_temp = float(BNO.getTemp());

    output.sensorStatus.reset(3);
    return 0;
}

/**
 * Reads Adafruit Ultimate GPS Breakout V3
 * It's index in sensorStatus is 4.
 * @param GPS Initialized Sensor instance
 * @return Returns `false` if GPS isn't ready in 500ms or no satellite fix, returns `true` otherwise
 */
uint8_t FLIGHT::read_GPS(Adafruit_GPS &GPS) {
    last_gps = GPS;
    
    uint32_t startms = millis();
    uint32_t timeout = startms + 500;

    while (millis() < timeout) {
        while (GPS.available()) {
            GPS.read();

            if (GPS.newNMEAreceived()) {
                // Serial.println(GPS.lastNMEA());
                if (!GPS.parse(GPS.lastNMEA())) {
                    continue;
                }

                if (GPS.fix && GPS.satellites > 0) {
                    // Serial.print("Satellites: ");
                    // Serial.println(GPS.satellites);
                    output.sensorStatus.reset(4);
                    return 0;
                }
            }
        }
    }

    output.sensorStatus.set(4);
    return 1;
}

