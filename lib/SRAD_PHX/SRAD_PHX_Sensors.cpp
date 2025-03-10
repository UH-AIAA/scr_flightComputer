#include "SRAD_PHX.h"

// MAKE SURE TO PASS BY REFERENCE (use the &)
bool FLIGHT::read_LSM(Adafruit_LSM6DSO32 &LSM) {
    sensors_event_t accel, gyro, temp;

  // Attempt to read sensor data
    if(!LSM.getEvent(&accel, &gyro, &temp))
    {
        return false;  // Return false if read fails
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

    return true;  // Return true if read succeeds
}

//BMP sensor (Temp & Pressure)
bool FLIGHT::read_BMP(Adafruit_BMP3XX &BMP) {
    if (!BMP.performReading()) {
        return false;
    }
    output.bmp_temp = BMP.temperature;
    output.bmp_press = BMP.pressure;
    output.bmp_alt = BMP.readAltitude(1013.25) - output.off_alt;
    return true;
}

//ADXL sensor (200g Accelerometer)
bool FLIGHT::read_ADXL(Adafruit_ADXL375 &ADXL) {
    sensors_event_t event;
    if (!ADXL.getEvent(&event)) {
        return false;
    }
    output.adxl_acc.x = event.acceleration.x;
    output.adxl_acc.y = event.acceleration.y;
    output.adxl_acc.z = event.acceleration.z;

    output.adxl_temp = float(event.temperature);
    return true;
}

// BNO sensor (Orientation (accelerometer, gyroscope and magnetometer)
bool FLIGHT::read_BNO(Adafruit_BNO055 &BNO) {
    sensors_event_t orientationData, angVelocityData, magnetometerData, accelerometerData;

    if (!BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
        return false;
    }
    if (!BNO.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
        return false;
    }
    if (!BNO.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER)) {
        return false;
    }
    if (!BNO.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER)) {
        return false;
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
    return true;
}
