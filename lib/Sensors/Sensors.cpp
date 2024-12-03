#include "Sensors.h"

// MAKE SURE TO PASS BY REFERENCE (use the &)
bool Sensors::read_LSM(Adafruit_LSM6DSO32 &LSM, Vector3 &accel_data, Vector3 &gyro_data, float &temp_data) {
  sensors_event_t accel, gyro, temp;

  // Attempt to read sensor data
  if(!LSM.getEvent(&accel, &gyro, &temp))
  {
    return false;  // Return false if read fails
  }

  // Store gyroscope data
  gyro_data.x = gyro.gyro.x;
  gyro_data.y = gyro.gyro.y;
  gyro_data.z = gyro.gyro.z;

  // Store accelerometer data
  accel_data.x = accel.acceleration.x;
  accel_data.y = accel.acceleration.y;
  accel_data.z = accel.acceleration.z;

  // Store temperature data
  temp_data = float(temp.temperature);

  return true;  // Return true if read succeeds
}

//BMP sensor (Temp & Pressure)
bool Sensors::read_BMP(Adafruit_BMP3XX &BMP, float &bmp_temperature, float &pressure, float &altitude) {
    if (!BMP.performReading()) {
        return false;
    }
    bmp_temperature = BMP.temperature;
    pressure = BMP.pressure;
    altitude = BMP.readAltitude(1013.25) - off_alt;
    return true;
}

//ADXL sensor (200g Accelerometer)
bool Sensors::read_ADXL(Adafruit_ADXL375 &ADXL, Vector3 &output_acc, float &output_temp) {
  sensors_event_t event;
  if (!ADXL.getEvent(&event)) {
      return false;
  }
  output_acc.x = event.acceleration.x;
  output_acc.y = event.acceleration.y;
  output_acc.z = event.acceleration.z;

  output_temp = float(event.temperature);
  return true;
}

// BNO sensor (Orientation (accelerometer, gyroscope and magnetometer)
bool Sensors::read_BNO(Adafruit_BNO055 &BNO, Quaternion &Orientation, Vector3 &Gyroscope, Vector3 &Accelerometer, float &bno_temperature) {
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
    Orientation.w = quat.w();
    Orientation.x = quat.x();
    Orientation.y = quat.y();
    Orientation.z = quat.z();

    Gyroscope.x = angVelocityData.gyro.x;
    Gyroscope.y = angVelocityData.gyro.y;
    Gyroscope.z = angVelocityData.gyro.z;

    Accelerometer.x = accelerometerData.acceleration.x;
    Accelerometer.y = accelerometerData.acceleration.y;
    Accelerometer.z = accelerometerData.acceleration.z;

    bno_temperature = float(BNO.getTemp());
    return true;
}
