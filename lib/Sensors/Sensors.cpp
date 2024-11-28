#include "Sensors.h"

bool Sensors::read_LSM() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // Attempt to read sensor data
  if(!LSM.getEvent(&accel, &gyro, &temp))
  {
    return false;  // Return false if read fails
  }

  // Store gyroscope data
  lsm_gyro.x = gyro.gyro.x;
  lsm_gyro.y = gyro.gyro.y;
  lsm_gyro.z = gyro.gyro.z;

  // Store accelerometer data
  lsm_acc.x = accel.acceleration.x;
  lsm_acc.y = accel.acceleration.y;
  lsm_acc.z = accel.acceleration.z;

  // Store temperature data
  lsm_temp = float(temp.temperature);

  return true;  // Return true if read succeeds
}