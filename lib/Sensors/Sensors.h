#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>

#include <Quaternion.h>

namespace Sensors {
    bool read_LSM(Adafruit_LSM6DSO32 &, Vector3 &, Vector3 &, float &);
    bool read_BMP(Adafruit_BMP3XX &, float &, float &, float &);
    bool read_ADXL(Adafruit_ADXL375 &, Vector3 &, float &);
};


#endif