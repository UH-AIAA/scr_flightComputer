#ifndef SRAD_PHX_H
#define SRAD_PHX_H

#include <Arduino.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>

#include <Quaternion.h>

struct FlightData {
    // data collected by sensors
    Vector3 lsm_gyro, lsm_acc;                      // Gyroscope/Accelerometer  (LSM6DS032 Chip)
    Vector3 adxl_acc;                               // Acceleromter (AXDL375 Chip)
    Vector3 bno_gyro, bno_acc, bno_mag;             // Gyro/Accel/Magnetic Flux (BNO055 Chip)
    Quaternion bno_orientation;                     // Orientation (also BNO055)
    float lsm_temp, adxl_temp, bno_temp;            // Temperature (all chips that record)
    float bmp_temp, bmp_press, bmp_alt;             // Barometer Pressure/Altitude (BMP388 Chip)

    // data processing variables
    float off_alt, prev_alt, v_vel;
    Vector3 angular_offset;             // GPS has some orientation bias -- this corrects when calibrated.
    bool offset_calibrated;             // flag to tell us if we've configured this

    uint64_t totalTime_ms;
};

enum STATES {
    PRE_NO_CAL = 0,
    PRE_CAL = 1,
    FLIGHT_ASCENT = 2,
    FLIGHT_DESCENT = 3,
    POST_LANDED = 4,
};

class FLIGHT {
    public:
        // initial constructor
        FLIGHT(int a1, int a2, int l1, int l2, String h, FlightData& o) 
        : accel_liftoff_threshold(a1), accel_liftoff_time_threshold(a2), 
        land_time_threshold(l1), land_altitude_threshold(l2), data_header(h), output(o) {
            STATE = STATES::PRE_NO_CAL;
            runningTime_ms = 0;
        }
        // constructor to automatically cast integer outputs from helpfer functions
        // FLIGHT(int stateVal) :  STATE(static_cast<STATES>(stateVal)) {}
        
        // high level functions
        void calculateState();
        bool read_LSM(Adafruit_LSM6DSO32 &);
        bool read_BMP(Adafruit_BMP3XX &);
        bool read_ADXL(Adafruit_ADXL375 &);
        bool read_BNO(Adafruit_BNO055 &);
        bool read_GPS(Adafruit_GPS &);
        void incrementTime();
        void writeSD(bool, File&);
        void writeSERIAL(bool, HardwareSerial&);

        // helper functions
        bool isCal();
        bool isAscent();
        bool isDescent();
        bool isLanded();
        bool calibrate();

    private:
        int accel_liftoff_threshold;        // METERS PER SECOND^2
        int accel_liftoff_time_threshold;   // MILLISECONDS
        int land_time_threshold;            // MILLISECONDS
        int land_altitude_threshold;        // METERS
        
        FlightData& output;
        String data_header;
        Adafruit_GPS& last_gps;             // used for data collection, for some reason the GPS stores it
        uint16_t deltaTime_ms;
        uint64_t runningTime_ms;

        bool calibrated = false;
        STATES STATE;
};

#endif
