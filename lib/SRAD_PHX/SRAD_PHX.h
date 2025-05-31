#ifndef SRAD_PHX_H
#define SRAD_PHX_H

#include<bitset>

#include <Arduino.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>

//#include <SerialTransfer.h>
#include <EasyTransfer.h> //using EasyTransfer lib instead
#include <Quaternion.h>

struct FlightData {
    // data collected by sensors
    Vector3 lsm_gyro, lsm_acc;                      // Gyroscope/Accelerometer  (LSM6DS032 Chip)
    Vector3 adxl_acc;                               // Acceleromter (AXDL375 Chip)
    Vector3 bno_gyro, bno_acc, bno_mag;             // Gyro/Accel/Magnetic Flux (BNO055 Chip)
    Quaternion bno_orientation;                     // Orientation (also BNO055)
    float lsm_temp, adxl_temp, bno_temp;            // Temperature (all chips that record)
    float bmp_temp, bmp_press, bmp_alt;             // Barometer Pressure/Altitude (BMP388 Chip)

    std::bitset<5> sensorStatus;
    uint64_t totalTime_ms;
};

struct TelemetryData { // Easy transfer can only work with basic data types 
                      //(int, float, etc.. but not vector3 stuff due to unpredictability)
    float lsm_gyro_x, lsm_gyro_y, lsm_gyro_z;
    float lsm_acc_x, lsm_acc_y, lsm_acc_z;
    float adxl_acc_x, adxl_acc_y, adxl_acc_z;
    float bno_gyro_x, bno_gyro_y, bno_gyro_z;
    float bno_acc_x, bno_acc_y, bno_acc_z;
    float bno_mag_x, bno_mag_y, bno_mag_z;
    float bno_ori_w, bno_ori_x, bno_ori_y, bno_ori_z;
    float lsm_temp, adxl_temp, bno_temp, bmp_temp;
    float bmp_press, bmp_alt;
    uint8_t sensor_status[4];  // bitset not supported by EasyTransfer
};

// struct __attribute__((packed)) TransmitFlightData {
//     // data collected by sensors
//     Vector3 lsm_gyro, lsm_acc;                      // Gyroscope/Accelerometer  (LSM6DS032 Chip)
//     Vector3 adxl_acc;                               // Acceleromter (AXDL375 Chip)
//     Vector3 bno_gyro, bno_acc, bno_mag;             // Gyro/Accel/Magnetic Flux (BNO055 Chip)
//     Quaternion bno_orientation;                     // Orientation (also BNO055)
//     float lsm_temp, adxl_temp, bno_temp;            // Temperature (all chips that record)
//     float bmp_temp, bmp_press, bmp_alt;             // Barometer Pressure/Altitude (BMP388 Chip)
  
//     std::bitset<5> sensorStatus;
//     uint64_t totalTime_ms;
// };

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
        FLIGHT(int a1, int a2, int l1, int l2, String h, Adafruit_GPS& g, FlightData& o) 
        : accel_liftoff_threshold(a1), accel_liftoff_time_threshold(a2), 
        land_time_threshold(l1), land_altitude_threshold(l2), data_header(h), last_gps(g), output(o) {
            STATE = STATES::PRE_NO_CAL;
            runningTime_ms = 0;

            // initialize arrays!
            altReadings_ind = 0;
            for(int i = 0; i < 10; i++) {
                altReadings[i] = 0;
            }
        }
        // constructor to automatically cast integer outputs from helpfer functions
        // FLIGHT(int stateVal) :  STATE(static_cast<STATES>(stateVal)) {}

        // high level functions
        void calculateState();
        uint8_t read_LSM(Adafruit_LSM6DSO32 &);
        uint8_t read_BMP(Adafruit_BMP3XX &);
        uint8_t read_ADXL(Adafruit_ADXL375 &);
        uint8_t read_BNO(Adafruit_BNO055 &);
        uint8_t read_GPS(Adafruit_GPS &);
        void incrementTime();
        void writeSD(bool, File &);
        void writeSERIAL(bool, Stream &);  // Stream allows Teensy USB as well
        void writeDataToTeensy(); //no stream parameter needed for EasyTransfer
        void readDataFromTeensy(); //no stream parameter needed for EasyTransfer
        void writeDEBUG(bool, Stream &);


        // helper functions
        bool isCal();
        bool isAscent();
        bool isDescent();
        bool isLanded();
        bool calibrate();

        void initTransferSerial(Stream &);
        // FlightData decodeTransmission(TransmitFlightData);
        // TransmitFlightData prepareToTransmit(FlightData);
        void AltitudeCalibrate();
        void printRate();

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

        // data processing variables
        float alt_offset;                   // DO NOT MODIFY
        float prev_alt, v_vel, offset_alt_fixed_temp;
        Vector3 angular_offset;             // GPS has some orientation bias -- this corrects when calibrated.
        bool offset_calibrated;             // flag to tell us if we've configured this
        
        float altReadings[10];
        uint8_t altReadings_ind;


        bool calibrated = false;
        STATES STATE;
        //SerialTransfer myTransfer;

        EasyTransfer ET;
        TelemetryData txData;
        TelemetryData rxData;   
};

#endif
