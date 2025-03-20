/* SRAD Avionics Flight Software for AIAA-UH
 *
 * Copyright (c) 2024 Nathan Samuell + Dedah + Thanh! (www.github.com/nathansamuell, www.github.com/UH-AIAA)
 *
 * More information on the MIT license as well as a complete copy
 * of the license can be found here: https://choosealicense.com/licenses/mit/
 *
 * All above text must be included in any redistribution.
 */


// I/O imports
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Chip Imports
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
//#include <Adafruit_BNO055.h>
// #include <Adafruit_BMP3XX.h>
// #include <Adafruit_LSM6DSO32.h> TODO: Note that this import is no longer needed here after migrating function to Sensors library

// Computing imports
#include <Quaternion.h>
#include <SRAD_PHX.h>

// Defining variables
const uint32_t PIN_BUZZER = 33;
const uint32_t PIN_LED = 29;
const uint32_t LSM_CS = 40;
const uint32_t BMP_CS = 41;
const uint32_t ADXL_CS = 39;
int32_t mstime;

// config thresholds
#define accel_liftoff_threshold                    30  // METERS PER SECOND^2
#define accel_liftoff_time_threshold              250  // MILLISECONDS
#define land_time_threshold                     30000  // MILLISECONDS
#define land_altitude_threshold                    50  // METERS

// Assigning IDs to sensors
Adafruit_LSM6DSO32 LSM;
Adafruit_BMP3XX BMP;
Adafruit_ADXL375 ADXL(ADXL_CS, &SPI, 12345);
Adafruit_BNO055 BNO(55, 0x28, &Wire);
Adafruit_GPS GPS(&Serial2);

FlightData currentData;
const String data_header =
    "time,lat,lon,"
    "satellites,speed,g_angle,gps_alt,"
    "bno ori w,bno ori x,bno ori y,bno ori z,"
    "bno_rate_x,bno_rate_y,bno_rate_z,"
    "bno_accel_x,bno_accel_y,bno_accel_z,"
    "adxl_accel_x,adxl_accel_y,adxl_accel_z,"
    "pressure,altitude,"
    "lsm_temp,adxl_temp,bno_temp,bmp_temp";

FLIGHT OPS = FLIGHT(accel_liftoff_threshold, accel_liftoff_time_threshold, land_time_threshold, land_altitude_threshold, data_header, currentData);

// Logging
bool log_enable = true;
File data;

void setup() {
    // USB Serial Port
    Serial.begin(115200);
    Serial1.begin(9600); // Radio Serial Port
    Serial2.begin(9600); // GPS Serial Port (Default hardware at 9600)

    // Configure LSM6DSO32
    LSM.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    LSM.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
    LSM.setAccelDataRate(LSM6DS_RATE_416_HZ);
    LSM.setGyroDataRate(LSM6DS_RATE_416_HZ);

    // Configure BMP390
    BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
    BMP.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    BMP.setOutputDataRate(BMP3_ODR_200_HZ);

    // Configure ADXL
    ADXL.setDataRate(ADXL343_DATARATE_200_HZ);

    // Configure BNO055
    BNO.setMode(OPERATION_MODE_CONFIG);
    BNO.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
    BNO.setMode(OPERATION_MODE_NDOF);

    // GPS Initialization and Configuration
    GPS.begin(115200);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

    // Create data logging file
    char dataname[17] = "FL0.csv";
    for (int i = 0; SD.exists(dataname); i++) {
        sprintf(dataname, "FL%d.csv", i);
    }

    // Open file for writing
    data = SD.open(dataname, FILE_WRITE);

    // Print data header
    OPS.writeSD(true, data);
    OPS.writeSERIAL(true, Serial1);
}

void loop() {
    mstime = millis();
    OPS.read_BMP(BMP);
    OPS.read_ADXL(ADXL);
    OPS.read_BNO(BNO);
    OPS.read_LSM(LSM);
    OPS.read_GPS(GPS);
    OPS.writeSD(false, data);
    OPS.writeSERIAL(false, Serial1);

    delay(50);

    // Debugging information
    Serial.print(F("Longitude: ")); Serial.print(GPS.longitude); Serial.print(" ");
    Serial.print(F("Latitude: ")); Serial.print(GPS.latitude); Serial.print(" ");
}
