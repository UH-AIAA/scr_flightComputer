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
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Chip Imports
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>

// Computing imports
#include <SRAD_PHX.h>

// GLOBAL UART
#ifdef UART_TEENSY
    // Assigning IDs to sensors
    Adafruit_GPS GPS(&Serial2);

    // Logging
    File data;

    const String data_header =
    "time,lat,lon,"
    "satellites,speed,g_angle,gps_alt,"
    "lsm_gyro_x, lsm_gyro_y, lsm_gyro_z, lsm_acc_x, lsm_acc_y, lsm_acc_z"
    "bno ori w,bno ori x,bno ori y,bno ori z,"
    "bno_rate_x,bno_rate_y,bno_rate_z,"
    "bno_accel_x,bno_accel_y,bno_accel_z,"
    "adxl_accel_x,adxl_accel_y,adxl_accel_z,"
    "bmp_pressure,bmp_altitude,"
    "lsm_temp,adxl_temp,bno_temp,bmp_temp,"
    "lsm_status,bmp_status,adxl_status,bno_status,gps_status";

    FlightData currentData;
    FLIGHT OPS = FLIGHT(data_header, GPS, currentData);
#endif

// GLOBAL SPI
#ifdef SPI_TEENSY
    // config thresholds
    #define accel_liftoff_threshold                    30  // METERS PER SECOND^2
    #define accel_liftoff_time_threshold              250  // MILLISECONDS
    #define land_time_threshold                     30000  // MILLISECONDS
    #define land_altitude_threshold                    50  // METERS

    // Defining variables
    const uint32_t PIN_BUZZER = 33;
    const uint32_t PIN_LED = 29;
    const uint32_t LSM_CS = 40;
    const uint32_t BMP_CS = 41;
    const uint32_t ADXL_CS = 39;

    // Assigning IDs to sensors
    Adafruit_LSM6DSO32 LSM;
    Adafruit_BMP3XX BMP;
    Adafruit_ADXL375 ADXL(ADXL_CS, &SPI, 12345);
    Adafruit_BNO055 BNO(55, 0x28, &Wire);

    // Logging
    // File data;

    const String data_header =
    "time,lat,lon,"
    "satellites,speed,g_angle,gps_alt,"
    "lsm_gyro_x, lsm_gyro_y, lsm_gyro_z, lsm_acc_x, lsm_acc_y, lsm_acc_z"
    "bno ori w,bno ori x,bno ori y,bno ori z,"
    "bno_rate_x,bno_rate_y,bno_rate_z,"
    "bno_accel_x,bno_accel_y,bno_accel_z,"
    "adxl_accel_x,adxl_accel_y,adxl_accel_z,"
    "bmp_pressure,bmp_altitude,"
    "lsm_temp,adxl_temp,bno_temp,bmp_temp,"
    "lsm_status,bmp_status,adxl_status,bno_status,gps_status";

    FlightData currentData;
    FLIGHT OPS = FLIGHT(accel_liftoff_threshold, accel_liftoff_time_threshold, land_time_threshold, land_altitude_threshold, data_header, currentData);
#endif

#define FLAG_PIN 24


void setup() {
    
    // SETUP (GLOBAL)
    Serial.begin(115200); // USB Serial Port
    #ifdef DEBUG
        delay(10000);
        Serial.print(CrashReport);
    #endif

    // // init SD card
    // while(!SD.begin(BUILTIN_SDCARD)) {
    //     Serial.println(F("SD not found..."));  // Print error message if SD card not found
    //     delay(1000);  // Wait for 1 second before retrying
    // }
    // Serial.println(F("SD initialized"));
    // SD.begin(BUILTIN_SDCARD);

    // // Create data logging file
    // char dataname[17] = "FL0.csv";
    // for (int i = 0; SD.exists(dataname); i++) {
    //     sprintf(dataname, "FL%d.csv", i);
    // }

    // // Open file for writing
    // data = SD.open(dataname, FILE_WRITE);

    // // Print data header
    // OPS.writeSD(true, data);

    #ifdef DEBUG
        OPS.writeDEBUG(true, Serial);
    #endif

    // SETUP (UART)
    #ifdef UART_TEENSY
        Serial1.begin(9600); // Radio Serial Port
        Serial2.begin(9600); // GPS Serial Port (Default hardware at 9600)
        Serial8.begin(9600);
        OPS.initTransferSerial(Serial8);
        pinMode(FLAG_PIN, OUTPUT);
        // GPS Initialization and Configuration
        GPS.begin(9600);
        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
        OPS.writeSERIAL(true, Serial1);
    #endif
    

    // SETUP (SPI) 
    #ifdef SPI_TEENSY
        Serial4.begin(9600);
        OPS.initTransferSerial(Serial4);
        pinMode(FLAG_PIN, INPUT);
        SPI.begin();
        // Configure LSM6DSO32
        while(!LSM.begin_SPI(LSM_CS, &SPI)) {
            Serial.println(F("LSM6DSO32 not found..."));  // Print error message if sensor not found
            delay(1000);  // Wait for 1 second before retrying
        }
        Serial.println(F("LSM6DSO32 initialized")); 
        LSM.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
        LSM.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
        LSM.setAccelDataRate(LSM6DS_RATE_416_HZ);
        LSM.setGyroDataRate(LSM6DS_RATE_416_HZ);

        // Configure BMP388. (Fixed naming)
        while(!BMP.begin_SPI(BMP_CS, &SPI)) {
            Serial.println(F("BMP388 not found..."));  // Print error message if sensor not found
            delay(1000);  // Wait for 1 second before retrying
        }
        Serial.println(F("BMP388 initialized")); 
        BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
        BMP.setPressureOversampling(BMP3_OVERSAMPLING_16X);
        BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        BMP.setOutputDataRate(BMP3_ODR_200_HZ);

        // Configure ADXL
        while(!ADXL.begin()) {
            Serial.println(F("ADXL375 not found..."));  // Print error message if sensor not found
            delay(1000);  // Wait for 1 second before retrying
        }
        Serial.println(F("ADXL375 initialized"));

        ADXL.setDataRate(ADXL343_DATARATE_200_HZ);

        // Configure BNO055
        while(!BNO.begin()) {
            Serial.println(F("BNO055 not found..."));  // Print error message if sensor not found
            delay(1000);  // Wait for 1 second before retrying
        }
        Serial.println(F("BNO055 initialized"));

        BNO.setMode(OPERATION_MODE_CONFIG);
        BNO.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
        BNO.setMode(OPERATION_MODE_NDOF);
    #endif
}

void loop() {
    #ifdef SPI_TEENSY 
        OPS.incrementTime();
        OPS.read_BMP(BMP);
        OPS.read_ADXL(ADXL);
        OPS.read_BNO(BNO);
        OPS.read_LSM(LSM);
        OPS.calculateState();
        if(digitalRead(FLAG_PIN) == HIGH) {
            #ifdef DEBUG
                Serial.println("Transmitting...");
            #endif
            
            OPS.writeDataToTeensy();
        }

        #ifdef DEBUG
            Serial.println("Debug data:");
            // OPS.writeDEBUG(false, Serial);
        #else 
            OPS.printRate(); // function to figure out what our cycle rate is
        #endif
    #endif

    #ifdef UART_TEENSY
        OPS.read_GPS(GPS);
        #ifdef DEBUG
            Serial.println("Writing pin high...");
        #endif
        digitalWrite(FLAG_PIN, HIGH);
        // delay(25);
        #ifdef DEBUG
            Serial.println("Reading from spi teensy...");
        #endif
        OPS.readDataFromTeensy();
        // delay(75);
        #ifdef DEBUG
            Serial.println("Writing pin low...");
        #endif
        digitalWrite(FLAG_PIN, LOW);
        OPS.incrementTime();
        OPS.writeSD(false, data);
        OPS.writeSERIAL(false, Serial1);
        #ifdef DEBUG
            OPS.writeDEBUG(false, Serial);
        #endif
    #endif
}
