/* SRAD Avionics Flight Software for AIAA-UH
 *
 * Copyright (c) 2024 Nathan Samuell + Dedah (www.github.com/nathansamuell)
 *
 * More information on the MIT license as well as a complete copy
 * of the license can be found here: https://choosealicense.com/licenses/mit/
 *
 * All above text must be included in any redistribution.
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>

// Defining variables
const uint32_t PIN_BUZZER = 33;
const uint32_t PIN_LED = 29;
const uint32_t LSM_CS = 40;
const uint32_t BMP_CS = 41;
const uint32_t ADXL_CS = 39;
int32_t mstime;

// Assigning IDs to sensors
Adafruit_LSM6DSO32 LSM;
Adafruit_BMP3XX BMP;
Adafruit_ADXL375 ADXL(ADXL_CS, &SPI, 12345);
Adafruit_BNO055 BNO(55, 0x28, &Wire);
Adafruit_GPS GPS(&Serial2);

// Data variable assignments
// Sensor raw data
double lsm_gyrox, lsm_gyroy, lsm_gyroz, lsm_accx, lsm_accy, lsm_accz;
double adxl_accx, adxl_accy, adxl_accz;
double bno_gyrox, bno_gyroy, bno_gyroz, bno_accx, bno_accy, bno_accz;
double bno_orientationx, bno_orientationy, bno_orientationz, bno_orientationw;

// Temperatures
float lsm_temp, adxl_temp, bno_temp, bmp_temp;

// Barometer data
float bmp_press, bmp_alt;
float off_alt, prev_alt, v_vel;

// Logging
bool log_enable = true;
File data;

const String data_header =
    "time,lat,lon,"
    "satellites,speed,g_angle,gps_alt,"
    "bno ori w,bno ori x,bno ori y,bno ori z,"
    "bno_rate_x,bno_rate_y,bno_rate_z,"
    "bno_accel_x,bno_accel_y,bno_accel_z,"
    "adxl_accel_x,adxl_accel_y,adxl_accel_z,"
    "pressure,altitude,"
    "lsm_temp,adxl_temp,bno_temp,bmp_temp";

// Sensor read functions
bool read_LSM() {
    sensors_event_t accel, gyro, temp;
    if (!LSM.getEvent(&accel, &gyro, &temp)) {
        return false;
    }
    lsm_gyrox = gyro.gyro.x;
    lsm_gyroy = gyro.gyro.y;
    lsm_gyroz = gyro.gyro.z;

    lsm_accx = accel.acceleration.x;
    lsm_accy = accel.acceleration.y;
    lsm_accz = accel.acceleration.z;

    lsm_temp = float(temp.temperature);
    return true;
}

bool read_BMP() {
    if (!BMP.performReading()) {
        return false;
    }
    bmp_temp = BMP.temperature;
    bmp_press = BMP.pressure;
    bmp_alt = BMP.readAltitude(1013.25) - off_alt;
    return true;
}

bool read_ADXL() {
    sensors_event_t event;
    if (!ADXL.getEvent(&event)) {
        return false;
    }
    adxl_accx = event.acceleration.x;
    adxl_accy = event.acceleration.y;
    adxl_accz = event.acceleration.z;

    adxl_temp = float(event.temperature);
    return true;
}

bool read_BNO() {
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
    bno_orientationw = quat.w();
    bno_orientationx = quat.x();
    bno_orientationy = quat.y();
    bno_orientationz = quat.z();

    bno_gyrox = angVelocityData.gyro.x;
    bno_gyroy = angVelocityData.gyro.y;
    bno_gyroz = angVelocityData.gyro.z;

    bno_accx = accelerometerData.acceleration.x;
    bno_accy = accelerometerData.acceleration.y;
    bno_accz = accelerometerData.acceleration.z;

    bno_temp = float(BNO.getTemp());
    return true;
}

void setup() {
    // USB Serial Port
    Serial.begin(115200);
    Serial1.begin(9600); // Radio Serial Port
    Serial2.begin(115200); // GPS Serial Port

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
    data.println(data_header);
    data.flush();
}

void loop() {
    mstime = millis();
    read_ADXL();
    read_BMP();
    read_BNO();
    read_LSM();

    // Update GPS
    while (GPS.available()) {
        GPS.read();
    }
    if (GPS.newNMEAreceived()) {
        GPS.parse(GPS.lastNMEA());
    }

    // Writing data to file
    data.print(mstime); data.print(",");
    if (GPS.fix) {
        data.print(GPS.latitudeDegrees, 6); data.print(",");
        data.print(GPS.longitudeDegrees, 6); data.print(",");
        data.print((int32_t)GPS.satellites); data.print(",");
        data.print(GPS.speed, 3); data.print(",");
        data.print(GPS.angle, 3); data.print(",");
        data.print(GPS.altitude, 3); data.print(",");
    } else {
        data.print("-1,No fix,-1,No fix,0,-1,-1,-1,");
    }
    // Sensor data
    data.print(bno_orientationw, 5); data.print(",");
    data.print(bno_orientationx, 5); data.print(",");
    data.print(bno_orientationy, 5); data.print(",");
    data.print(bno_orientationz, 5); data.print(",");
    data.print(bno_accx, 4); data.print(",");
    data.print(bno_accy, 4); data.print(",");
    data.print(bno_accz, 4); data.print(",");
    data.print(adxl_accx, 2); data.print(",");
    data.print(adxl_accy, 2); data.print(",");
    data.print(adxl_accz, 2); data.print(",");
    data.print(bmp_press, 6); data.print(",");
    data.print(bmp_alt, 4); data.print(",");
    data.print(lsm_temp, 2); data.print(",");
    data.print(adxl_temp, 2); data.print(",");
    data.print(bno_temp, 2); data.print(",");
    data.print(bmp_temp, 2); data.println();
    data.flush();

    delay(50);

    // Debugging information
    Serial.print(F("Longitude: ")); Serial.print(GPS.longitude); Serial.print(" ");
    Serial.print(F("Latitude: ")); Serial.print(GPS.latitude); Serial.print(" ");
}