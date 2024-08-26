// SRAD Avionics Flight Computer Software for AIAA UH
//
// Copyright (c) 2024 Dedah (www.github.com/UH-AIAA)
// Licensed under the MIT License
//
// More information on the MIT license as well as a complete copy
// of the license can be found here: https://choosealicense.com/licenses/mit/
// All above text must be included in any restribution.


#include <Wire.h>
#include <SPI.h>
#include <SD.h>


#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>


  // defining variables

  const uint32_t PIN_BUZZER = 33;
  const uint32_t PIN_LED = 29;
  const uint32_t LSM_CS = 40;
  const uint32_t BMP_CS = 41;
  const uint32_t ADXL_CS = 39;
  int32_t mstime;

// assigning id to sensors

    Adafruit_LSM6DSO32 LSM;
    Adafruit_BMP3XX BMP;
    Adafruit_ADXL375 ADXL(ADXL_CS, &SPI, 12345);
    Adafruit_BNO055 BNO(55, 0x28, &Wire);
    Adafruit_GPS gps(&Serial2);

//Data Variable assignments

  // Sensor raw data
    double lsm_gyrox, lsm_gyroy, lsm_gyroz, lsm_accx, lsm_accy, lsm_accz;
    double adxl_accx, adxl_accy, adxl_accz;

    double bno_gyrox, bno_gyroy, bno_gyroz, bno_accx, bno_accy, bno_accz;
    double bno_orientationx, bno_orientationy, bno_orientationz, bno_orientationw;

    // Inertia data, angular_rate acceleration_body, acceleration_inertial; magnetic_flux; euler_angles; angular_rate;

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
  //"state,"
  //"eul_x,eul_y,eul_z,"
  //"q_w,q_x,q_y,q_z,"
  "bno ori w,bno ori x,bno ori y,bno ori z," //9
  "bno_rate_x,bno_rate_y,bno_rate_z,"
  "bno_accel_x,bno_accel_y,bno_accel_z,"
  "adxl_accel_x,adxl_accel_y,adxl_accel_z,"
  "pressure,altitude,"
  "lsm_temp,adxl_temp,bno_temp,bmp_temp";

// Sensor read functions
  bool read_LSM()
  {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    if(!LSM.getEvent(&accel, &gyro, &temp))
    {
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
bool read_BMP()
{
  if(!BMP.performReading())
  {
    return false;
  }
  bmp_temp = BMP.temperature;
  bmp_press = BMP.pressure;
  bmp_alt = BMP.readAltitude(1013.25) - off_alt;
  return true;
}
bool read_ADXL()
{
  sensors_event_t event;
  if(!ADXL.getEvent(&event))
  {
    return false;
  }

  adxl_accx = event.acceleration.x;
  adxl_accy = event.acceleration.y;
  adxl_accz = event.acceleration.z;

  adxl_temp = float(event.temperature);

  return true;
}
bool read_BNO()
{
  sensors_event_t orientationData , angVelocityData, magnetometerData, accelerometerData;

  if(!BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER))
  {
    return false;
  }
  if(!BNO.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE))
  {
    return false;
  }
  if(!BNO.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER))
  {
    return false;
  }
  if(!BNO.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER))
  {
    return false;
  }

  imu::Quaternion quat = BNO.getQuat();
  bno_orientationw = quat.w();
  bno_orientationx = quat.x();
  bno_orientationy = quat.y();
  bno_orientationz = quat.z();
  //bno_orientation *= Quaternion().euler_to_quaternion(Vector3(0, 0, (90 + 20) * DEG_TO_RAD));

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
  // Radio Serial Port
  Serial1.begin(9600);
  // gps Serial Port
  Serial2.begin(115200);

// Configure LSM6DSO32
  {
    LSM.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
   LSM.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
   LSM.setAccelDataRate(LSM6DS_RATE_416_HZ);
    LSM.setGyroDataRate(LSM6DS_RATE_416_HZ);
  }
// Configure BMP390
{
  BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  BMP.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  BMP.setOutputDataRate(BMP3_ODR_200_HZ);
}

// Configure ADXL
ADXL.setDataRate(ADXL343_DATARATE_200_HZ);

// Configure BNO055
{
  BNO.setMode(OPERATION_MODE_CONFIG);
  BNO.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
  BNO.setMode(OPERATION_MODE_NDOF);
}
// gps Initialization and Configuration
{
  gps.begin(115200);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
}
//Create datalogging file
char dataname[17] = "FL0.csv";
for(int i = 0; SD.exists(dataname); i++){
  sprintf(dataname, "FL%d.csv", i);
}

//Open file for writing
data = SD.open(dataname, FILE_WRITE);

//Print data header
data.println(data_header);
data.flush();
}

void loop() {
    mstime = millis();
   // Read Sensors
    read_ADXL();
    read_BMP();
    read_BNO();
    read_LSM();
//Update gps
  while(gps.available())
    {
      gps.read();
    }

      if(gps.newNMEAreceived())
      {
        gps.parse(gps.lastNMEA());
      }

 // writing to file

 //data.write(const char *buffer, size_t size)
 //{
  data.print(mstime);                  data.print(",");

  data.print(gps.latitudeDegrees, 6);
  data.print(gps.lat);                 data.print(",");
  data.print(gps.longitudeDegrees, 6);
  data.print(gps.lon);                 data.print(",");

  data.print((int32_t)gps.satellites); data.print(",");
  data.print(gps.speed, 3);            data.print(",");
  data.print(gps.angle, 3);            data.print(",");
  data.print(gps.altitude, 3);         data.print(",");

  //from gps, needs work
  //data.print(euler_angles.x, 2);       data.print(",");
  //data.print(euler_angles.y, 2);       data.print(",");
  //data.print(euler_angles.z, 2);       data.print(",");

  //from gps orientation 4

  data.print(bno_orientationw, 5);    data.print(",");
  data.print(bno_orientationx, 5);    data.print(",");
  data.print(bno_orientationy, 5);    data.print(",");
  data.print(bno_orientationz, 5);    data.print(",");

  data.flush();
  //from gps, angrate, acc_inertia, acc_body 9

  data.print(bno_accx, 4);       data.print(",");
  data.print(bno_accy, 4);       data.print(",");
  data.print(bno_accz, 4);       data.print(",");

  data.print(adxl_accx, 2);       data.print(",");
  data.print(adxl_accy, 2);       data.print(",");
  data.print(adxl_accz, 2);       data.print(",");

  data.print(bmp_press, 6);    data.print(",");
  data.print(bmp_alt, 4);      data.print(",");  //22
  data.print(v_vel, 4);        data.print(",");

  data.print(lsm_temp, 2);       data.print(",");
  data.print(adxl_temp, 2);      data.print(",");
  data.print(bno_temp, 2);       data.print(",");
  data.print(bmp_temp, 2);       data.print(",");  //27

  data.println();
  data.flush();

  delay(50);
// }
Serial.print(F("Longitute "));
Serial.print((gps.longitude));
Serial.print(" ");

Serial.print(F("Latitude "));
Serial.print((gps.latitude));
Serial.print("  ");


Serial.print(F(" # Sats "));
Serial.print((gps.satellites));
Serial.println("");

}
