
#include <Arduino.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_HTS221.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_MLX90393.h>
#include <Adafruit_MMC56x3.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_MSA301.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

//#include"maths.h"
#include"main.h"

Adafruit_MPL3115A2 baro;
Adafruit_LSM6DS33 lsm6ds;
Adafruit_LPS25 lps;
Adafruit_LIS3MDL lis3mdl;

Servo brake;

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

SF sensor_filter;


state RocketState;
// GLOBAL SENSOR VALUES

float MPL_PRESSURE;
float MPL_ALTI;

float LPS_PRESSURE;
float LPS_TEMP;

float ACC_X = 0.0f;
float ACC_Y = 0.0f;
float ACC_Z = 0.0f;

float GYRO_X = 0.0f;
float GYRO_Y = 0.0f;
float GYRO_Z = 0.0f;

float MAG_X = 0.0f;
float MAG_Y = 0.0f;
float MAG_Z = 0.0f;


sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t tempp;

long t = 0;
float dt = 0;


float *copyQuat; // float array to copy quaternion to RocketState


void setup() {
  digitalWrite(13, HIGH);

  Wire.begin();
  Wire.setClock( 400000UL);
  
  Serial.begin(115200);
  while(!Serial);
  //Serial.println("Airbrakes!");

  if (!init_sensors()){
    Serial.println("Failed to initialize sensors!");
  }

  brake.attach(9); // attach airbrake

  setup_sensors(); // setup sensors


  brakeTest(); // Test airbrake
 
  
  readSensors(); // Read sensors
}


void loop() {
  t = micros();
  readSensors();

  RocketState.updateState();

  Serial.print(RocketState.getAX());
  Serial.print(", ");
  Serial.print(RocketState.getAY());
  Serial.print(", ");
  Serial.println(RocketState.getAZ());

  delay(10);

  dt = (float)(micros()-t)/1000000.0f;
}



// SERVO FUNCTIONS

void deployBrake(){
  brake.write(0);
}

void retractBrake(){
  brake.write(130);
}



// READ SENSORS


long dT_baro = 0;

float filter_dt;

  


void readSensors(){
  
  if (!baro.conversionComplete()){
  sensors_event_t temp;
  sensors_event_t pressure;
  lps.getEvent(&pressure, &temp);// get pressure


  lsm6ds.getEvent(&accel, &gyro, &tempp);

  lis3mdl.getEvent(&mag);

  

  // updated every 40ms (25 hz)
  // fast-updating pressure and temp data
  LPS_PRESSURE = pressure.pressure; // float, hPa
  LPS_TEMP = temp.temperature;      // float, C
  
  // updated ~1KHz
  // acceleration and gyro data
  ACC_X = accel.acceleration.x;  //float, m/s2
  ACC_Y = accel.acceleration.y;
  ACC_Z = accel.acceleration.z;

  GYRO_X = gyro.gyro.x;          // float, rad/s
  GYRO_Y = gyro.gyro.y;
  GYRO_Z = gyro.gyro.z;

  MAG_X = mag.magnetic.x;
  MAG_Y = mag.magnetic.y;
  MAG_Z = mag.magnetic.z;

  }

  else{
    // Updated every ~1 second (1 hz)
    // calibrated altitude data
    MPL_PRESSURE = baro.getLastConversionResults(MPL3115A2_PRESSURE);  // float, hPa
    MPL_ALTI = baro.getLastConversionResults(MPL3115A2_ALTITUDE);      // float, m
    baro.startOneShot();

    //Serial.println(micros() - dT_baro);
    
    
  }

  // Use Madgwick filter to update sensor filter

  filter_dt = sensor_filter.deltatUpdate();
  sensor_filter.MadgwickUpdate(GYRO_X, GYRO_Y, GYRO_Z, ACC_X, ACC_Y, ACC_Z, MAG_X, MAG_Y, MAG_Z, filter_dt); // Use Madgwick filter to update sensor filter

  // Set RocketState quaternion to sensor_filter quaternion
  copyQuat = sensor_filter.getQuat();
  RocketState.setQuatW(copyQuat[0]);
  RocketState.setQuatX(copyQuat[1]);
  RocketState.setQuatY(copyQuat[2]);
  RocketState.setQuatZ(copyQuat[3]);

  Serial.print(RocketState.getQuatW());
  Serial.print(", ");
  Serial.print(RocketState.getQuatX());
  Serial.print(", ");
  Serial.print(RocketState.getQuatY());
  Serial.print(", ");
  Serial.println(RocketState.getQuatZ());


}

float V_Z = 0.0f;
float ALTITUDE = 0.0f;


long uS_dT = 0;

/*Quaternion DeltaRotQuaternion; // Quaternion to describe change in rotation angle from gyroscope data.
Quaternion TempQuaternion; // Temprorary quaternion
*/



/*void updateState(){

  // Get filtered quaternion
  copyQuat = sensor_filter.getQuat();
  RocketState.setQuatW(copyQuat[0]);
  RocketState.setQuatX(copyQuat[1]);
  RocketState.setQuatY(copyQuat[2]);
  RocketState.setQuatZ(copyQuat[3]);





  RocketState.AccelLocal.setX(ACC_X);
  RocketState.AccelLocal.setY(ACC_Y);
  RocketState.AccelLocal.setZ(ACC_Z);


  // Update rocket rotation quaternions


  DeltaRotQuaternion = Quaternion(0, GYRO_X, GYRO_Y, GYRO_Z); // Rotation quaternion for change in rotation angle, using gyroscope
  
  TempQuaternion = RocketState.RotQuaternion + (((DeltaRotQuaternion * RocketState.RotQuaternion)) * 0.5) * DeltaT; // Update rotation quaternion, q_{i+1} = q_{i} + q_{gyro}*q_{i}* dt/2 
  RocketState.RotQuaternion = TempQuaternion.normalize(); // Normalize the temporary quaternion



  // Convert local acceleration to global acceleration

  RocketState.AccelLocalQuaternion.setX(RocketState.AccelLocal.getX());
  RocketState.AccelLocalQuaternion.setY(RocketState.AccelLocal.getY());
  RocketState.AccelLocalQuaternion.setZ(RocketState.AccelLocal.getZ());

  RocketState.AccelQuaternion = (RocketState.RotQuaternion * RocketState.AccelLocalQuaternion) * RocketState.RotQuaternion.conjugate();
  */


  //RocketState.Vel_Local.set_z((ACC_Z-ACC_Z_ZERO) * ((float) dT / 1000000.0));
  //ALTITUDE = 0.9 * (ALTITUDE + V_Z * ((float) dT / 1000000.0) + 1/2 * (ACC_Z-ACC_Z_ZERO) * pow((float) dT / 1000000.0, 2)) + 0.1 * MPL_ALTI;


 
  /*Serial.print("DeltaT: ");
  Serial.println(DeltaT);
*/
/*
  Serial.print("Gyro: ");
  Serial.print(DeltaRotQuaternion.getX());
  Serial.print(", ");
  Serial.print(DeltaRotQuaternion.getY());
  Serial.print(", ");
  Serial.print(DeltaRotQuaternion.getZ());
  Serial.print(", ");
  */
  /*Serial.print("Quaternion:");
  Serial.print(RocketState.RotQuaternion.getX());
  Serial.print(", ");
  Serial.print(RocketState.RotQuaternion.getY());
  Serial.print(", ");
  Serial.println(RocketState.RotQuaternion.getZ());
*/
/*  Serial.print(GYRO_X, 5);
  Serial.print(", ");
  Serial.print(GYRO_Y, 5);
  Serial.print(", ");
  Serial.print(GYRO_Z, 5);
  Serial.print(", ");
  Serial.print(ACC_X, 5);
  Serial.print(", ");
  Serial.print(ACC_Y, 5);
  Serial.print(", ");
  Serial.println(ACC_Z, 5);
  //Serial.print(",BAR:");
  //Serial.println(MPL_ALTI);
  */
/*
  Serial.print(RocketState.AccelQuaternion.getX());
  Serial.print(", ");
  Serial.print(RocketState.AccelQuaternion.getY());
  Serial.print(", ");
  Serial.println(RocketState.AccelQuaternion.getZ());
  */
  //uS_dT = micros();
//}

void brakeTest(){
  
  retractBrake();
  delay(500);
  deployBrake();
  delay(500);
  retractBrake();
}