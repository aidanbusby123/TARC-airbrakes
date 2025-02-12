#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_Sensor_Calibration_SDFat.h>
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
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <SdFat.h>

//#include"maths.h"
#include"main.h"
#include"sim.h"
#include "config.h"

#define TEST_TIME 20.0f
#define START_TIME 20.0f

SdFile Config;

Adafruit_MPL3115A2 baro;

//V1


Adafruit_LSM6DS33 lsm6ds;
Adafruit_LPS25 lps;
Adafruit_LIS3MDL lis3mdl;

Adafruit_BNO055 bno055;

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

Adafruit_Sensor_Calibration_SDFat cal;

SF sensor_filter;


state rocketState;

brakeState airBrakeState;

controller rocketControl;

status simStatus;

config rocketConfig;


stateHistory* rocketStateHistory; // Rocket State history
stateHistory* simStateHistory; // Simulation state history

uint rocketStateHistory_index = 0;
uint rocketStateHistory_size = 0;

uint simStateHistory_index = 0;
uint simStateHistory_size = 0;
// GLOBAL SENSOR VALUES

float MPL_PRESSURE;
float MPL_ALTI;
float MPL_TEMP;

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

float t = 0.0f;
float t_start = 0.0f;
float t_launch = 0.0f;
float t_last = 0.0f;
float dt = 0;

float test_dt = 0.0f;
float test_dt_now = 0.0f;
float test_dt_last = 0.0f;


float *copyQuat; // float array to copy quaternion to RocketState



void setup() {
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.begin();
  Wire.setClock( 400000UL);
  
  
  t_start = (float)micros()/1000000.0f;

  Serial.begin(115200);
     while(!Serial);
  //Serial.println("Airbrakes!");
  delay(100);
 /* 
  if (!initSensors()){
    Serial.println("Failed to initialize sensors!");
  } else {
    Serial.println("Worked!");
  }*/

  rocketState.stateType = ROCKET;
  Serial.println("State type established");
  // Initialize rocket state history


  Serial.println("rocketStateHistory created");
 // initSD();
  Serial.println("SD initialized");

  //initLogs(); // Initialize state history logs

  Serial.println("Logs initialized");
  //initBT();

  //rocketControl.initBrake();
  //rocketControl.deployBrake(BRAKE_RETRACTED);

  //initFlash(); // Initialize system flash

  //initCalibration();
 //magnetometer->printSensorDetails();

 // setupSensors(); // setup sensors
  //delay(10000);

  /*for (;;){
    brakeTest(); // Test airbrake
  }*/
  
 // readSensors(); // Read sensors
  //rocketState.setAltitude(baro.getAltitude());

  while (1){

    runTestSim();
  }
  delay(10000000000000);
  exit(0);


  //delay(1000);
  

}


void loop() {
  t = (float)micros()/1000000.0f; 
  
  
  readSensors();
  //Serial.print(rocketState.getAZ());

  
 // Serial.println("readSensors complete");
  /*dt = (float)(micros())/1000000.0f - t;
  rocketState.delta_t = dt;
  rocketState.time = (float)micros()/1000000.0f;
*/
  rocketState.updateDeltaT();
  rocketState.updateState();

  // If time is past start time (time for the rocket to get set up)

  if (rocketState.flightPhase == PAD){
    //Serial.println("here we go!");

    while (rocketState.flightPhase == LAUNCH){ // While rocket flight phase is LAUNCH, continue to read sensors until moving.
        
  
      readSensors(); // get sensor input

      if ((((rocketState.time * 1000000)/(LOG_TIME_STEP * 1000000)) -((t_last * 1000000)/(LOG_TIME_STEP * 1000000))) >= 1){
          t_last = rocketState.time;
          //Serial.println("logging");
          logRocketState();
          //logSimState();
          //logState(simState);
        }
  
      // Serial.println("readSensors complete");

      rocketState.updateDeltaT();
      rocketState.updateState();

      simState.time = (float)micros()/1000000.0f;
      

      //Serial.println("flightphase pad");
      if (rocketState.getAZ() > TRIGGER_ACCEL){ // If launch is detected
        Serial.println("flighphase launch");
        rocketState.flightPhase = IGNITION;
        t_launch = rocketState.time;
        //brake.write(60);

      }
    }
    if (rocketState.flightPhase == IGNITION || rocketState.flightPhase == COAST){

      updateSim();

      //Needless to say, this bit could use some work.
      if (simStatus.apogee >= TARGET_APOGEE){
        rocketControl.deployBrake(100);
      } else {
        rocketControl.deployBrake(0);
      }
      //Serial.println("logging");
      if (rocketState.time > BURN_TIME)
        rocketState.flightPhase = COAST;
        
      if ((((rocketState.time * 1000000)/100000) -((t_last * 1000000)/100000)) >= 1){
        t_last = t;
        //Serial.println("logging");
        logRocketState();
        //logSimState();
      }
      if (simState.time > TEST_TIME){
        rocketState.flightPhase = LAND;
        writeRocketStateLog();
        closeLogs();
        Serial.println("closed logs");
        exit(0);
      }
    }
  if (rocketState.flightPhase == LAND){
    return;
  }
    }
 /*Serial.print(rocketState.getBaroAltitude());
 Serial.print(", ");
 Serial.println(rocketState.getAltitude());*/
/*
 Serial.print(rocketState.getAX());
 Serial.print(", ");
 Serial.print(rocketState.getAY());
 Serial.print(", ");
 Serial.println(rocketState.getAZ());
 */
 //loopBT();
 
/*
  Serial.print("Quaternion: ");
  Serial.print(rocketState.getQuatW(), 4);
  Serial.print(", ");
  Serial.print(rocketState.getQuatX(), 4);
  Serial.print(", ");
  Serial.print(rocketState.getQuatY(), 4);
  Serial.print(", ");
  Serial.println(rocketState.getQuatZ(), 4); 
  //Serial.println(" ");

  Serial.print("Acceleration: ");
  Serial.print(rocketState.getAX());
  Serial.print(", ");
  Serial.print(rocketState.getAY());
  Serial.print(", ");
  Serial.println(rocketState.getAZ());
*/



/* Serial.print(rocketState.getAX());
  Serial.print(", ");
  Serial.print(rocketState.getAY());
  Serial.print(", ");
  Serial.println(rocketState.getAZ());
*/
/*if (t <= 5000000.0f){
  rocketState.flightPhase = PAD;
 // Serial.println("Flighphase: PAD");
}
  else rocketState.flightPhase = LAUNCH;
  */
/*
  if (rocketState.flightPhase == PAD){
    cal.gyro_zerorate[0] = GYRO_X;
    cal.gyro_zerorate[1] = GYRO_Y;
    cal.gyro_zerorate[2] = GYRO_Z;
  }
  */
 //Serial.println(rocketState.getAltitude()-80.00);



  
 // Serial.println(dt, 6);
}



// READ SENSORS

float filter_dt;

void readSensors(){
  
  if (!baro.conversionComplete()){

  /*V1 Code*/

  sensors_event_t temp;
  sensors_event_t pressure;
  //lps.getEvent(&pressure, &temp);// get pressure

  lsm6ds.getEvent(&accel, &gyro, &tempp);

  lis3mdl.getEvent(&mag);



  // updated every 40ms (25 hz)
  // fast-updating pressure and temp data
  //LPS_PRESSURE = pressure.pressure; // float, hPa
  //LPS_TEMP = temp.temperature;      // float, C
  
  // updated ~1KHz
  // acceleration and gyro data

  /*V3 code
  bno055.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno055.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno055.getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);
*/
  ACC_X = accel.acceleration.x;  //float, m/s2
  ACC_Y = accel.acceleration.y;
  ACC_Z = accel.acceleration.z;

  GYRO_X = gyro.gyro.x;          // float, rad/s
  GYRO_Y = gyro.gyro.y;
  GYRO_Z = gyro.gyro.z;

  MAG_X = mag.magnetic.x;
  MAG_Y = mag.magnetic.y;
  MAG_Z = mag.magnetic.z;

  

  calibrateSensors();

  }

  else{
    // Updated every ~1 second (1 hz)
    // calibrated altitude data
    MPL_PRESSURE = baro.getLastConversionResults(MPL3115A2_PRESSURE);  // float, hPa
    MPL_ALTI = baro.getLastConversionResults(MPL3115A2_ALTITUDE);      // float, m
    MPL_TEMP = baro.getLastConversionResults(MPL3115A2_TEMPERATURE);
    baro.startOneShot();

    rocketState.setBaroAltitude(MPL_ALTI);
    rocketState.setBaroPressure(MPL_PRESSURE);
    rocketState.setBaroTemperature(MPL_TEMP);
    //Serial.println(MPL_ALTI);

    //Serial.println(micros() - dT_baro);
   calibrateSensors();
 /* Serial.print(copyQuat[0]);
  Serial.print(", ");
  Serial.print(copyQuat[1]);
  Serial.print(", ");
  Serial.print(copyQuat[2]);
  Serial.print(", ");
  Serial.println(copyQuat[3]);
    */
  }

  
  rocketState.setAX_Local(ACC_X);
  rocketState.setAY_Local(ACC_Y);
  rocketState.setAZ_Local(ACC_Z);

  // Use Madgwick filter to update sensor filter

filter_dt = sensor_filter.deltatUpdate();
 
sensor_filter.MadgwickUpdate(GYRO_X, GYRO_Y, GYRO_Z, ACC_X, ACC_Y, ACC_Z, MAG_X, MAG_Y, MAG_Z, filter_dt); // Use Madgwick filter to update sensor filter

   //Set RocketState quaternion to sensor_filter quaternion
 copyQuat = sensor_filter.getQuat();

rocketState.setQuatW(copyQuat[0]);
rocketState.setQuatX(copyQuat[1]);
rocketState.setQuatY(copyQuat[2]);
rocketState.setQuatZ(copyQuat[3]);



}


void brakeTest(){
  Serial.println("brakeTest check 1");
  retractBrake();
  delay(500);
  deployBrake();
  delay(500);
  retractBrake();
  delay(500);
  Serial.println("brakeTest check 2");
}

void state::updateDeltaT(){
  Now =  micros();
  delta_t = (float)((Now - lastTime)/1000000.0f);
  lastTime = Now;
}
