#include "main.h"
#include <Arduino.h>
#include <Adafruit_LSM9DS1.h>
//#include <Adafruit_Sensor_Calibration.h>
//include <Adafruit_Sensor_Calibration_SDFat.h>
//#include <BNO055.h>


//to-do figure out how to use the new sensors

#include <Adafruit_BNO055.h>

//#define BNO055_SAMPLERATE_DELAY_MS 10
Adafruit_LSM9DS1 lsm;

/*
bool initSensors(void) {
  if (!lsm6ds.begin_I2C(0x6B) || !lis3mdl.begin_I2C(0x1E) || !baro.begin() || !lps.begin_I2C()) {
    Serial.println(lsm6ds.begin_I2C(0x6B));
    Serial.println(lis3mdl.begin_I2C(0x1E));
    Serial.println(baro.begin());
    Serial.println(lps.begin_I2C());
    return false;
  } 
  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;

  return true;
}
*/
//V3

bool initSensors(void) {
  //bno055 = Adafruit_BNO055(19, 0x29);
  /*if (!bno055.begin() || !baro.begin()) {
    Serial.println(baro.begin());
    Serial.println(bno055.begin());
    return false;
  } */
 /*if (!bno055.begin(OPERATION_MODE_AMG) || !bmp_baro.begin_I2C(0x77)){
  Serial.println(bno055.begin(OPERATION_MODE_AMG));
  Serial.println(bmp_baro.begin_I2C(0x77));
  return false;
 }*/

  lsm = Adafruit_LSM9DS1();

  if (!baro.begin() || !lsm.begin()){
    return false;
  }



 return true;
  //bno055.write8(BNO055_ACCEL_DATA_X_LSB_ADDR, 0x0F); // change accel range to 16g
}


void setupSensors(void) {

/*
  // set lowest range
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

//  Serial.println("setup_sensors check 1");

  // set slightly above refresh rate
  lsm6ds.setAccelDataRate(LSM6DS_RATE_52_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_52_HZ);
  //lsm6ds.highPassFilter(true, LSM6DS_HPF_ODR_DIV_400);

 // Serial.println("setupSensors check 1.5");

  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  */

 // Serial.println("setup_sensors check 2");
  
  baro.setMode(MPL3115A2_ALTIMETER);
  baro.startOneShot();
  baro.setSeaPressure(baro.getPressure());

  for (int i = 0; i < 10; i++){
    baro.startOneShot();
    baro.setSeaPressure(baro.getPressure());
  }

 //bmp_baro.readAltitude(rocketConfig.getPressure());
 //bmp_baro.setOutputDataRate(BMP3_ODR_100_HZ);

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G, lsm.LSM9DS1_ACCELDATARATE_50HZ);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);



  calibrateSensors();

  for (int i = 0; i < 100; i++){
    readSensors();
    rocketState.stepTime();
    rocketState.updateDeltaT();
  }

 // Serial.println("setup_sensors check 3");
}


SdFile calfile;


void initCalibration(void){
  //if (!sd.begin(4, SPI_HALF_SPEED)) sd.initErrorHalt();

  if (!cal.begin("calibrat.dat")){
    Serial.println("Failed to initialize calibration helper");
    
  }else if (!cal.loadCalibrationFromFile()){
    Serial.println("No calibration loaded/found");
  }
 /*
  if (!cal.begin(0)){
    Serial.println("Failed to initialize calibration helper");
    while (1) { yield(); }
  }*/
  
 // cal.printSavedCalibration();
  calfile.close();

}

void calibrateSensors(void){
  cal.calibrate(accel);
  cal.calibrate(gyro);
  cal.calibrate(mag);
}