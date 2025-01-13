#include "main.h"
#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_Sensor_Calibration_SDFat.h>

bool initSensors(void) {
  if (!lsm6ds.begin_I2C(0x6B) || !lis3mdl.begin_I2C(0x1E) || !baro.begin() || !lps.begin_I2C()) {
    return false;
  } 
  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;

  return true;
}

void setupSensors(void) {
  // set lowest range
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  Serial.println("setup_sensors check 1");

  // set slightly above refresh rate
  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);

  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  Serial.println("setup_sensors check 2");

  baro.setSeaPressure(1013.26);  
  baro.setMode(MPL3115A2_ALTIMETER);
  

  calibrateSensors();

  Serial.println("setup_sensors check 3");
}

SdFat sd;
SdFile calfile;

void initCalibration(void){
  if (!sd.begin(4, SPI_HALF_SPEED)) sd.initErrorHalt();

  if (!calfile.open("calibrat.dat", O_RDWR | O_CREAT)){
    sd.errorHalt("opening sensor calibration failed");
  }
  if (!cal.begin("calibrat.dat", dynamic_cast<FatFileSystem*>(&sd))){
    Serial.println("Failed to initialize calibration helper");
    while (1) { yield(); }
  }
  if (!cal.loadCalibration()){
    Serial.println("No calibration loaded/found");
  }
  cal.printSavedCalibration();

}

void calibrateSensors(void){
  cal.calibrate(accel);
  cal.calibrate(gyro);
  cal.calibrate(mag);
}