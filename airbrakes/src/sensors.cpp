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

//Adafruit_BMP3XX bmp_baro;

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

  #ifdef AIRBRAKE_V7

  if (!bmp_baro.begin_I2C()){
    handleError("can't init BMP baro");
    return false;
  }

  

  #else

  if (!baro.begin()){
    handleError("Can't init barometer");
    return false;
  }

  #endif

  if (!lsm.begin()){
    handleError("Can't init accelerometer");
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
  #ifndef AIRBRAKE_V7
  baro.setMode(MPL3115A2_BAROMETER);
  #endif

 /* for (int i = 0; i < 10; i++){
    baro.startOneShot();
    while (!baro.conversionComplete())
      delay(10);
    
  }*/

  //baro.setSeaPressure(890.3);

 //bmp_baro.readAltitude(rocketConfig.getPressure());
 //bmp_baro.setOutputDataRate(BMP3_ODR_100_HZ);

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G, lsm.LSM9DS1_ACCELDATARATE_119HZ);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);



  calibrateSensors();

  #ifdef AIRBRAKE_V7

  bmp_baro.setOutputDataRate(BMP3_ODR_200_HZ);
  bmp_baro.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp_baro.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  
  for (int i = 0; i < 1000; i++){
    rocketState.setGroundPressure(bmp_baro.readPressure());
    rocketState.setGroundTemperature(bmp_baro.readTemperature());
    rocketState.updateState();
    rocketState.stepTime();
  }
  #else

  for (int i = 0; i < 10; i++){
    baro.startOneShot();
    while (!baro.conversionComplete())
      delay(10);
    rocketState.setGroundPressure(baro.getLastConversionResults(MPL3115A2_PRESSURE));
    rocketState.setBaroTemperature(baro.getLastConversionResults(MPL3115A2_TEMPERATURE));
    rocketState.updateState();
    rocketState.stepTime();
  }

 // Serial.println("setup_sensors check 3");
  baro.startOneShot();

  #endif
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

