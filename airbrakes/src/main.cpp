#include <Arduino.h>
// #include <Adafruit_Sensor_Calibration.h>
// #include <Adafruit_Sensor_Calibration_SDFat.h>
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
#include <Adafruit_BMP3XX.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <SdFat.h>

// #include"maths.h"
#include "main.h"
#include "sim.h"
#include "config.h"
#include "coms.h"
#include "telemetry.h"

SdFile Config;

Adafruit_MPL3115A2 baro;
Adafruit_BMP3XX bmp_baro;

// V1
/*
Adafruit_LSM6DS33 lsm6ds;
Adafruit_LPS25 lps;
Adafruit_LIS3MDL lis3mdl;
*/
Adafruit_BNO055 bno055;

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// Adafruit_Sensor_Calibration_SDFat cal; Broken AF
sensorCalibration cal;

SF sensor_filter;

state rocketState;

brakeState airBrakeState;

controller rocketControl;

status rocketStatus;

config rocketConfig;

Adafruit_NeoPixel statusLight(1, 20, NEO_GRB + NEO_KHZ800);

// GLOBAL SENSOR VALUES

float MPL_PRESSURE;
float MPL_ALTI;
float MPL_TEMP;

float BMP_PRESSURE;
float BMP_TEMPERATURE;

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

float start_altitude;
float target_apogee = 0.0f;

float test_dt = 0.0f;
float test_dt_now = 0.0f;
float test_dt_last = 0.0f;

uint32_t RED;
uint32_t GREEN;
uint32_t BLUE;
uint32_t YELLOW;
uint32_t WHITE;

float *copyQuat; // float array to copy quaternion to RocketState

void initPins();

void setup()
{
  initPins();
  Wire.begin();
  Wire.setClock(400000UL);

  t_start = (float)micros() / 1000000.0f;

  rocketStatus.updateTime();

  Serial.begin(115200);
  // while (!Serial);
  //  Serial.println("Airbrakes!");
  delay(100);

  if (!initSensors())
  {
    handleError("UNABLE TO INIT SENSORS");
  }
  else
  {
  }

  initColors();

  statusLight.begin();
  statusLight.show();

  statusLight.setPixelColor(0, WHITE);
  statusLight.show();
  delay(1000);
  statusLight.clear();
  statusLight.show();

  rocketState.init(ROCKET);
  rocketState.reset();

  


  if (initSD() == false)
  {
    handleError("UNABLE TO INIT SD CARD");
  }

  initLogs(); // Initialize state history logs

  Serial.println("logs initialized");


  rocketControl.initBrake();
  Serial.println("brake intialized");
  rocketControl.deployBrake(0);
  Serial.println("brake set to zero");
  delay(100);
  rocketControl.deployBrake(50);
  Serial.println("brake set to 50");
  delay(1000);
  rocketControl.deployBrake(0);

  initConfig();
  Serial.println("config init");
  rocketConfig.loadConfigFromFile();

  rocketState.setMass(rocketConfig.getMass());
  rocketState.setDragCoef(rocketConfig.getDragCoef());
  rocketState.setRefArea(rocketConfig.getRefArea());

  Serial.println(rocketConfig.getDragCoef());

  Serial.print("ref area: ");
  Serial.println(rocketConfig.getRefArea());
  
  Serial.println("rocket mass: ");
  Serial.println(rocketState.getMass());


  Serial.println("config intitialized");

  airBrakeState.loadConfig(rocketConfig);

  Serial.println("config finished");

  initCalibration();

  Serial.println("Cal loaded");

  setupSensors(); // setup sensors

  Serial.println("Sensors are set up");

  
  // readSensors(); // Read sensors

  // baro.startOneShot();

  for (int i = 0; i < (int)(5.0f / ((float)STEP_TIME / 1000.0f)); i++)
  {
    readSensors();
    //rocketState.updateTime();
    rocketState.updateState();
    rocketState.stepTime();
  }

  rocketState.updateTargetApogee(rocketConfig.getTargetApogee());
  Serial.println(rocketState.getBaroTemperature());
  
  Serial.println("set altitude");

  initSim();
  Serial.println("sim mass");
  Serial.println(simState.getMass());

  rocketStatus.use_lora = false;

  statusLight.setPixelColor(0, WHITE);
  statusLight.show();

  //runTestSim();
  //delay(1000000);

  
  //delay(1000);
}

void loop()
{
  //Serial.println(rocketState.getBaroAltitude());
  //Serial.println(rocketState.getBaroPressure());
  rocketStatus.updateTime();

  readSensors();

  rocketState.updateState();

  Serial.print("Apogee");
  Serial.println(rocketState.getApogee());


  switch (rocketState.flightPhase)
  {

  case PAD:
    // if (digitalRead(USE_LORA_PIN) == HIGH) // LoRa telemetry turned on
    // rocketStatus.use_lora = true;
    // Serial.println("here we go!");

    Serial.print("pitch: ");
    Serial.println(360 / (2*PI) * sqrt(rocketState.getPitch() * rocketState.getPitch() + rocketState.getRoll() * rocketState.getRoll()));
    if (rocketStatus.t > LAUNCH_DELAY)
    {
      if ((sqrt(rocketState.getPitch() * rocketState.getPitch() + rocketState.getRoll() * rocketState.getRoll()) * 360 /(2*PI)) < 30)
      {
        rocketState.setFlightPhase(LAUNCH);
        // digitalWrite()
      }
    }
    rocketState.setGroundPressure(rocketState.getBaroPressure());
    rocketState.setGroundTemperature(rocketState.getBaroTemperature());
    rocketState.updateTargetApogee(rocketConfig.getTargetApogee());
    break;

  case LAUNCH:

    if ((((rocketStatus.t * 1000000) / (LOG_TIME_STEP * 1000000)) - ((rocketStatus.t_last * 1000000) / (LOG_TIME_STEP * 1000000))) >= 1)
    {
      rocketStatus.t_last = rocketStatus.t;

      logRocketState();
      sendRocketTelemetry();
      // logSimState();
    }

    Serial.println("flightphase pad");
    if (rocketState.getAZ() > rocketConfig.getTriggerAcceleration())
    { // If launch is detected
      rocketState.setFlightPhase(IGNITION);

      // rocketControl.deployBrake(0);
    }
    //rocketState.setGroundAltitude(rocketState.getBaroAltitude());
    // Serial.println(rocketState.getAZ());
    rocketState.setGroundPressure(rocketState.getBaroPressure());
    rocketState.setGroundTemperature(rocketState.getBaroTemperature());
    rocketState.updateTargetApogee(rocketConfig.getTargetApogee());
    //Serial.println(rocketState.getTargetApogee());
    break;

  case IGNITION:
    if ((rocketState.time) > BURN_TIME)
    {
      rocketState.setFlightPhase(COAST);
    }
    if (((rocketStatus.t * 1000000) / (LOG_TIME_STEP * 1000000) - ((rocketStatus.t_last * 1000000) / (LOG_TIME_STEP * 1000000))) >= 1)
    {
      rocketStatus.t_last = rocketStatus.t;
      logRocketState();
      sendRocketTelemetry();
    }

    break;

  case COAST:

    // Needless to say, this bit could use some work.
    if (rocketState.getApogee() >= rocketState.getTargetApogee())
    {
      #ifdef DYNAMIC_DRAG
      statusLight.setPixelColor(0, RED);
      statusLight.show();
  
      if (airBrakeState.getPercentDeployed() < 75){
        rocketControl.deployBrake(airBrakeState.getPercentDeployed() + 5);
      }
      statusLight.setPixelColor(0, YELLOW);
      statusLight.show();
      #else
      statusLight.setPixelColor(0, RED);
      statusLight.show();
      rocketControl.deployBrake(75);
      statusLight.setPixelColor(0, YELLOW);
      statusLight.show();
      #endif
    }
    else
    {
      #ifdef DYNAMIC_DRAG
     // statusLight.setPixelColor(0, RED);
     // statusLight.show();
      if (airBrakeState.getPercentDeployed() > 0){
        rocketControl.deployBrake(airBrakeState.getPercentDeployed() - 5);
      }
      //statusLight.setPixelColor(0, YELLOW);
      //statusLight.show();
      #else
      rocketControl.deployBrake(0);
      #endif
    }

    if (((rocketStatus.t * 1000000) / (LOG_TIME_STEP * 1000000) - ((rocketStatus.t_last * 1000000) / (LOG_TIME_STEP * 1000000))) >= 1)
    {
      rocketStatus.t_last = rocketStatus.t;
      logRocketState();
      sendRocketTelemetry();
    }
    if (rocketState.time > rocketConfig.getMaxTime())
    {
      Serial.println("flightphase land");
      rocketState.flightPhase = LAND;
      statusLight.setPixelColor(0, WHITE);
      statusLight.show();
      // rocketControl.deployBrake(0);
      writeRocketStateLog();
      closeLogs();

      //exit(0);
    }
    Serial.println(rocketState.getTargetApogee());


    break;

  case LAND:

    return;

    break;

  default:
    break;
  }


  if (rocketState.flightPhase != PAD && rocketState.flightPhase != LAUNCH)
  {
    updateSim();
  }

  rocketState.stepTime();
  rocketStatus.updateTime();
}

// READ SENSORS

float filter_dt;

void readSensors()
{
  #ifdef AIRBRAKE_V7

  BMP_PRESSURE = bmp_baro.pressure/100;
  BMP_TEMPERATURE = bmp_baro.temperature;

  rocketState.setBaroPressure(BMP_PRESSURE);
 
  rocketState.setBaroTemperature(BMP_TEMPERATURE);
  rocketState.baroConversionFinished = true;

  rocketState.setBaroAltitude(rocketState.calcBaroAltitude());

  //rocketState.updateAirDensity();

  bmp_baro.performReading();
  

  #else

  if (baro.conversionComplete())
  {
    MPL_PRESSURE = baro.getLastConversionResults(MPL3115A2_PRESSURE); // float, hPa
    MPL_TEMP = baro.getLastConversionResults(MPL3115A2_TEMPERATURE);

    baro.startOneShot();
    //rocketState.setBaroAltitude(rocketState.calcBaroAltitude());
    rocketState.setBaroPressure(MPL_PRESSURE);
    rocketState.setBaroTemperature(MPL_TEMP);
    rocketState.baroConversionFinished = true;
    //rocketState.setAltitude(rocketState.calcBaroAltitude());
    rocketState.setBaroAltitude(rocketState.calcBaroAltitude());

    rocketState.updateAirDensity();

    // calibrateSensors();
  }

  #endif

  

  lsm.getEvent(&accel, &mag, &gyro, &tempp);



  calibrateSensors();
  

  ACC_X = -accel.acceleration.x; // float, m/s2
  ACC_Y = accel.acceleration.y;
  ACC_Z = accel.acceleration.z;
  // flip sign when using lsm9ds1/0
  GYRO_X = -gyro.gyro.x; // float, rad/s
  GYRO_Y = gyro.gyro.y;
  GYRO_Z = gyro.gyro.z;

  MAG_X = mag.magnetic.x;
  MAG_Y = mag.magnetic.y;
  MAG_Z = mag.magnetic.z;
  rocketState.setAX_Local(ACC_X);
  rocketState.setAY_Local(ACC_Y);
  rocketState.setAZ_Local(ACC_Z);


  // Use Madgwick filter to update sensor filter

  filter_dt = sensor_filter.deltatUpdate();

  sensor_filter.MadgwickUpdate(GYRO_X, GYRO_Y, GYRO_Z, ACC_X, ACC_Y, ACC_Z, MAG_X, MAG_Y, MAG_Z, filter_dt); // Use Madgwick filter to update sensor filter

  // Set RocketState quaternion to sensor_filter quaternion
  copyQuat = sensor_filter.getQuat();

  rocketState.setQuatW(copyQuat[0]);
  rocketState.setQuatX(copyQuat[1]);
  rocketState.setQuatY(copyQuat[2]);
  rocketState.setQuatZ(copyQuat[3]);
}

void brakeTest()
{
  Serial.println("brakeTest check 1");
  retractBrake();
  delay(500);
  deployBrake();
  delay(500);
  retractBrake();
  delay(500);
  Serial.println("brakeTest check 2");
}

void state::updateDeltaT()
{
  now = micros();
  delta_t = (float)((now - last_time) / 1000000.0f);
  last_time = now;
}

void state::stepTime()
{
  delay(STEP_TIME);
}

void state::updateTime()
{
  if (rocketState.flightPhase != PAD && rocketState.flightPhase != LAUNCH)
    time = (float)micros() / 1000000.0f - t_launch;
  else
    time = 0;

  updateDeltaT();
}


void state::setFlightPhase(phase flightPhase){
  this->flightPhase = flightPhase;
  switch (flightPhase){
    case PAD:
      statusLight.setPixelColor(0, WHITE);
      statusLight.show();
      Serial.println("FLIGHTPHASE: PAD");
      break;

    case LAUNCH:
      statusLight.setPixelColor(0, GREEN);
      statusLight.show();
      Serial.println("FLIGHTPHASE: LAUNCH");
      break;

    case IGNITION:
      statusLight.setPixelColor(0, BLUE);
      statusLight.show();
      rocketState.t_launch = rocketStatus.t;
      Serial.println("FLIGHTPHASE: IGNITION");
      break;

    case LAND:
      statusLight.setPixelColor(0, WHITE);
      statusLight.show();
      Serial.println("FLIGHTPHASE: LAND");
      break;

    case ERROR:
      statusLight.setPixelColor(0, RED);
      statusLight.show();
      Serial.println("FLIGHTPHASE: ERROR");
      break;

    default:
      break;

  }
}

void status::updateTime()
{
  t = (float)(micros()) / 1000000.0f;
}

void initPins()
{
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(USE_LORA_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void initColors()
{
  RED = statusLight.Color(255, 0, 0);
  GREEN = statusLight.Color(0, 255, 0);
  BLUE = statusLight.Color(0, 0, 255);
  YELLOW = statusLight.Color(255, 255, 0);
  WHITE = statusLight.Color(255, 255, 255);
}


void handleError(char *errormsg){
  if (strlen(errormsg) < 0){
    handleError("LEN OF ERROR MSG IS ZERO");
  } 
  if (errorLog.isOpen()){
    logError(errormsg);
  }
  Serial.println(errormsg);
  rocketState.setFlightPhase(ERROR);
  while (1)
    delay (10);
}

void state::init(statetype stateType){
  this->stateType = stateType;
}