#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>


Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"

Adafruit_MPL3115A2 baro;
Adafruit_LPS25 lps;

Servo brake;


// pick your filter! slower == better quality output
//Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;  // faster than NXP
Adafruit_Mahony filter;  // fastest/smalleset


#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif


#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 1
//#define AHRS_DEBUG_OUTPUT


uint32_t timestamp;

// Quaternion
float qw, qx, qy, qz;

float gravity;
float vertical_accel, vertical_velocity, altitude;
unsigned long previousMicros = 0;
const float dt = 0.01;

float MPL_PRESSURE;
float MPL_ALTI;

float LPS_PRESSURE;
float LPS_TEMP;

long dT_baro = 0;

float ACC_X = 0;
float ACC_Y = 0;
float ACC_Z = 0;

float GYRO_X = 0;
float GYRO_Y = 0;
float GYRO_Z = 0;

/*
float ACC_X_ZERO = 0;
float ACC_Y_ZERO = 0;
float ACC_Z_ZERO = 0;
*/






void setup() {
  Serial.begin(115200);
  while (!Serial) yield();

  // Connect to peripherals
  
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }

  if (!lps.begin_I2C()) {
    Serial.println("Failed to find LPS25 chip");
    while (1) { delay(10); }
  }

  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  if (!lsm6ds.begin_I2C(0x6B)) {
    // if (!lsm6ds33.begin_SPI(LSM_CS)) {
    // if (!lsm6ds33.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }  
  }

  brake.attach(9); // Servo


  // Sensor setup
  
  lps.setDataRate(LPS25_RATE_25_HZ);


  // use to set sea level pressure for current location
  // this is needed for accurate altitude measurement
  // STD SLP = 1013.26 hPa
  baro.setSeaPressure(1013.26);

  baro.setMode(MPL3115A2_ALTIMETER);
  baro.startOneShot();

  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();
  

  Wire.setClock(400000); // 400KHz

  
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  
  gravity = accel.acceleration.z;

  retractBrake();
  delay(500);
  deployBrake();
  delay(500);
  retractBrake();
  delay(500);
}


void loop() {

  //===============GET ORIENTATION QUATERNION====================
  
  float roll, pitch, heading;
  float gx, gy, gz;
  static uint8_t counter = 0;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0;

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.println("");
#endif

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.println(roll);

  
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  Serial.print("Quaternion: ");
  Serial.print(qw, 4);
  Serial.print(", ");
  Serial.print(qx, 4);
  Serial.print(", ");
  Serial.print(qy, 4);
  Serial.print(", ");
  Serial.println(qz, 4);  

  
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif


//=======================BARO ALTI==================
  
  if(baro.conversionComplete()){
    // Updated every ~1 second (1 hz)
    // calibrated altitude data
    MPL_PRESSURE = baro.getLastConversionResults(MPL3115A2_PRESSURE);  // float, hPa
    MPL_ALTI = baro.getLastConversionResults(MPL3115A2_ALTITUDE);      // float, m
    baro.startOneShot();

    //Serial.println(micros() - dT_baro);

    dT_baro = micros();
    
    
  }

  //====================GET VERTICAL ACCELERATION===============

  // Step 1: Apply the quaternion to rotate the accelerometer data
  float rotated_accel_z = rotateAccelerometerToVertical(accel.acceleration.x,
                                                        accel.acceleration.y,
                                                        accel.acceleration.z,
                                                        qw, qx, qy, qz);

  // Step 2: Calculate vertical acceleration (gravity component)
  vertical_accel = rotated_accel_z - gravity;

  // Step 3: Integrate vertical acceleration to get vertical velocity
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= dt * 1000000) {
    previousMicros = currentMicros;

    // Integrate vertical acceleration to get vertical velocity
    vertical_velocity += vertical_accel * dt;

    // Step 4: Integrate vertical velocity to get altitude
    altitude += vertical_velocity * dt;

    // Print results
    Serial.print("X accel: ");
    Serial.println(accel.acceleration.x);
    Serial.print("Y accel: ");
    Serial.println(accel.acceleration.y);
    Serial.print("Z accel: ");
    Serial.println(accel.acceleration.z);
    Serial.print("Vertical Accel (m/s^2): ");
    Serial.println(vertical_accel);
    Serial.print("Vertical Velocity (m/s): ");
    Serial.println(vertical_velocity);
    Serial.print("Altitude (m): ");
    Serial.println(altitude);
  }

  
}


//=======================FUNCTIONS======================


// Function to rotate accelerometer data based on quaternion orientation (thanks chatgpt)

float rotateAccelerometerToVertical(float accel_x, float accel_y, float accel_z, 
                                     float q_w, float q_x, float q_y, float q_z) {
  // Rotate the accelerometer vector (ACC_X, ACC_Y, ACC_Z) using the quaternion (qw, qx, qy, qz)
  float q0 = q_w, q1 = q_x, q2 = q_y, q3 = q_z;
  float vx = accel_x, vy = accel_y, vz = accel_z;

  // Quaternion rotation: q * v * q'
  // First, calculate q' (the conjugate of the quaternion)
  float q0_conj = q0;
  float q1_conj = -q1;
  float q2_conj = -q2;
  float q3_conj = -q3;

  // Perform quaternion multiplication q * v
  float qw_v = q0 * vx + q1 * vy + q2 * vz;
  float qx_v = q0 * vx + q1 * vy + q2 * vz;
  float qy_v = q0 * vx + q1 * vy + q2 * vz;
  float qz_v = q0 * vx + q1 * vy + q2 * vz;
  
  // Complete the quaternion multiplication to get final rotated vector
  return qz_v;
}



// Read sensors

void readSensors(){
  
  if (!baro.conversionComplete()){
      sensors_event_t temp;
  sensors_event_t pressure;
  lps.getEvent(&pressure, &temp);// get pressure

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t tempp;
  lsm6ds.getEvent(&accel, &gyro, &tempp);

  // fast-updating pressure and temp data
  LPS_PRESSURE = pressure.pressure; // float, hPa
  LPS_TEMP = temp.temperature;      // float, C
  
  // acceleration and gyro data
  ACC_X = accel.acceleration.x;  //float, m/s2
  ACC_Y = accel.acceleration.y;
  ACC_Z = accel.acceleration.z;

  GYRO_X = gyro.gyro.x;          // float, rad/s
  GYRO_Y = gyro.gyro.y;
  GYRO_Z = gyro.gyro.z;
  }

  else{
    // Updated every ~1 second (1 hz)
    // calibrated altitude data
    MPL_PRESSURE = baro.getLastConversionResults(MPL3115A2_PRESSURE);  // float, hPa
    MPL_ALTI = baro.getLastConversionResults(MPL3115A2_ALTITUDE);      // float, m
    baro.startOneShot();

    //Serial.println(micros() - dT_baro);

    dT_baro = micros();
    
    
  }

}


// SERVO FUNCTIONS

void deployBrake(){
  brake.write(0);
}

void retractBrake(){
  brake.write(130);
}
