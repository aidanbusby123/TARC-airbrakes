
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Wire.h>

#include"math.h"

Adafruit_MPL3115A2 baro;
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_LPS25 lps;

Servo brake;


struct state {
  // Local acceleration vector
  Vec3 AccelLocal = Vec3(0.0f, 0.0f, 0.0f);
  Quaternion AccelLocalQuaternion;

  // Global vertical acceleration
  Vec3 Accel = Vec3(0.0f, 0.0f, 0.0f);
  Quaternion AccelQuaternion;

  // Local velocity vector

  Vec3 VelLocal = Vec3(0.0f, 0.0f, 0.0f);

  // global velocities

  Vec3 Vel = Vec3(0.0f, 0.0f, 0.0f);

  // 

  Vec3 Rot = Vec3(0.0f, 0.0f, 0.0f);
  Quaternion RotQuaternion;

}RocketState;
// GLOBAL SENSOR VALUES

float MPL_PRESSURE;
float MPL_ALTI;

float LPS_PRESSURE;
float LPS_TEMP;

float ACC_X = 0;
float ACC_Y = 0;
float ACC_Z = 0;

float GYRO_X = 0;
float GYRO_Y = 0;
float GYRO_Z = 0;

float ACC_Z_ZERO = 0;

int DELTA_T = 0;

long t = 0;
float DeltaT = 0;

float dt = 0;
float time_now = 0;
float time_last = 0;

void setup() {
  digitalWrite(13, HIGH);

  Wire.begin();
  Wire.setClock( 400000UL);
  
  Serial.begin(115200);
  while(!Serial);
  //Serial.println("Airbrakes!");

  // Try to initialize altimeter 2
  if (!lps.begin_I2C()) {
    Serial.println("Failed to find LPS25 chip");
    while (1) { delay(10); }
  }
  //Serial.println("LPS25 Found!");

  lps.setDataRate(LPS25_RATE_25_HZ);

  
  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  if (!lsm6ds33.begin_I2C(0x6B)) {
    // if (!lsm6ds33.begin_SPI(LSM_CS)) {
    // if (!lsm6ds33.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }  
  }

  brake.attach(9);
  /*retractBrake();
  delay(500);
  deployBrake();
  delay(500);
  retractBrake();*/
  
  
  // use to set sea level pressure for current location
  // this is needed for accurate altitude measurement
  // STD SLP = 1013.26 hPa
  baro.setSeaPressure(1013.26);

  baro.setMode(MPL3115A2_ALTIMETER);
  //baro.startOneShot();


  // Acc setup

  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  //Serial.println("+-16G");

  lsm6ds33.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  
  readSensors();
  //ACC_Z_ZERO = ACC_Z;



  // Gyro setup
  
  // lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  lsm6ds33.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);


  // Interrupts (not important)

  lsm6ds33.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds33.configInt2(false, true, false); // gyro DRDY on INT2



}


void loop() {
  t = micros();
  readSensors();
  updateState();

  /*Serial.print("M:");
  Serial.print(MPL_PRESSURE);
  Serial.print(",");
  Serial.print("L:");
  Serial.print(LPS_PRESSURE);
*/
  //Serial.print("A:");
  //Serial.println(MPL_ALTI);  
  /*
  Serial.print("x:");
  Serial.print(ACC_X);

  Serial.print(",y:");
  Serial.print(ACC_Y);

  Serial.print(",z:");
  Serial.println(ACC_Z); */
  DeltaT = (float)(micros()-t)/1000000.0f;
  
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

void readSensors(){
  
  if (!baro.conversionComplete()){
      sensors_event_t temp;
  sensors_event_t pressure;
  lps.getEvent(&pressure, &temp);// get pressure

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t tempp;
  lsm6ds33.getEvent(&accel, &gyro, &tempp);

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

  delay(DELTA_T);



}

float V_Z = 0.0f;
float ALTITUDE = 0.0f;


long uS_dT = 0;

Quaternion DeltaRotQuaternion; // Quaternion to describe change in rotation angle from gyroscope data.
Quaternion TempQuaternion; // Temprorary quaternion


void updateState(){
  //RocketState.Vel_Local.set_z((ACC_Z-ACC_Z_ZERO) * ((float) dT / 1000000.0));
  //ALTITUDE = 0.9 * (ALTITUDE + V_Z * ((float) dT / 1000000.0) + 1/2 * (ACC_Z-ACC_Z_ZERO) * pow((float) dT / 1000000.0, 2)) + 0.1 * MPL_ALTI;


  RocketState.AccelLocal.setX(ACC_X);
  RocketState.AccelLocal.setY(ACC_Y);
  RocketState.AccelLocal.setZ(ACC_Z);


  // Update rocket rotation quaternions

  time_now = micros();
  dt = time_now - time_last;
  time_last = time_now;
  DeltaRotQuaternion = Quaternion(1, GYRO_X * dt/1000000.0f * 0.5, GYRO_Y * dt/1000000.0f * 0.5, GYRO_Z * dt/1000000.0f * 0.5); // Rotation quaternion for change in rotation angle, using gyroscope
  
  TempQuaternion = RocketState.RotQuaternion + (((DeltaRotQuaternion * RocketState.RotQuaternion))); // Update rotation quaternion, q_{i+1} = q_{i} + q_{gyro}*q_{i}* dt/2 
  RocketState.RotQuaternion = TempQuaternion.normalize(); // Normalize the temporary quaternion



  // Convert local acceleration to global acceleration

  RocketState.AccelLocalQuaternion.setX(RocketState.AccelLocal.getX());
  RocketState.AccelLocalQuaternion.setY(RocketState.AccelLocal.getY());
  RocketState.AccelLocalQuaternion.setZ(RocketState.AccelLocal.getZ());

  RocketState.AccelQuaternion = (RocketState.RotQuaternion * RocketState.AccelLocalQuaternion) * RocketState.RotQuaternion.conjugate();
  
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

  Serial.print(RocketState.AccelQuaternion.getX());
  Serial.print(", ");
  Serial.print(RocketState.AccelQuaternion.getY());
  Serial.print(", ");
  Serial.println(RocketState.AccelQuaternion.getZ());
  
  uS_dT = micros();
}
