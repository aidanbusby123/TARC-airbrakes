#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include <Servo.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ZeroDMA.h>
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
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h>

#pragma once
//#include <Wire.h>

#include"maths.h"

#define INIT_MASS 0.441
#define GRAVITY 9.8

#define SEAPRESSURE 1039.8

#define STATEHISTORY_SIZE 64// size of state history buffers


extern Adafruit_MPL3115A2 baro;
extern Adafruit_LSM6DS33 lsm6ds;
extern Adafruit_LPS25 lps;
extern Adafruit_LIS3MDL lis3mdl;

extern Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
extern Adafruit_Sensor_Calibration_SDFat cal;

extern sensors_event_t accel;
extern sensors_event_t gyro;
extern sensors_event_t mag;
extern sensors_event_t tempp;

extern Servo brake;

//extern FatVolume fatfs;
extern SdFat sd;

enum phase {
    PAD,
    LAUNCH,
    POWERED,
    COAST,
    APOGEE,
    LAND
};

enum statetype{ // What kind of state (simulation or physical)
    ROCKET,
    SIM
};

class state{
    private:

        float mass = INIT_MASS;

        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;

        float ax_local = 0.0f;
        float ay_local = 0.0f;
        float az_local = 0.0f;

        float pitch = 0.0f;
        float roll = 0.0f;
        float yaw = 0.0f;

        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;

        float vx_local = 0.0f;
        float vy_local = 0.0f;
        float vz_local = 0.0f;

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        float qw = 1.0f;
        float qx = 0.0f;
        float qy = 0.0f;
        float qz = 0.0f;

        float fx_local = 0.0f;
        float fy_local = 0.0f;
        float fz_local = 0.0f;



        float baroAltitude = 0.0f; // Barometric altitude
        float altitude = 0.0f; // Real altitude

        float baroPressure = 0.0f; // Barometric pressure

        float baroTemperature = 0.0f; // temperature

        float dragCoefficient = 0.0f; // Fix this


    public:

        // change in time, used to calculate velocity and position
        float delta_t = 0.0f;

        // Rocket flight phase

        phase flightPhase = PAD;
        
        // State type

        statetype stateType;

        // Get state values

        float getMass() { return mass; }

        float getAX() { return ax; }
        float getAY() { return ay; }
        float getAZ() { return az; }

        float getAX_Local() { return ax_local; }
        float getAY_Local() { return ay_local; }
        float getAZ_Local() { return az_local; }

        float getPitch() { return pitch; }
        float getRoll() { return roll; }
        float getYaw() { return yaw; }

        float getVX() { return vx; }
        float getVY() { return vy; }
        float getVZ() { return vz; }

        float getVX_Local() { return vx_local; }
        float getVY_Local() { return vy_local; }
        float getVZ_Local() { return vz_local; }

        float getX() { return x; }
        float getY() { return y; }
        float getZ() { return z; }

        float getQuatW() { return qw; }
        float getQuatX() { return qx; }
        float getQuatY() { return qy; }
        float getQuatZ() { return qz; }

        float getFX_Local() { return fx_local; }
        float getFY_Local() { return fy_local; }
        float getFZ_Local() { return fz_local; }

        float getAltitude() { return altitude; }
        float getBaroAltitude() { return baroAltitude; }

        float getBaroPressure() { return baroPressure; } 

        float getBaroTemperature() { return baroTemperature; }

        // Set state values

        void setMass(float mass) { this->mass = mass; }

        void setAX(float ax) { this->ax = ax; }
        void setAY(float ay) { this->ay = ay; }
        void setAZ(float az) { this->az = az; }
        
        void setAX_Local(float ax_local) { this->ax_local = ax_local; }
        void setAY_Local(float ay_local) { this->ay_local = ay_local; }
        void setAZ_Local(float az_local) { this->az_local = az_local; }

        void setVX(float vx) { this->vx = vx; }
        void setVY(float vy) { this->vy = vy; }
        void setVZ(float vz) { this->vz = vz; }

        void setVX_Local(float vx_local) { this->vx_local = vx_local; }
        void setVY_Local(float vy_local) { this->vy_local = vy_local; }
        void setVZ_Local(float vz_local) { this->vz_local = vz_local; }

        void setX(float x) { this->x = x; }
        void setY(float y) { this->y = y; }
        void setZ(float z) { this->z = z; }

        void setQuatW(float qw) { this->qw = qw; }
        void setQuatX(float qx) { this->qx = qx; }
        void setQuatY(float qy) { this->qy = qy; }
        void setQuatZ(float qz) { this->qz = qz; }

        void setFX_Local(float fx_local) { this->fx_local = fx_local; }
        void setFY_Local(float fy_local) { this->fy_local = fy_local; }
        void setFZ_Local(float fz_local) { this->fz_local = fz_local; }

        void setBaroAltitude(float baroAltitude) { this->baroAltitude = baroAltitude; }
        void setAltitude(float altitude) { this->altitude = altitude; }

        void setBaroPressure(float baroPressure) { this->baroPressure = baroPressure; }

        void setBaroTemperature(float baroTemperature) { this->baroTemperature = baroTemperature; }


        void updateState();
        
        void globalizeAcceleration();
        //void globalizeForces();
};
struct stateHistory{

  float mass = INIT_MASS;

        float time = 0.0f;

        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;

        float ax_local = 0.0f;
        float ay_local = 0.0f;
        float az_local = 0.0f;

        float pitch = 0.0f;
        float roll = 0.0f;
        float yaw = 0.0f;

        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;

        float vx_local = 0.0f;
        float vy_local = 0.0f;
        float vz_local = 0.0f;

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        float qw = 1.0f;
        float qx = 0.0f;
        float qy = 0.0f;
        float qz = 0.0f;

        float fx_local = 0.0f;
        float fy_local = 0.0f;
        float fz_local = 0.0f;

        float baroAltitude = 0.0f; // Barometric altitude
        float altitude = 0.0f; // Real altitude

        float baroPressure = 0.0f; // Barometric pressure

        float baroTemperature = 0.0f; // temperature

        float dragCoefficient = 0.0f; // Fix this
};


extern stateHistory* rocketStateHistory;
extern stateHistory* simStateHistory;

extern uint rocketStateHistory_index;
extern uint rocketStateHistory_size;

extern uint simStateHistory_index;
extern uint simStateHistory_size;

extern state rocketState; // Rocket state

extern state simState; // Simulation state


void readSensors(); // Read sensor data

void retractBrake(); // Retract airbrake
void deployBrake(); // Deploy airbrake
void brakeTest(); // Test airbrake


bool initSensors(void); // initialize sensors
void setupSensors(void); // setup the sensors
void initCalibration(void); // Initialize sensor calibration
void calibrateSensors(void); // calibrate sensors

void initSD(void); // initialize SD card

void initBT(); // Initialize Bluetooth
void loopBT(); // Loop Bluetooth

void initLogs(); // initialize flight logs
void logRocketState();
void logSimState();
void closeLogs();
void writeRocketStateLog();
void writeSimStateLog();
