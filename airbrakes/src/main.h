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

//#include <Wire.h>

#include"maths.h"


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

extern FatVolume fatfs;

enum phase {
    PAD,
    LAUNCH,
    POWERED,
    COAST,
    APOGEE,
    LAND
};

class state{
    private:

        float ax;
        float ay;
        float az;

        float ax_local;
        float ay_local;
        float az_local;

        float pitch;
        float roll;
        float yaw;

        float vx;
        float vy;
        float vz;

        float x;
        float y;
        float z;

        float qw;
        float qx;
        float qy;
        float qz;

        float baroAltitude; // Barometric altitude
        float altitude; // Real altitude

        float gravity = 9.8;

    public:

        // change in time, used to calculate velocity and position
        float delta_t = 0.0f;

        // Rocket flight phase

        phase flightPhase = PAD;

        // Get state values

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

        float getX() { return x; }
        float getY() { return y; }
        float getZ() { return z; }

        float getQuatW() { return qw; }
        float getQuatX() { return qx; }
        float getQuatY() { return qy; }
        float getQuatZ() { return qz; }

        float getAltitude() { return altitude; }

        // Set state values

        void setAX(float ax) { this->ax = ax; }
        void setAY(float ay) { this->ay = ay; }
        void setAZ(float az) { this->az = az; }
        
        void setAX_Local(float ax_local) { this->ax_local = ax_local; }
        void setAY_Local(float ay_local) { this->ay_local = ay_local; }
        void setAZ_Local(float az_local) { this->az_local = az_local; }

        void setVX(float vx) { this->vx = vx; }
        void setVY(float vy) { this->vy = vy; }
        void setVZ(float vz) { this->vz = vz; }

        void setX(float x) { this->x = x; }
        void setY(float y) { this->y = y; }
        void setZ(float z) { this->z = z; }

        void setQuatW(float qw) { this->qw = qw; }
        void setQuatX(float qx) { this->qx = qx; }
        void setQuatY(float qy) { this->qy = qy; }
        void setQuatZ(float qz) { this->qz = qz; }

        void setBaroAltitude(float baroAltitude) { this->baroAltitude = baroAltitude; }


        void updateState();
};


void readSensors(); // Read sensor data

void retractBrake(); // Retract airbrake
void deployBrake(); // Deploy airbrake
void brakeTest(); // Test airbrake


bool initSensors(void); // initialize sensors
void setupSensors(void); // setup the sensors
void initCalibration(void); // Initialize sensor calibration
void calibrateSensors(void); // calibrate sensors

void initFlash(void); // initialize flash memory
