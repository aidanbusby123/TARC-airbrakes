#include "main.h"
#include <ArduinoJson.h>
#include <SdFat.h>
#pragma once

#define BRAKE_RETRACTED 35
#define BRAKE_DEPLOYED 80

#define DEFAULT_TARGET_APOGEE 241
#define DEFAULT_REF_AREA 0.00343

#define DEFAULT_KP 0.2
#define DEFAULT_KI 0.05
#define DEFAULT_KD 0.2

const float DEFAULT_DRAG_FORCE_COEF_COEFS[3] = {0, -0.00101833, 0.00051306};

const float DEFAULT_DRAG_COEF = 0.5;

class config {
    private:
        float brakeDragForceCoefCoefs[1] = {DEFAULT_DRAG_COEF};
        float brakeCoef = -1.0;
        float deploymentTimeCoefs[3];    
        float ref_area;
        float target_apogee;
        float temperature;
        float pressure;
        float max_time;
        float drag_coefficient;
        float trigger_acceleration;
        float mass;
        float brake_retracted = BRAKE_RETRACTED;
        float brake_deployed = BRAKE_DEPLOYED;
        float kp = DEFAULT_KP;
        float ki = DEFAULT_KI;
        float kd = DEFAULT_KD;

        float p_var[4] = {1, 1, 1, 1};
        float q_var[4] = {0.5, 0.5, 0.1, 0.4};
        float r_var[2] = {0.5, 1.0};


        bool use_ekf = false;


    public:
        float ground_lora_address;
        JsonDocument configJSON;
        
        int begin(const char* filename);
        bool loadConfigFromFile();
        void loadConfigFromPacket(char* configdata);
        void loadConfigDefaults();

        float *getBrakeDragForceCoefCoefs(){
            return brakeDragForceCoefCoefs;
        }

        float getDragCoef() { return drag_coefficient; }
        float getBrakeCoef() {return brakeCoef; }

        float getRefArea();
        float getTargetApogee();
        float getTemperature() { return temperature; }
        float getPressure() { return pressure; }
        float getMaxTime() { return max_time; }
        float getTriggerAcceleration() { return trigger_acceleration; }
        float getBrakeRetracted() { return brake_retracted; }
        float getBrakeDeployed() { return brake_deployed; }
        float getMass() { return mass; }
        float getKP() { return kp; }
        float getKI() { return ki; }
        float getKD() { return kd; }
        float* getP() { return p_var; }
        float* getQ() { return q_var; }
        float* getR() { return r_var; }
        
        float getUseEKF(){
            return use_ekf;
        }
};

//SdFile configFile;
bool initConfig();
bool loadConfigFromFile();