#include "main.h"
#include <ArduinoJson.h>
#include <SdFat.h>
#pragma once

#define BRAKE_RETRACTED 55
#define BRAKE_DEPLOYED 135

#define DEFAULT_TARGET_APOGEE 241
#define DEFAULT_REF_AREA 0.00343

#define DEFAULT_KP 1
#define DEFAULT_KI 1
#define DEFAULT_KD 1

const float DEFAULT_DRAG_FORCE_COEF_COEFS[3] = {0, -0.00101833, 0.00051306};

class config {
    private:
        float dragForceCoefCoefs[3] = {0, 0, 0};
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
    public:
        float ground_lora_address;
        JsonDocument configJSON;
        
        int begin(const char* filename);
        bool loadConfigFromFile();
        void loadConfigFromPacket(char* configdata);
        void loadConfigDefaults();

        float *getDragForceCoefCoefs(){
            return dragForceCoefCoefs;
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
};

//SdFile configFile;
bool initConfig();
bool loadConfigFromFile();