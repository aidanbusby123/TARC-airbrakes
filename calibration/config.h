#include "main.h"
#include <ArduinoJson.h>
#include <SdFat.h>
#pragma once
#ifndef CONFIG_H
#define CONFIG_H

#define DEFAULT_TARGET_APOGEE 185
#define DEFAULT_REF_AREA 0.00343

const float DEFAULT_DRAG_FORCE_COEF_COEFS[3] = {0, -0.00101833, 0.00051306};

class config {
    private:
        float dragForceCoefCoefs[3] = {0, 0, 0};
        float deploymentTimeCoefs[3];    
        float ref_area;
        float target_apogee;
        float temperature;
        float pressure;
        float max_time;
        float drag_coefficient;
        float trigger_acceleration;
        float mass;
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

        float getRefArea();
        float getTargetApogee();
        float getTemperature() { return temperature; }
        float getPressure() { return pressure; }
        float getMaxTime() { return max_time; }
        float getTriggerAcceleration() { return trigger_acceleration; }
};

//SdFile configFile;
bool initConfig();
bool loadConfigFromFile();

#endif