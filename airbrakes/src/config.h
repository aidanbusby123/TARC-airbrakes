#include "main.h"
#include <ArduinoJson.h>
#include <SdFat.h>
#pragma once

#define DEFAULT_TARGET_APOGEE 87
#define DEFAULT_REF_AREA 0.00343

const float DEFAULT_DRAG_FORCE_COEF_COEFS[3] = {0, -0.00101833, 0.00051306};

class config {
    private:
        float dragForceCoefCoefs[3];
        float deploymentTimeCoefs[3];    
        float ref_area;
        float target_apogee;
        float temperature;
        float pressure;
        float max_time;
        float drag_coefficient;
    public:
        float ground_lora_address;
        JsonDocument configJSON;
        
        int begin(const char* filename);
        void loadConfigFromFile();
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
};

//SdFile configFile;
void initConfig();
void loadConfigFromFile();