#include "main.h"
#include <ArduinoJson.h>
#include <SdFat.h>
#pragma once

class config {
    private:
        float dragForceCoefCoefs[3];
        float deploymentTimeCoefs[3];    
        float refArea;
        float targetApogee;
    public:
        StaticJsonDocument<512> configJSON;
        
        int begin(const char* filename);
        void loadConfigFromFile();
        void loadConfigFromPacket(char* configdata);

        float *getDragForceCoefCoefs(){
            return dragForceCoefCoefs;
        }

        float getRefArea();
        float getTargetApogee();
};

//SdFile configFile;
void initConfig();
void loadConfigFromFile();