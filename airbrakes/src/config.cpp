#include "main.h"
#include "config.h"
#include <SdFat.h>
#include <ArduinoJson.h>

SdFile configFile;

void initConfig(){
    if (!configFile.exists("config.dat")){
        sd.errorHalt("Config file does not exist!");
    }
    if (!configFile.open("config.dat", O_RDONLY)){
        sd.errorHalt("unable to open config file");
    }

    rocketConfig.begin("config.dat");
}

int config::begin(const char *filename){
    if (!configFile.open(filename, O_RDONLY)){
        sd.errorHalt("unable to open config file");
    }
}
void config::loadConfig(){
    DeserializationError error = deserializeJson(configJSON, configFile);

    targetApogee = configJSON['target_apogee'];

    refArea = configJSON['reference_area'];

    for (int i = 0; i < 3; i++){
        dragForceCoefCoefs[i] = configJSON['drag_force_coef_coefs'][i];
    }
    Serial.println("config loaded");
}

float config::getRefArea(){
    return refArea;
}

float config::getTargetApogee(){
    return targetApogee;
}