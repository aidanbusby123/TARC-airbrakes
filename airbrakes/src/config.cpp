#include "main.h"
#include "config.h"
#include <SdFat.h>
#include <ArduinoJson.h>
SdFile configFile;
void initConfig(){
    /*if (!configFile.exists("config.dat")){
        sd.errorHalt("Config file does not exist!");
    }*/
  if (!configFile.open("/", O_RDONLY)){
    sd.errorHalt("unable to open root");
   }
   configFile.close();
    if (!configFile.open("config.dat", O_RDWR | O_CREAT)){
        sd.errorHalt("unable to open config file (num 1)");
    }

   // rocketConfig.begin("config.dat");
}

int config::begin(const char *filename){

    Serial.println(filename);
   // configFileTest.open("/");
    if (!configFile.open("configs2.dat", O_RDWR | O_CREAT)){
        sd.errorHalt("unable to open config file, dunno why");
    }
}
void config::loadConfigFromFile(){
    DeserializationError error = deserializeJson(configJSON, configFile);

    if (error){
        Serial.println(F("Failed to read config file"));
    }

    targetApogee = configJSON["target_apogee"];

    refArea = configJSON["reference_area"];

    for (int i = 0; i < 3; i++){
        dragForceCoefCoefs[i] = configJSON["drag_force_coef_coefs"][i];
    }
   // Serial.println("config loaded");
}

void config::loadConfigFromPacket(char *configdata){
    DeserializationError error = deserializeJson(configJSON, configdata);

    if (error){
        Serial.println(F("Failed to read config data"));
    }

    targetApogee = configJSON["target_apogee"];

    refArea = configJSON["reference_area"];

    for (int i = 0; i < 3; i++){
        dragForceCoefCoefs[i] = configJSON["drag_force_coef_coefs"][i];
    }
    Serial.println("config loaded");
}

float config::getRefArea(){
    return refArea;
}

float config::getTargetApogee(){
    return targetApogee;
}