#include "main.h"
#include "config.h"
#include <SdFat.h>
#include <ArduinoJson.h>
SdFile configFile;
bool initConfig(){
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
    Serial.println( " all good config-wise");
    rocketConfig.loadConfigDefaults();
    Serial.println("config defaults loaded");

    return true;

   // rocketConfig.begin("config.dat");
}

int config::begin(const char *filename){

    Serial.println(filename);
   // configFileTest.open("/");
    if (!configFile.open("configs2.dat", O_RDWR | O_CREAT)){
        sd.errorHalt("unable to open config file, dunno why");
    }
}
bool config::loadConfigFromFile(){
    DeserializationError error = deserializeJson(configJSON, configFile);

    if (error){
        Serial.println(F("Failed to read config file"));
        statusLight.setPixelColor(0, RED);
        return false;
    } else {

        target_apogee = configJSON["target_apogee"];
        if (target_apogee <= 0){
            Serial.println("Error: incorrect apogee");
            target_apogee = DEFAULT_TARGET_APOGEE;
        }

        ref_area = configJSON["ref_area"];

        if (ref_area <= 0){
            ref_area = DEFAULT_REF_AREA;
        }

        for (int i = 0; i < 3; i++){
            dragForceCoefCoefs[i] = configJSON["drag_force_coef_coefs"][i];
            if (dragForceCoefCoefs[i] < 0)
                dragForceCoefCoefs[i] = 0;
        }

        for (int i = 0; i < 3; i++){
            deploymentTimeCoefs[i] = configJSON["deployment_time_coefs"][i];
            if (deploymentTimeCoefs[i] < 0)
                deploymentTimeCoefs[i] = 0;
        }

        pressure = configJSON["pressure"];
        temperature = configJSON["temperature"];
        ground_lora_address = configJSON["ground_lora_address"];
        if (ground_lora_address < 0){
            ground_lora_address = 0;
        }

        drag_coefficient = configJSON["drag_coefficient"];
        if (drag_coefficient <= 0){
            drag_coefficient = DEFAULT_DRAG_COEF;
        }

        trigger_acceleration = configJSON["trigger_acceleration"];
        if (trigger_acceleration <= 0)
            trigger_acceleration = DEFAULT_TRIGGER_ACCEL;

        max_time = configJSON["max_time"];

        mass = configJSON["mass"];
        if (mass <= 0)
            mass = INIT_MASS;
        Serial.println("config loaded");

        brake_retracted = configJSON["brake_retracted"];
        brake_deployed = configJSON["brake_deployed"];

        if (brake_retracted == brake_deployed){
            brake_retracted = BRAKE_RETRACTED;
            brake_deployed = BRAKE_DEPLOYED;
        }

    }
    return true;
}

void config::loadConfigFromPacket(char *configdata){
    DeserializationError error = deserializeJson(configJSON, configdata);

    if (error){
        Serial.println(F("Failed to read config data"));
    }

    target_apogee = configJSON["target_apogee"];

    ref_area = configJSON["ref_area"];

    for (int i = 0; i < 3; i++){
        dragForceCoefCoefs[i] = configJSON["drag_force_coef_coefs"][i];
    }
    ground_lora_address = configJSON["ground_lora_address"];
    Serial.println("config loaded");
}

void config::loadConfigDefaults(){
    target_apogee = DEFAULT_TARGET_APOGEE;
    ref_area = DEFAULT_REF_AREA;
    for (int i = 0; i < 3; i++){
        dragForceCoefCoefs[i] = DEFAULT_DRAG_FORCE_COEF_COEFS[i];
    }
    pressure = 1013.25;
    temperature = 20.0;
    ground_lora_address = 0;
    drag_coefficient = DEFAULT_DRAG_COEF;

}

float config::getRefArea(){
    return ref_area;
}

float config::getTargetApogee(){
    return target_apogee;
}

