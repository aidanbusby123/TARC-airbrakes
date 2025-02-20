#include "main.h"
#include "coms.h"

packet serialPacket = {0};

void initSerial(){

}

void readSerial(){
    uint8_t byte = 0;
    uint8_t count = 0;
    bool isMessage = false;
    while (Serial.available()){
        byte = Serial.read();
        if (isMessage == false){
            if (byte == MSG_START_BYTE){
                isMessage = true;
                count++;
            }
        } else {
            if (count == 1){
                serialPacket.type = byte;
            }
            if (count == 2){
                serialPacket.data_size = byte;
            }
            serialPacket.data = (uint8_t*)malloc(serialPacket.data_size);
            if ((count-2) < serialPacket.data_size){
                serialPacket.data[count-2] = byte;
            }
            if (byte == MSG_END_BYTE){
                break;
            }
            count++;
        }
    }
}

void writeSerial(){

}

void handleSerial(){
    if (serialPacket.type == MSG_TYPE_CALIBRATION){
        char *caldata = (char*)malloc(serialPacket.data_size + 1);
        memcpy(caldata, serialPacket.data, serialPacket.data_size);
        caldata[serialPacket.data_size] = 0;

        cal.loadCalibrationFromPacket(caldata);
    }
    if (serialPacket.type == MSG_TYPE_CONFIG){
        char *configdata = (char*)malloc(serialPacket.data_size + 1);
        memcpy(configdata, serialPacket.data, serialPacket.data_size);
        configdata[serialPacket.data_size] = 0;

        rocketConfig.loadConfigFromPacket(configdata);
    }
}