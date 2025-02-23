#include "main.h"
#include "coms.h"
#include "telemetry.h"

packet serialPacket = {0};

void initSerial(){
    
}

void readSerial(){
    serialPacket.start_byte = MSG_START_BYTE;
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
                handleSerial();
                continue;
            }
            count++;
        }
    }
}

void writeSerial(uint8_t type, uint8_t data_size, uint8_t *data){
    uint8_t *buffer;
    uint8_t *buffer_start;
    serialPacket.start_byte = MSG_START_BYTE;
    serialPacket.type = type;
    serialPacket.data_size = data_size;
    serialPacket.data = (uint8_t*)malloc(data_size);
    memcpy(serialPacket.data, data, data_size);

    int buffer_size = sizeof(serialPacket.start_byte) + sizeof(serialPacket.type) + sizeof(serialPacket.data_size) + serialPacket.data_size;

    buffer = (uint8_t*)malloc(sizeof(serialPacket.start_byte) + sizeof(serialPacket.type) + sizeof(serialPacket.data_size) + serialPacket.data_size);
    buffer_start = buffer;

    memcpy(serialPacket.data, data, data_size);

    memcpy(buffer, &serialPacket.start_byte, sizeof(serialPacket.start_byte));
    buffer += sizeof(uint8_t);
    memcpy(buffer, &serialPacket.type, sizeof(uint8_t));
    buffer += sizeof(uint8_t);
    memcpy(buffer, &serialPacket.data_size, sizeof(uint8_t));
    buffer += sizeof(uint8_t);
    memcpy(buffer, serialPacket.data, serialPacket.data_size);
    //Serial.println(serialPacket.data_size);
    buffer += serialPacket.data_size;

    buffer = buffer_start;

    Serial.write(buffer, buffer_size);
    
    free(buffer);




    
}

void handleSerial(){
    if (serialPacket.type == MSG_TYPE_CALIBRATION){
        char *caldata = (char*)malloc(serialPacket.data_size);
        memcpy(caldata, serialPacket.data, serialPacket.data_size);
        if (caldata[serialPacket.data_size-1] != 0){
            Serial.println(F("Incorrectly formatted calibration data packet"));
        }

        cal.loadCalibrationFromPacket(caldata);
    }

    if (serialPacket.type == MSG_TYPE_CONFIG){
        char *configdata = (char*)malloc(serialPacket.data_size);
        memcpy(configdata, serialPacket.data, serialPacket.data_size);
        if (configdata[serialPacket.data_size-1] != 0){
            Serial.println(F("Incorrectly formatted config data packet"));
        }

        rocketConfig.loadConfigFromPacket(configdata);
    }
}