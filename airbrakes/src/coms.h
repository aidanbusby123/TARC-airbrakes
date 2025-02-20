#include "main.h"

#define MSG_TYPE_TELEMETRY 0x8A
#define MSG_TYPE_CONFIG 0x8B
#define MSG_TYPE_CALIBRATION 0x8C

#define MSG_START_BYTE 0xFA
#define MSG_END_BYTE 0xFB

struct packet {
    uint8_t start_byte = MSG_START_BYTE;
    uint8_t type;
    uint8_t data_size;
    uint8_t* data;
    uint8_t end_byte = MSG_END_BYTE;
};

void readSerial();