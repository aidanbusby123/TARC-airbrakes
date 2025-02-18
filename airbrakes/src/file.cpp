#include "main.h"
#include <Arduino.h>
//#include <Adafruit_SPIFlash.h>
#include <SdFat.h>
//#include <SD.h>
/*Adafruit_FlashTransport_SPI flashTransport(4, SPI);
Adafruit_SPIFlash flash(&flashTransport);*/

//#define SD_CONFIG SdioConfig(FIFO_SDIO)

SdFat sd;


void initSD(void){
    // Initialize the external flash
    if (!sd.begin(SdioConfig(FIFO_SDIO))) //sd.initErrorHalt();
        Serial.println("unable to init sd");
    // Open file system on flash (Fat16)
}