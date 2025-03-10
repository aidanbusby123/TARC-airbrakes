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

    
    // Teensy 3.2
     // Open file system on flash (Fat16)
    /*pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);
    if (!sd.begin(SdSpiConfig(10, 0,  SPI_HALF_SPEED)))
        Serial.println("unable to init sd"); */
}