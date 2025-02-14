#include "main.h"
#include <Arduino.h>
//#include <Adafruit_SPIFlash.h>
#include <SdFat.h>

/*Adafruit_FlashTransport_SPI flashTransport(4, SPI);
Adafruit_SPIFlash flash(&flashTransport);*/

SdFat sd;


void initSD(void){
    // Initialize the external flash
    if (!sd.begin(4, SPI_HALF_SPEED)) sd.initErrorHalt();

    // Open file system on flash (Fat16)
}