#include "main.h"
#include <Arduino.h>
#include <Adafruit_SPIFlash.h>

Adafruit_FlashTransport_SPI flashTransport(4, SPI);
Adafruit_SPIFlash flash(&flashTransport);

FatVolume fatfs;


void initFlash(void){
    // Initialize the external flash
    flash.begin();

    // Open file system on flash (Fat16)
    if (!fatfs.begin(&flash)){
        Serial.println("Fat16 filesystem does not exist");
    }
}