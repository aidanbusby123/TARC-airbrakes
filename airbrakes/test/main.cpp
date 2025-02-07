#include <Arduino.h>
#include<Wire.h>

void setup(){
    Serial.println("Test!");
    Wire.begin(400000UL);
    pinMode(11, OUTPUT);
    digitalWrite(11, HIGH);
}

void loop () {
    Serial.println("This is a test!");
    digitalWrite(11, HIGH);
    delay(1000);
    digitalWrite(11, LOW);
}