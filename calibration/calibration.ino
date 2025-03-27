
#include "main.h"
#include <Adafruit_Sensor.h>
//#include "sensor_calibration.h"

//#include "/home/aidan/ws_airbrakes/airbrakes/src/main.h"

#define DEGREES_PER_RADIAN 360 / (2 * PI)

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
sensorCalibration cal;


sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t tempp;


void setup(){
    Serial.begin(115200);
    while (!Serial) delay (10);

    initSensors();
    setupSensors();
    initSD();

    initCalibration();


}

int loopcount = 0;
int maincount = 0;

void loop(){
  lsm.getEvent(&accel, &mag, &gyro, &tempp);
  loopcount++;
  Serial.print("Raw:");
  Serial.print(int(accel.acceleration.x*8192/9.8)); Serial.print(",");
  Serial.print(int(accel.acceleration.y*8192/9.8)); Serial.print(",");
  Serial.print(int(accel.acceleration.z*8192/9.8)); Serial.print(",");
  Serial.print(int(gyro.gyro.x*DEGREES_PER_RADIAN*16)); Serial.print(",");
  Serial.print(int(gyro.gyro.y*DEGREES_PER_RADIAN*16)); Serial.print(",");
  Serial.print(int(gyro.gyro.z*DEGREES_PER_RADIAN*16)); Serial.print(",");
  Serial.print(int(mag.magnetic.x*10)); Serial.print(",");
  Serial.print(int(mag.magnetic.y*10)); Serial.print(",");
  Serial.print(int(mag.magnetic.z*10)); Serial.println("");

  // unified data
  Serial.print("Uni:");
  Serial.print(accel.acceleration.x); Serial.print(",");
  Serial.print(accel.acceleration.y); Serial.print(",");
  Serial.print(accel.acceleration.z); Serial.print(",");
  Serial.print(gyro.gyro.x, 4); Serial.print(",");
  Serial.print(gyro.gyro.y, 4); Serial.print(",");
  Serial.print(gyro.gyro.z, 4); Serial.print(",");
  Serial.print(mag.magnetic.x); Serial.print(",");
  Serial.print(mag.magnetic.y); Serial.print(",");
  Serial.print(mag.magnetic.z); Serial.println("");
    cal.receiveCalibration();
      // occasionally print calibration
  if (loopcount == 50 || loopcount > 100) {
    Serial.print("Cal1:");
    for (int i=0; i<3; i++) {
      Serial.print(cal.accel_zerog[i], 3); 
      Serial.print(",");
    }
    for (int i=0; i<3; i++) {
      Serial.print(cal.gyro_zerorate[i], 3);
      Serial.print(",");
    }  
    for (int i=0; i<3; i++) {
      Serial.print(cal.mag_hardiron[i], 3); 
      Serial.print(",");
    }  
    Serial.println(cal.mag_field, 3);
    loopcount++;
    maincount++;
  }
  if (loopcount >= 100) {
    Serial.print("Cal2:");
    for (int i=0; i<9; i++) {
      Serial.print(cal.mag_softiron[i], 4); 
      if (i < 8) Serial.print(',');
    }
    Serial.println();
    loopcount = 0;
  }

  delay(10); 
}