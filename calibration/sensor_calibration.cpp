
#include "sensor_calibration.h"
#include <ArduinoJson.h>

byte caldata[68]; // buffer to receive magnetic calibration data
byte calcount=0;

uint16_t sensorCalibration::crc16_update(uint16_t crc, uint8_t a) {
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

bool sensorCalibration::begin(const char* filename){
    strcpy(calfilename, filename);
    calfile.open("/");
    if (!calfile.exists(calfilename)){
        Serial.println(calfilename);
        sd.errorHalt("calfile does not exist");
        return false;
    }
    return true;
}

bool sensorCalibration::saveCalibration(){
    if (!calfile.open(calfilename, O_WRITE | O_CREAT | O_TRUNC)){
        sd.errorHalt("unable to open calibration file");
        return false;
    }

    JsonObject root = calibJSON.to<JsonObject>();
    JsonArray mag_hard_data = root.createNestedArray("mag_hardiron");
    for (int i = 0; i < 3; i++) {
        mag_hard_data.add(mag_hardiron[i]);
    }
    JsonArray mag_soft_data = root.createNestedArray("mag_softiron");
    for (int i = 0; i < 9; i++) {
        mag_soft_data.add(mag_softiron[i]);
    }
    root["mag_field"] = mag_field;
    JsonArray gyro_zerorate_data = root.createNestedArray("gyro_zerorate");
    for (int i = 0; i < 3; i++) {
        gyro_zerorate_data.add(gyro_zerorate[i]);
    }
    JsonArray accel_zerog_data = root.createNestedArray("accel_zerog");
    for (int i = 0; i < 3; i++) {
        accel_zerog_data.add(accel_zerog[i]);
    }
    // serializeJsonPretty(root, Serial);

    // Serialize JSON to file
    if (serializeJson(calibJSON, calfile) == 0) {
        Serial.println(F("Failed to write to file"));
        return false;
    }

    calfile.close();

    return true;
}

bool sensorCalibration::loadCalibrationFromFile(){
    if (!calfile.open(calfilename, O_READ)){
        sd.errorHalt("unable to open calibration file");
        return false;
    }

    DeserializationError error = deserializeJson(calibJSON, calfile);
    if (error) {
        Serial.println(F("Failed to read file"));
        return false;
    }

    calfile.close();

    for (int i = 0; i < 3; i++) {
        mag_hardiron[i] = calibJSON["mag_hardiron"][i] | 0.0;
    }
    for (int i = 0; i < 9; i++) {
        float def = 0;
        if (i == 0 || i == 4 || i == 8) {
        def = 1;
        }
        mag_softiron[i] = calibJSON["mag_softiron"][i] | def;
    }
    mag_field = calibJSON["mag_field"] | 0.0;
    for (int i = 0; i < 3; i++) {
        gyro_zerorate[i] = calibJSON["gyro_zerorate"][i] | 0.0;
    }
    for (int i = 0; i < 3; i++) {
        accel_zerog[i] = calibJSON["accel_zerog"][i] | 0.0;
    }
    

    return true;
}

bool sensorCalibration::loadCalibrationFromPacket(char *caldata){
    DeserializationError error = deserializeJson(calibJSON, caldata);
    if (error) {
        Serial.println(F("Failed to read cal buffer"));
        return false;
    }

    calfile.close();

    for (int i = 0; i < 3; i++) {
        mag_hardiron[i] = calibJSON["mag_hardiron"][i] | 0.0;
    }
    for (int i = 0; i < 9; i++) {
        float def = 0;
        if (i == 0 || i == 4 || i == 8) {
        def = 1;
        }
        mag_softiron[i] = calibJSON["mag_softiron"][i] | def;
    }
    mag_field = calibJSON["mag_field"] | 0.0;
    for (int i = 0; i < 3; i++) {
        gyro_zerorate[i] = calibJSON["gyro_zerorate"][i] | 0.0;
    }
    for (int i = 0; i < 3; i++) {
        accel_zerog[i] = calibJSON["accel_zerog"][i] | 0.0;
    }

    return true;
}

bool sensorCalibration::calibrate(sensors_event_t &event){
    
  if (event.type == SENSOR_TYPE_MAGNETIC_FIELD) {
    // hard iron cal
    float mx = event.magnetic.x - mag_hardiron[0];
    float my = event.magnetic.y - mag_hardiron[1];
    float mz = event.magnetic.z - mag_hardiron[2];
    // soft iron cal
    event.magnetic.x =
        mx * mag_softiron[0] + my * mag_softiron[1] + mz * mag_softiron[2];
    event.magnetic.y =
        mx * mag_softiron[3] + my * mag_softiron[4] + mz * mag_softiron[5];
    event.magnetic.z =
        mx * mag_softiron[6] + my * mag_softiron[7] + mz * mag_softiron[8];
  } else if (event.type == SENSOR_TYPE_GYROSCOPE) {
    event.gyro.x -= gyro_zerorate[0];
    event.gyro.y -= gyro_zerorate[1];
    event.gyro.z -= gyro_zerorate[2];
  } else if (event.type == SENSOR_TYPE_ACCELEROMETER) {
    event.acceleration.x -= accel_zerog[0];
    event.acceleration.y -= accel_zerog[1];
    event.acceleration.z -= accel_zerog[2];
  } else {
    return false;
  }
  return true;
}

void sensorCalibration::receiveCalibration() {
  uint16_t crc;
  byte b, i;

  while (Serial.available()) {
    b = Serial.read();
    if (calcount == 0 && b != 117) {
      // first byte must be 117
      return;
    }
    if (calcount == 1 && b != 84) {
      // second byte must be 84
      calcount = 0;
      return;
    }
    // store this byte
    caldata[calcount++] = b;
    if (calcount < 68) {
      // full calibration message is 68 bytes
      Serial.println("cal message");
      return;
    }
    // verify the crc16 check
    crc = 0xFFFF;
    for (i=0; i < 68; i++) {
      crc = crc16_update(crc, caldata[i]);
    }
    if (crc == 0) {
      // data looks good, use it
      Serial.println("saving data");
      float offsets[16];
      memcpy(offsets, caldata+2, 16*4);
      cal.accel_zerog[0] = offsets[0];
      cal.accel_zerog[1] = offsets[1];
      cal.accel_zerog[2] = offsets[2];
      
      cal.gyro_zerorate[0] = offsets[3];
      cal.gyro_zerorate[1] = offsets[4];
      cal.gyro_zerorate[2] = offsets[5];
      
      cal.mag_hardiron[0] = offsets[6];
      cal.mag_hardiron[1] = offsets[7];
      cal.mag_hardiron[2] = offsets[8];

      cal.mag_field = offsets[9];
      
      cal.mag_softiron[0] = offsets[10];
      cal.mag_softiron[1] = offsets[13];
      cal.mag_softiron[2] = offsets[14];
      cal.mag_softiron[3] = offsets[13];
      cal.mag_softiron[4] = offsets[11];
      cal.mag_softiron[5] = offsets[15];
      cal.mag_softiron[6] = offsets[14];
      cal.mag_softiron[7] = offsets[15];
      cal.mag_softiron[8] = offsets[12];

      if (! cal.saveCalibration()) {
        Serial.println("**WARNING** Couldn't save calibration");
      } else {
        Serial.println("Wrote calibration");    
      }
      calcount = 0;
      return;
    }
    // look for the 117,84 in the data, before discarding
    for (i=2; i < 67; i++) {
      if (caldata[i] == 117 && caldata[i+1] == 84) {
        // found possible start within data
        calcount = 68 - i;
        memmove(caldata, caldata + i, calcount);
        return;
      }
    }
    // look for 117 in last byte
    if (caldata[67] == 117) {
      caldata[0] = 117;
      calcount = 1;
    } else {
      calcount = 0;
    }
  }
}

void sensorCalibration::close(){
    calfile.close();
}

