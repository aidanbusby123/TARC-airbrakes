#include "main.h"
#include "sensor_calibration.h"
#include <ArduinoJson.h>

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
    if (!calfile.exists(calfilename)){
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

bool sensorCalibration::loadCalibration(){
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

