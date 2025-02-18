#include "main.h"
#include <SdFat.h>
#include <ArduinoJson.h>

#pragma once
class sensorCalibration{
    public:
    static uint16_t crc16_update(uint16_t crc, uint8_t a);
    
  bool calibrate(sensors_event_t &event);

  /**! XYZ vector of offsets for zero-g, in m/s^2 */
  float accel_zerog[3] = {0, 0, 0};

  /**! XYZ vector of offsets for zero-rate, in rad/s */
  float gyro_zerorate[3] = {0, 0, 0};

  /**! XYZ vector of offsets for hard iron calibration (in uT) */
  float mag_hardiron[3] = {0, 0, 0};

  /**! The 3x3 matrix for soft-iron calibration (unitless) */
  float mag_softiron[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  /**! The magnetic field magnitude in uTesla */
  float mag_field = 50;

  char calfilename[64] = {0};
  SdFile calfile;

  JsonDocument calibJSON;

  bool begin(const char* filename);
  bool saveCalibration();
  bool loadCalibration();
};