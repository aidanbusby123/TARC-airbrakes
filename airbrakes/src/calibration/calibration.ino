#include <Adafruit_Sensor.h>
#include "../sensor_calibration.h"
#include "../main.h"

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

    cal.begin("calibrat.dat");


}