#include "main.h"

void state::updateState () { // Really only for rocketState, not for runge-kutta
  updateAcceleration();

  updateTime();

  // Update velocity values

  if (stateType == ROCKET && flightPhase != PAD && flightPhase != LAUNCH){
    vx += ax * delta_t;
    vy += ay * delta_t;
    vz += az * delta_t;
  // Update position values (need to implement Kalman filter, this uses simple complimentary filter)

    x += vx * delta_t;
    y += vy * delta_t;
    z += vz * delta_t;
  }

  /* not super sure about these*/

  //localizeAcceleration();
  localizeVelocity();

  updateEulerAngles();
  //

  // Very simple complimentary filter

  if (stateType == ROCKET && flightPhase != PAD && flightPhase != LAUNCH && baroConversionFinished == true){
    altitude = (1-BARO_GAIN) * (altitude + vz * delta_t) + BARO_GAIN * baro_altitude; 
    baroConversionFinished = false;
  } else if (stateType == ROCKET && flightPhase != PAD && flightPhase != LAUNCH) {
    altitude = altitude + vz * delta_t;
  }
  else if (stateType == SIM){
    altitude = altitude + vz * delta_t;

  }
}

void state::updateAcceleration(){
  globalizeAcceleration();
  az -= GRAVITY;
  localizeAcceleration();
}

void state::globalizeAcceleration(){
  // Temporary quaternion values
  float temp_w;
  float temp_x;
  float temp_y;
  float temp_z;

  // Use quaternions to find global acceleration

  temp_w = qw * 0 - qx * ax_local - qy * ay_local - qz * az_local;
  temp_x = qw * ax_local + qx * 0 + qy * az_local - qz * ay_local;
  temp_y = qw * ay_local - qx * az_local + qy * 0 + qz * ax_local;
  temp_z = qw * az_local + qx * ay_local - qy * ax_local + qz * 0;

  ax = temp_w * (-qx) + temp_x * qw + temp_y * (-qz) - temp_z * (-qy);
  ay = temp_w * (-qy) - temp_x * (-qz) + temp_y * qw + temp_z * (-qx);
  az = temp_w * (-qz) + temp_x * (-qy) - temp_y * (-qx) + temp_z * qw;
}

void state::globalizeVelocity(){
  // Temporary quaternion values
  float temp_w;
  float temp_x;
  float temp_y;
  float temp_z;

  // Use quaternions to find global acceleration

  temp_w = qw * 0 - qx * vx_local - qy * vy_local - qz * vz_local;
  temp_x = qw * vx_local + qx * 0 + qy * vz_local - qz * vy_local;
  temp_y = qw * vy_local - qx * vz_local + qy * 0 + qz * vx_local;
  temp_z = qw * vz_local + qx * vy_local - qy * vx_local + qz * 0;

  vx = temp_w * (-qx) + temp_x * qw + temp_y * (-qz) - temp_z * (-qy);
  vy = temp_w * (-qy) - temp_x * (-qz) + temp_y * qw + temp_z * (-qx);
  vz = temp_w * (-qz) + temp_x * (-qy) - temp_y * (-qx) + temp_z * qw;
}

void state::localizeVelocity(){
  float temp_w;
  float temp_x;
  float temp_y;
  float temp_z;

  temp_w = qw * 0 + qx * vx + qy * vy + qz * vz;
  temp_x = qw * vx - qx * 0 - qy * vz + qz * vy;
  temp_y = qw * vy + qx * vz - qy * 0 - qz * vx;
  temp_z = qw * vz - qx * vy + qy * vx - qz * 0;

  vx_local = temp_w * qx + temp_x * qw + temp_y * qz - temp_z * qy;
  vy_local = temp_w * qy - temp_x * qz + temp_y * qw + temp_z * qx;
  vz_local = temp_w * qz + temp_x * qy - temp_y * qx + temp_z * qw;
}

void state::localizeAcceleration(){
  float temp_w;
  float temp_x;
  float temp_y;
  float temp_z;

  temp_w = qw * 0 + qx * ax + qy * ay + qz * az;
  temp_x = qw * ax - qx * 0 - qy * az + qz * ay;
  temp_y = qw * ay + qx * az - qy * 0 - qz * ax;
  temp_z = qw * az - qx * ay + qy * ax - qz * 0;

  ax_local = temp_w * qx + temp_x * qw + temp_y * qz - temp_z * qy;
  ay_local = temp_w * qy - temp_x * qz + temp_y * qw + temp_z * qx;
  az_local = temp_w * qz + temp_x * qy - temp_y * qx + temp_z * qw;
}
void state::updatePos(){
  x += vx * delta_t;
  y += vy * delta_t;
  z += vz * delta_t;
  if ((stateType == ROCKET) && (flightPhase != PAD && flightPhase != LAUNCH && baroConversionFinished == true)){
    altitude = (1-BARO_GAIN) * (altitude + vz * delta_t) + BARO_GAIN * baro_altitude; 
    baroConversionFinished = false;
  } else if ((stateType == ROCKET) && (flightPhase != PAD && flightPhase != LAUNCH)) {
    altitude = altitude + vz * delta_t;
  }
  else if (stateType == SIM){
    altitude = altitude + vz * delta_t;

  }
}

void state::updateEulerAngles(){
  roll = atan2(2*(qw*qx + qy*qz), qw*qw - qx*qx - qy*qy + qz*qz);
  pitch = asin(2*(qw*qy - qx*qz));
  yaw = atan2(2*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz);
}

/*void state::globalizeForces(){
    // Temporary quaternion values
  float temp_w;
  float temp_x;
  float temp_y;
  float temp_z;

  // Use quaternions to find global acceleration

  temp_w = qw * 0 - qx * fx_local - qy * fy_local - qz * fz_local;
  temp_x = qw * fx_local + qx * 0 + qy * fz_local - qz * fy_local;
  temp_y = qw * ay_local - qx * az_local + qy * 0 + qz * ax_local;
  temp_z = qw * az_local + qx * ay_local - qy * ax_local + qz * 0;

  ax = temp_w * (-qx) + temp_x * qw + temp_y * (-qz) - temp_z * (-qy);
  ay = temp_w * (-qy) - temp_x * (-qz) + temp_y * qw + temp_z * (-qx);
  az = temp_w * (-qz) + temp_x * (-qy) - temp_y * (-qx) + temp_z * qw;
}*/

float state::calcBaroAltitude(){
  return  (((getGroundTemperature() + 273.15f) / 0.0065) * (1 - pow((rocketState.getBaroPressure()/rocketState.getGroundPressure()), 0.1903)));
}

float state::calcActualTargetApogee(float comp_apogee){ // Competition altimeters use a base temperature of 15C, we need to thus change our target apogee to account for this
  //Serial.println((getBaroTemperature()+273.15) / (288.15f) * comp_apogee);
 //Serial.println(getBaroTemperature());
  return ((getBaroTemperature()+273.15f) / (288.15f) * comp_apogee);
}


void state::updateTargetApogee(float comp_apogee){
  float actual_target_apogee = calcActualTargetApogee(rocketConfig.getTargetApogee());
  if (actual_target_apogee <= 0){
    handleError("Error: real target apogee is bad");
  }

  setTargetApogee(actual_target_apogee);
}
