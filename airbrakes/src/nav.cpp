#include "main.h"

void state::updateState () {

  globalizeAcceleration();


  az -= GRAVITY;
  

  // Update velocity values

  vx += ax * delta_t;
  vy += ay * delta_t;
  vz += az * delta_t;

  // Update position values (need to implement Kalman filter, this uses simple complimentary filter)

  x += vx * delta_t;
  y += vy * delta_t;
  z += vz * delta_t;

  // Very simple complimentary filter

  if (stateType == ROCKET){
    altitude = 0.99 * (altitude + vz * delta_t) + 0.01 * baroAltitude; 
  } else {
    altitude = altitude + vz * delta_t;
  }
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

void state::updatePos(){
  x += vx * delta_t;
  y += vy * delta_t;
  z += vz * delta_t;
  if (stateType == ROCKET){
    altitude = 0.99 * (altitude + vz * delta_t) + 0.01 * baroAltitude; 
  } else {
    altitude = altitude + vz * delta_t;
  }
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