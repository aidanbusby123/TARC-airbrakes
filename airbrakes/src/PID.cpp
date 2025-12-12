#include "main.h"
#include "config.h"
#include "sim.h"
#include "PID.h"

void PIDController::init(){
    last_time = time =   (float)(micros()) / 1000000.0f;

}
float PIDController::compute(float predicted_apogee, float target_apogee){
    updateTime();

    float dt = getDeltaT();

    float delta_apogee = 0;
    if (predicted_apogee > target_apogee)    
        delta_apogee = predicted_apogee - target_apogee;
    else
        delta_apogee = 0;


    // add in the delta time aspect
    
    this->d = (delta_apogee - delta_apogee_prime) * this->kd * dt;
    this->i = this->i + delta_apogee * this->ki * dt;
    this->p += this-> kp * delta_apogee * dt;

    this->delta_apogee_prime = delta_apogee;
    
    pid = d + i + p;
    
    return pid = d + i + p;
}

void PIDController::updateTime(){
    time = (float)(micros()) / 1000000.0f;
    delta_t = time - last_time;
    last_time = time;
}

float PIDController::getDeltaT(){
    return this->delta_t;
}