#include "main.h"
#include "config.h"
#include "sim.h"
#include "PID.h"

float SIGMA_DELTA = 0.5;

void PIDController::init(config configuration){
    last_time = time =   (float)(micros()) / 1000000.0f;
    this->kp = configuration.getKP();
    this->ki = configuration.getKI();
    this->kd = configuration.getKD();

}
float PIDController::compute(float predicted_apogee, float target_apogee){
    updateTime();

    float dt = getDeltaT();

    float delta_apogee = 0;

    delta_apogee = predicted_apogee - target_apogee;





    // add in the delta time aspect
    this->d = (delta_apogee - delta_apogee_prime) / dt * this->kd;
    this->i += delta_apogee * this->ki * dt;
    if (this->i >= kp * SIGMA_DELTA * 0.25){
        this->i = kp * SIGMA_DELTA * 0.25;
    }
    if (this->i <= - 1 * kp * SIGMA_DELTA * 0.25 )
        this->i = -1 * kp * SIGMA_DELTA * 0.25;

        
    this->p = delta_apogee * this->kp;

    
    this->delta_apogee_prime = delta_apogee;
    

    this->pid += (p + i + d) * dt; 

    

    if (pid < 0)
        pid = 0;

    if (pid > 1)
        pid = 1;
    

    Serial.println("PID constants");
    Serial.println(this->kp);
    Serial.println(this->ki);
    Serial.println(this->kd);


    Serial.print("P: ");
    Serial.println(this->p);
    Serial.print("I: ");
    Serial.println(this->i);
    Serial.print("D: ");
    Serial.println(this->d);
    return this->pid;
}

void PIDController::updateTime(){
    time = (float)(micros()) / 1000000.0f;
    delta_t = time - last_time;
    last_time = time;
}

float PIDController::getDeltaT(){
    return this->delta_t;
}