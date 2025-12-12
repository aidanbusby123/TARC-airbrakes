#include "main.h"
#include <Servo.h>


void controller::deployBrake(float angle){
    brake.write(angle);
}

        
bool controller::initBrake(){
    if (brake.attach(SERVO_PIN) == 0){
        //Serial.println("unable to initialize brake");
       // return false;
    }
    return true;
}

void brakeState::loadConfig(config Config){
    float *ptr;
    ptr = Config.getDragForceCoefCoefs();
    dragForceCoefCoefficients[0] = ptr[0];
    dragForceCoefCoefficients[1] = ptr[1];
    dragForceCoefCoefficients[2] = ptr[2];
    Serial.println(ptr[0]);
    Serial.println(ptr[1]);
    Serial.println(ptr[2]);
}

void brakeState::setPercentDeployed(float percent){ // set the current percent deployed
    percentDeployed = percent;
}

void brakeState::setDeltaPercent(float delta_percent){
    float newPercent = targetPercent + delta_percent;

    if (newPercent < 0)
        newPercent = 0;
    if (newPercent > 100.0)
        newPercent = 100;

    targetPercent = newPercent;
}
void brakeState::setTargetPercent(float percent){ // set the percent deployed target
    targetPercent = percent;
}

float brakeState::getDeployAngle(){
    return targetPercent;
}
float brakeState::getBrakeDeployCoef(){
    float dragForceCoef = 0.0f;
    if (DRAG_FORCE_COEF_COEFS_SIZE > 0){
        for (int i = 0; i < DRAG_FORCE_COEF_COEFS_SIZE; i++){
            dragForceCoef += dragForceCoefCoefficients[i] * pow(percentDeployed, i);
        }
    } else {
        dragForceCoef = targetPercent * rocketConfig.getBrakeCoef() + (1 - targetPercent) * rocketConfig.getDragCoef();
    }
    return dragForceCoef;
}

void brakeState::updateDeltaT(){
    Now = micros();
    delta_t = (float)((Now - lastTime) / 1000000.0f);
    lastTime = Now;
}

float brakeState::getDeployTime(){
    return deployTime;
}

void brakeState::updateDeployTime(){
    if (percentDeployed < targetPercent){
        deployTime += delta_t;
    } else if (percentDeployed > targetPercent){
        deployTime -= delta_t;
    } else if (percentDeployed == targetPercent);
}

void brakeState::updateState(){
    updateDeltaT();
    updateDeployTime();
    for (int i = 0; i < DEPLOYMENT_COEFS_SIZE; i++){
        percentDeployed = deploymentCoefficients[i] * pow(deployTime, i);
    } 
}