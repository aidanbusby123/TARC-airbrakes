#include "main.h"
#include <Servo.h>


float brakeState::getFormulaBrakeAngle(float theta){
    float angle = 0.0;
    if (r != 0 && w != 0 && s != 0 && h != 0 && l != 0){
        float num = pow(h - l * cos (-1 * theta/180.0), 2) + pow(w - l * sin (-1 * theta/180.0), 2) + pow(r, 2) - pow(s, 2);
        float denom = 2 * r * (w - l * sin(-1 * theta/180.0));

        if (denom >= 0.001){
            angle = acos(num / denom) * 180.0 / PI;
        }
    }
    return angle;
}
void controller::deployBrake(float angle){
    servo_angle = angle;
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
    ptr = Config.getBrakeDragForceCoefCoefs();
    dragForceCoefCoefficients[0] = ptr[0];
    dragForceCoefCoefficients[1] = ptr[1];
    dragForceCoefCoefficients[2] = ptr[2];

    dragForceCoefCoef = Config.getBrakeCoef();


    Serial.print("Airbrake Drag Coefs: ");
    Serial.print(ptr[0]);
    Serial.print(", ");
    Serial.print(ptr[1]);
    Serial.print(", ");
    Serial.println(ptr[2]);

    start_angle = Config.getBrakeRetracted();

    end_angle = Config.getBrakeDeployed();



    if (Config.getUseBrakeFormula() == true){
        use_brake_formula = true;
        start_angle = getFormulaBrakeAngle(0);
        end_angle = 90 - getFormulaBrakeAngle(50);
    }
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
    calcDeployAngle(percent);
    calcServoAngle(targetDeployAngle);
    curDragCoefficient = dragForceCoefCoef * sin(targetPercent/100.0f * 50.0 / 180.0 * PI);
}

void brakeState::calcDeployAngle(float percent){

    targetDeployAngle = start_angle + percent / 100.0 * end_angle;
}


float brakeState::getDeployAngle(){

    return targetDeployAngle;
}

void brakeState::calcServoAngle(float angle){

    if (use_brake_formula == true){
        targetServoAngle = getFormulaBrakeAngle(angle);
    } else 
    //if (sin(PI * angle / 180) > 0.)
    //targetServoAngle = 180 / PI * acos((pow(r, 2) + 2 * pow(l, 2) * (1 + cos(PI * angle/180)) - pow(s, 2))/(2 * l * r * sin(PI * angle / 180)));
        targetServoAngle = (float)(angle/60.0)*(end_angle-start_angle) + start_angle;
}
float brakeState::getServoAngle(){

    return targetServoAngle;
}



float brakeState::getBrakeDeployCoef(){
    float dragForceCoef = 0.0f;
    /*if (DRAG_FORCE_COEF_COEFS_SIZE > 0){
        for (int i = 0; i < DRAG_FORCE_COEF_COEFS_SIZE; i++){
            dragForceCoef += dragForceCoefCoefficients[i] * pow(percentDeployed, i);
        }
    } else {
        dragForceCoef = targetPercent * rocketConfig.getBrakeCoef() + (1 - targetPercent) * rocketConfig.getDragCoef();
    }*/
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