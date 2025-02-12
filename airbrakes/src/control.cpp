#include "main.h"


void controller::deployBrake(float percent){
    brake.write(BRAKE_RETRACTED * (1-percent) + BRAKE_DEPLOYED*percent);
    airBrakeState.setPercentDeployed(percent);
}
        
void controller::initBrake(){
    brake.attach(9);
}

void brakeState::setPercentDeployed(float percent){
    percentDeployed = percent;
}

float brakeState::getDragForce(){
    float dragForceCoef = 0.0f;
    for (int i = 0; i < DRAG_FORCE_COEF_COEFS_SIZE; i++){
        dragForceCoef += dragForceCoefCoefficients[i] * percentDeployed;
    }
}