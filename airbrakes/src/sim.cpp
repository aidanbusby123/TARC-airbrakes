#include <Arduino.h>
#include "main.h"
#include "sim.h"



state simState;
float dragCoefficient;

state k1, k2, k3, k4;


void simLoop(){
    copyState(simState, rocketState);
    simState.delta_t = (float)SIM_TIME_S/SIM_REPS;

    for (int i = 0; i < SIM_REPS; i++){ // IMPORTANT!!! Change this to LAND once you have figured out the physics of coming back down!!!
        runSim();
    }
}

void runSim(){
    copyState(k1, simState);

    k1.delta_t = simState.delta_t;
    
    copyState(k2, k1);

    // Runge Kutta 4th Order approximation


    /* Muy, muy importante, pendojo! arregla la pendejado codigo tu estupido maricon*/
    // You need to use the fucking derivative of the acceleration, retard!!!!!
    
    k2.setVZ_Local(k1.getVZ_Local() + k1.getAZ_Local() * k1.delta_t * 0.5f);
    k2.setAZ_Local(-0.5 * 0 * pow(k1.getVZ_Local(),2) * dragCoefficient / k1.getMass());

    Serial.print("VZ_Local: ");
    Serial.print(k2.getVZ_Local());
    Serial.print(" AZ_Local: ");
    Serial.print(k2.getAZ_Local());
    Serial.print(" Air density: ");
    Serial.print(getAirDensity());
    


    k2.updateState();

    Serial.print(" VZ: ");
    Serial.print(k2.getVZ());
    Serial.print(" AZ: ");
    Serial.println(k2.getAZ());

    k3.setVZ_Local(k1.getVZ_Local() + k2.getAZ_Local() * k1.delta_t * 0.5f);
    k3.setAZ_Local(-0.5 * getAirDensity() * pow(k2.getVZ_Local(), 2) * dragCoefficient / k1.getMass());

    k3.updateState();

    k4.setVZ(k1.getVZ_Local() + k3.getAZ_Local() * k1.delta_t);
    k4.setAZ_Local(-0.5 * getAirDensity() * pow(k3.getVZ_Local(),2) * dragCoefficient/k1.getMass());

    simState.setVZ_Local((float)(1.0f/6.0f) * (k1.getVZ_Local() + 2 * k2.getVZ_Local() + 2* k3.getVZ_Local() + k4.getVZ_Local()));
    simState.setAZ_Local((float)(1.0f/6.0f) * (k1.getAZ_Local() + 2 * k2.getAZ_Local() + 2* k3.getAZ_Local() + k4.getAZ_Local()));
    
    simState.updateState();
}

void calcForces(){
    simState.setFZ_Local(-0.5 * getAirDensity()); // Calculate Fz (local inertial frame)
    
}

// !!!!!
float getAirDensity(){ // IMPORTANT!!! fix this, should not be defined here, should have option to get sim air density
    Serial.println(simState.getBaroPressure());
    Serial.println(simState.getBaroTemperature());
    //return ((28.97 * rocketState.getBaroPressure()) / (pow(8.31432, -3) * rocketState.getBaroTemperature())); // rho = MP/RT, gas density equation
    return(1.294627f);
}

void copyState(state& newState, state& curState){
    newState.setAX(curState.getAX());
    newState.setAY(curState.getAY());
    newState.setAZ(curState.getAZ());

    newState.setAX_Local(curState.getAX_Local());
    newState.setAY_Local(curState.getAY_Local());
    newState.setAZ_Local(curState.getAZ_Local());

    newState.setVX(curState.getVX());
    newState.setVY(curState.getVY());
    newState.setVZ(curState.getVZ());

    newState.setVX_Local(curState.getVX_Local());
    newState.setVY_Local(curState.getVY_Local());
    newState.setVZ_Local(curState.getVZ_Local());

    newState.setX(curState.getX());
    newState.setY(curState.getY());
    newState.setZ(curState.getZ());

    newState.setQuatW(curState.getQuatW());
    newState.setQuatX(curState.getQuatX());
    newState.setQuatY(curState.getQuatY());
    newState.setQuatZ(curState.getQuatZ());

    newState.setFX_Local(curState.getFX_Local());
    newState.setFY_Local(curState.getFY_Local());
    newState.setFZ_Local(curState.getFZ_Local());

    newState.setAltitude(curState.getAltitude());

}

void runTestSim(){
    copyState(simState, rocketState);
    simState.setVZ_Local(100);
    Serial.println(simState.getVZ_Local());
    Serial.println(simState.getVZ());
    simState.updateState();
    simState.delta_t = (float)SIM_TIME_S/SIM_REPS;

    for (int i = 0; i < SIM_REPS; i++){ // IMPORTANT!!! Change this to LAND once you have figured out the physics of coming back down!!!
        runSim();

     /*   Serial.print("Sim acceleration: ");
        Serial.print(simState.getAX());
        Serial.print(", ");
        Serial.print(simState.getAY());
        Serial.print(", ");
        Serial.print(simState.getAZ());
        Serial.print(" Sim velocity: ");
        Serial.print(simState.getVX());
        Serial.print(", ");
        Serial.print(simState.getVY());
        Serial.print(", ");
        Serial.println(simState.getVZ());
*/
    }
}

