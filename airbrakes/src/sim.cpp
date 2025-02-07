#include <Arduino.h>
#include "main.h"
#include "sim.h"

/*TO-DO: TRANSITION TO USING THE ACTUAL SENSOR DATA, LOOP THROUGH stepSim IN updateSim, PREPARE FOR ACTUAL DATA.
Also, figure out how to convert euler angles to quaternions, will help with simulation testing*/

state simState;
float dragCoefficient = 0.53f;
//float dragCoefficient = 0.0f;
float crossSection = 0.00342f;

state k1, k2, k3, k4;


uint simStepNum;
double simStartTime = 0.0f;


void simLoop(){
    copyState(simState, rocketState);
    simState.delta_t = (float)SIM_TIME_S/SIM_REPS;

    for (int i = 0; i < SIM_REPS; i++){ // IMPORTANT!!! Change this to LAND once you have figured out the physics of coming back down!!!
        stepSim();
    }
}

void updateSim(){
    if (simState.time - simStartTime < SIM_TIME_S){
        stepSim();
    } else {
        //logSimState();
        simState.reset();
        simStepNum = 0;
    }
}

void stepSim(){ // do i need to update position for intermediaries?
    copyState(k1, simState);

    k1.delta_t = simState.delta_t;
    simState.time += simState.delta_t;

    copyState(k2, k1);

    // Runge Kutta 4th Order approximation


    /* Muy, muy importante, pendojo! arregla la pendejado codigo tu estupido maricon*/
    // You need to use the fucking derivative of the acceleration, retard!!!!!

    
    k2.setVZ_Local(k1.getVZ_Local() + k1.getAZ_Local() * k1.delta_t * 0.5f);
    k2.setAZ_Local((-0.5 * getAirDensity() * k1.getVZ_Local() * abs(k1.getVZ_Local()) * dragCoefficient * crossSection/ k1.getMass()));

    k2.globalizeVelocity();
    k2.globalizeAcceleration();

    k2.setAZ(k2.getAZ()-(float)GRAVITY);

    k2.localizeVelocity();

    //k2.updatePos();


    k3.setVZ_Local(k1.getVZ_Local() + k2.getAZ_Local() * k1.delta_t * 0.5f);
    k3.setAZ_Local((-0.5 * getAirDensity() * k2.getVZ_Local() * abs(k2.getVZ_Local()) * dragCoefficient * crossSection/ k1.getMass()));

    k3.globalizeVelocity();
    k3.globalizeAcceleration();

    k3.setAZ(k3.getAZ()-(float)GRAVITY);

    k3.localizeVelocity();

    //k3.updatePos();

    k4.setVZ_Local(k1.getVZ_Local() + k3.getAZ_Local() * k1.delta_t);
    k4.setAZ_Local((-0.5 * getAirDensity() * k3.getVZ_Local() * abs(k3.getVZ_Local()) * dragCoefficient * crossSection /k1.getMass()));

    k4.globalizeVelocity();
    k4.globalizeAcceleration();

    k4.setAZ(k4.getAZ()-(float)GRAVITY);

    k4.localizeVelocity();

    k1.updatePos();
    k2.updatePos();
    k3.updatePos();
    k4.updatePos();

    
    simState.setAltitude(simState.getAltitude() + (float)(1.0f/6.0f) * (k1.getVZ() + 2 * k2.getVZ() + 2 * k3.getVZ() + k4.getVZ()) * simState.delta_t);
    simState.setVZ_Local(simState.getVZ_Local() + (float)(1.0f/6.0f) * (k1.getAZ_Local() + 2 * k2.getAZ_Local() + 2* k3.getAZ_Local() + k4.getAZ_Local()) * simState.delta_t);
    

    simState.globalizeVelocity();

    simState.setAZ_Local((-0.5 * getAirDensity() * simState.getVZ_Local() * abs(simState.getVZ_Local()) * dragCoefficient * crossSection /simState.getMass()));

    simState.globalizeAcceleration();

    simState.setAZ(simState.getAZ()-float(GRAVITY));

    simState.localizeVelocity();
    
    //simState.updatePos();

   /* Serial.print("VZ_Local: ");
    Serial.print(simState.getVZ_Local());
    Serial.print(" AZ_Local: ");
    Serial.print(simState.getAZ_Local());
    Serial.print(" Air density: ");
   // Serial.print(getAirDensity());

    Serial.print(" VZ: ");
    Serial.print(simState.getVZ());
    Serial.print(" AZ: ");
    Serial.println(simState.getAZ());
*/
    simStepNum++;
}

void calcForces(){
    simState.setFZ_Local(-0.5 * getAirDensity()); // Calculate Fz (local inertial frame)
    
}

// !!!!!
float getAirDensity(){ // IMPORTANT!!! fix this, should not be defined here, should have option to get sim air density
   // Serial.println(simState.getBaroPressure());
   // Serial.println(simState.getBaroTemperature());
    //return ((28.97 * rocketState.getBaroPressure()) / (pow(8.31432, -3) * rocketState.getBaroTemperature())); // rho = MP/RT, gas density equation
    return(1.20f);
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
    float lastVel = 0.0f;
    float apogee = 0.0f;
    simState.setAltitude(43);
    simState.setVZ_Local(74.34);
    simState.setAZ_Local(-24.4);
    simState.stateType = SIM;
    //Serial.println(simState.getVZ_Local());
    //Serial.println(simState.getVZ());
    simState.delta_t = 0.1;
    lastVel = simState.getVZ();
    simState.updateState();
    while (1){
    if (simState.time - simStartTime < SIM_TIME_S){
        stepSim();
        Serial.print(simState.time - simStartTime + 1.14);
        Serial.print(", ");
        Serial.print(simState.getAltitude());
        Serial.print(", ");
        Serial.print(simState.getAZ());
        Serial.print(", ");
        Serial.println(simState.getVZ());
        if (simState.getVZ() < 0 && lastVel >= 0){
            apogee = simState.getAltitude();
        }
        
    } else {
        //logSimState();
        simState.reset();
        simStepNum = 0;
        break;
    }
    }
}

void state::reset(){
    ax = 0.0f;
    ay = 0.0f;
    az = 0.0f;
    ax_local = 0.0f;
    ay_local = 0.0f;
    az_local = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    roll = 0.0f;
    vx = 0.0f;
    vy = 0.0f;
    vz = 0.0f;
    vx_local = 0.0f;
    vy_local = 0.0f;
    vz_local = 0.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qx = 0.0f;
    qz = 0.0f;
    fx_local = 0.0f;
    fy_local = 0.0f;
    fz_local = 0.0f;
    baroAltitude = 0.0f;
    altitude = 0.0f;
    baroPressure = 0.0f;
    baroTemperature = 0.0f;
    dragCoefficient = 0.0f;
    time = 0;
    delta_t = 0;
}
