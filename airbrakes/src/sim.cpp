#include <Arduino.h>
#include "main.h"
#include "sim.h"

/*TO-DO: TRANSITION TO USING THE ACTUAL SENSOR DATA, LOOP THROUGH stepSim IN updateSim, PREPARE FOR ACTUAL DATA.
Also, figure out how to convert euler angles to quaternions, will help with simulation testing*/

/*TO-DO Figure out how to fix timing, have both time since mcu bootup and simulation time*/

state simState;
//float dragCoefficient = 0.0f;
float dragCoefficient = 0.52f;
float crossSection = 0.00342f;

state k1, k2, k3, k4;


uint simStepNum;
float simStartTime = 0.0f;


/*void simLoop(){
    copyState(simState, rocketState);
    simState.delta_t = (float)SIM_TIME_S/SIM_REPS;

    for (int i = 0; i < SIM_REPS; i++){ // IMPORTANT!!! Change this to LAND once you have figured out the physics of coming back down!!!
        stepSim();
    }
}*/

void initSim(){
    Serial.println("init sim");
    simState.ref_area = airBrakeState.getDragForceCoef();
    Serial.println("drag force coefficient: ");
    Serial.println(simState.ref_area);
    simState.ref_area = rocketConfig.getRefArea();
    Serial.println("reference area");
    Serial.println();
    Serial.println("got reference area");
    simState.time = (float)(micros())/1000000.0f;
    simStartTime = simState.time;
    simState.delta_t = 0.10f;
    Serial.println("sim initialized");
}

void updateSim(){
    float last_vel = 0.0f;
    float last_altitude = 0.0f;
    uint count = 0;
    copyState(simState, rocketState);
    last_altitude = simState.getAltitude();
    while ((simState.time-simStartTime) < SIM_TIME_S){
        last_vel = simState.getVZ();
        stepSim();
       // Serial.println("stepping sim");
        
        if (simState.getAltitude() < last_altitude || simState.getVZ() < 0){
            if (count >= 4){
                rocketStatus.apogee = simState.getAltitude();
                Serial.println("Apogee reached");
                break;
            }
            count++;
          
        } else {
            count = 0;
            last_altitude = simState.getAltitude();
        }
        
        
    } 
        //logSimState();
    simState.reset();
    simStepNum = 0;
    Serial.println("sim updated");

}

void stepSim(){ // do i need to update position for intermediaries?
    copyState(k1, simState);

    k1.delta_t = simState.delta_t;
    simState.time += simState.delta_t;

    copyState(k2, k1);
    copyState(k3, k1);
    copyState(k4, k1);

    // Runge Kutta 4th Order approximation

    
    k2.setVZ_Local(k1.getVZ_Local() + k1.getAZ_Local() * k1.delta_t * 0.5f);
    k2.setAZ_Local((-0.5 * getAirDensity() * k1.getVZ_Local() * abs(k1.getVZ_Local()) * simState.drag_coefficient * simState.ref_area/ k1.getMass()) + getThrust()/k1.getMass());


    k2.globalizeVelocity();
    k2.globalizeAcceleration();


    k2.setAZ(k2.getAZ()-(float)GRAVITY);

    
    k2.localizeVelocity();
    k2.localizeAcceleration();


    k3.setVZ_Local(k1.getVZ_Local() + k2.getAZ_Local() * k1.delta_t * 0.5f);
    k3.setAZ_Local((-0.5 * getAirDensity() * k2.getVZ_Local() * abs(k2.getVZ_Local()) * simState.drag_coefficient * simState.ref_area / k1.getMass()) + getThrust()/k2.getMass());


    k3.globalizeVelocity();
    k3.globalizeAcceleration();

    k3.setAZ(k3.getAZ()-(float)GRAVITY);

    k3.localizeVelocity();
    k3.localizeAcceleration();

    k4.setVZ_Local(k1.getVZ_Local() + k3.getAZ_Local() * k1.delta_t);
    k4.setAZ_Local((-0.5 * getAirDensity() * k3.getVZ_Local() * abs(k3.getVZ_Local()) * simState.drag_coefficient * simState.ref_area /k1.getMass()) + getThrust()/k2.getMass());

    k4.globalizeVelocity();
    k4.globalizeAcceleration();
    
    k4.setAZ(k4.getAZ()-(float)GRAVITY);

    k4.localizeVelocity();
    k4.localizeAcceleration();

    k1.updatePos();
    k2.updatePos();
    k3.updatePos();
    k4.updatePos();

    simState.setAltitude(simState.getAltitude() + (float)(1.0f/6.0f) * (k1.getVZ() + 2 * k2.getVZ() + 2 * k3.getVZ() + k4.getVZ()) * simState.delta_t);
    simState.setVZ_Local(simState.getVZ_Local() + (float)(1.0f/6.0f) * (k1.getAZ_Local() + 2 * k2.getAZ_Local() + 2* k3.getAZ_Local() + k4.getAZ_Local()) * simState.delta_t);
    

    simState.globalizeVelocity();

    simState.setAZ_Local((-0.5 * getAirDensity() * simState.getVZ_Local() * abs(simState.getVZ_Local()) * simState.drag_coefficient * simState.ref_area /simState.getMass()));
   
    // gotta set these to zero to avoid fucking up next round of simulation.
    simState.setAX_Local(0);
    simState.setAY_Local(0);

    simState.globalizeAcceleration();

    simState.setAZ(simState.getAZ()-float(GRAVITY));

    simState.localizeVelocity();
    simState.localizeAcceleration();

    k1.reset();
    k2.reset();
    k3.reset();
    k4.reset();
    
    //simState.updatePos();
/*
   Serial.print("V Local: ");
    Serial.print(simState.getVZ_Local());
    Serial.print(", ");
    Serial.print(simState.getVX_Local());
    Serial.print(", ");
    Serial.println(simState.getVY_Local());
    Serial.print(" A Local: ");
    Serial.print(simState.getAZ_Local());
    Serial.print(", ");
    Serial.print(simState.getAX_Local());
    Serial.print(", ");
    Serial.println(simState.getAY_Local());
    Serial.print(" Air density: ");
   // Serial.print(getAirDensity());

    Serial.print(" V: ");
    Serial.print(simState.getVZ());
    Serial.print(", ");
    Serial.print(simState.getVX());
    Serial.print(", ");
    Serial.println(simState.getVY());
    Serial.print(" A: ");
    Serial.print(simState.getAZ());
    Serial.print(", ");
    Serial.print(simState.getAX());
    Serial.print(", ");
    Serial.println(simState.getAY());

    Serial.print(" altitude: ");
    Serial.println(simState.getAltitude());
    Serial.print(" time: ");
    Serial.println(simState.time);
*/
    simStepNum++;
}


// !!!!!
float getAirDensity(){ // IMPORTANT!!! fix this, should not be defined here, should have option to get sim air density
   // Serial.println(simState.getBaroPressure());
   // Serial.println(simState.getBaroTemperature());
    //return ((28.97 * rocketState.getBaroPressure()) / (pow(8.31432, -3) * rocketState.getBaroTemperature())); // rho = MP/RT, gas density equation
    return(1.20f);
}

float getThrust(){ // get thrust from thrust curve, need to implement this, too tired
    return 0.0f;
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
    simState.reset();
    simState.setAltitude(42.95);
    simState.setVZ_Local(73.93);
    simState.setAZ_Local(-24.3);
    simState.stateType = SIM;
    //Serial.println(simState.getVZ_Local());
    //Serial.println(simState.getVZ());
    simState.delta_t = 0.05;
    lastVel = simState.getVZ();
    //simState.updateState();

   simState.setQuatW(1);
    simState.setQuatX(0.0);

    simState.globalizeVelocity();

    simState.globalizeAcceleration();
    simState.setAZ(simState.getAZ()-(float)GRAVITY);
    Serial.print("initial global accel: ");
    Serial.println(simState.getAZ());
    simState.localizeAcceleration();
    Serial.print("initial local accel: ");
    Serial.println(simState.getAZ_Local());
    

    simState.time = (float)(micros()/1000000.0f);
    simStartTime = simState.time;
    delay(100);
    
    while (1){
    if ((simState.time - simStartTime) < SIM_TIME_S){
        stepSim();
        Serial.print(simState.time - simStartTime + 1.14);
        Serial.print(", ");
        Serial.print(simState.getAltitude());
        Serial.print(", ");
        Serial.print(simState.getAZ());
        Serial.print(", ");
        Serial.println(simState.getVZ());
        lastVel = simState.getVZ();
        if (simState.getVZ() < 0 && lastVel >= 0){
            apogee = simState.getAltitude();
        }
        delay(300);
        
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
    baro_altitude = 0.0f;
    altitude = 0.0f;
    baro_pressure = 0.0f;
   // baroTemperature = 0.0f;
   // dragCoefficient = 0.0f;
    time = 0;
    //delta_t = 0;
}
