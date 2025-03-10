#include "main.h"
#include "telemetry.h"
#include "coms.h"
#include <wiring_private.h>
#include <RTClib.h>

//Uart BTSerial(&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

SdFile rocketStateLog;
SdFile simStateLog;

stateHistory *rocketStateHistoryTemp;
stateHistory *simStateHistoryTemp;


stateHistory *rocketStateHistory; // Rocket State history
stateHistory *simStateHistory;    // Simulation state history

stateHistory rocketStateStruct;

uint rocketStateHistoryTemp_index = 0;
uint simStateHistoryTemp_index = 0;
uint rocketStateHistoryTemp_size = 0;
uint simStateHistoryTemp_size = 0;

uint rocketStateHistory_index = 0;
uint rocketStateHistory_size = 0;

uint simStateHistory_index = 0;
uint simStateHistory_size = 0;

/*void SERCOM1_Handler()
{
    BTSerial.IrqHandler();
}

void initBT()
{
    BTSerial.begin(115200);
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);
}

void loopBT()
{
    if (Serial.available())
    {
        BTSerial.write(Serial.read());
    }
    if (BTSerial.available())
    {
        Serial.write(BTSerial.read());
    }
}*/

void logRocketState()
{
    if (rocketState.flightPhase == PAD){

        if (rocketStateHistory_index <= rocketStateHistory_size - 1)
        {
            logState(rocketStateHistory, rocketStateHistory_index, rocketState);

            memcpy(&rocketStateStruct, &rocketStateHistory[rocketStateHistory_index], sizeof(rocketStateHistory[rocketStateHistory_index]));

            rocketStateHistory_index++;
                    
            if (rocketStateHistory_index == rocketStateHistory_size)
            {
            logTempState(rocketStateHistoryTemp, rocketStateHistory, rocketStateHistory_size);
            
            delete rocketStateHistory;
            rocketStateHistory = nullptr;
            rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
            rocketStateHistory_index = 0;
            }
    } else if (rocketStateHistory_index == rocketStateHistory_size-1)
        {

            logTempState(rocketStateHistoryTemp, rocketStateHistory, rocketStateHistory_size);
            
            delete rocketStateHistory;
            rocketStateHistory = nullptr;
            rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
            rocketStateHistory_index = 0;
        }
    } else { // rocket not on pad
        if (rocketStateHistory_index < rocketStateHistory_size){
            logState(rocketStateHistory, rocketStateHistory_index, rocketState);
            
            
            memcpy(&rocketStateStruct, &rocketStateHistory[rocketStateHistory_index], sizeof(rocketStateHistory[rocketStateHistory_index]));
            

            rocketStateHistory_index++;
            if (rocketStateHistory_index == rocketStateHistory_size){
                writeRocketStateLog();
                delete rocketStateHistory;
                rocketStateHistory = nullptr;
                rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
                rocketStateHistory_index = 0;
            }
        } else if (rocketStateHistory_index == rocketStateHistory_size-1){
            writeRocketStateLog();
            delete rocketStateHistory;
            rocketStateHistory = nullptr;
            rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
            rocketStateHistory_index = 0;
        }
    }
}
void logSimState()
{
    if (rocketState.flightPhase == PAD){

        if (simStateHistory_index <= simStateHistory_size - 1)
        {
            logState(simStateHistory, simStateHistory_index, simState);

            if (simStateHistory_index == simStateHistory_size)
            {
                logTempState(simStateHistoryTemp, simStateHistory, simStateHistory_size);
                simStateHistoryTemp_index = simStateHistory_size;
            }
        } else if (simStateHistory_index == simStateHistory_size)
        {
            logTempState(simStateHistoryTemp, simStateHistory, simStateHistory_size);
            simStateHistoryTemp_index = simStateHistoryTemp_size;
        }
    }
    else
    {
        if (simStateHistory_index <= simStateHistory_size - 1)
        {
            logState(simStateHistory, simStateHistory_index, simState);

            simStateHistory_index++;
            if (simStateHistory_index == simStateHistory_size){
                writeSimStateLog();
                delete simStateHistory;
                simStateHistory = nullptr;
                simStateHistory = new stateHistory[STATEHISTORY_SIZE];
                simStateHistory_index = 0;
            }
        } else if (simStateHistory_index == simStateHistory_size){
            writeSimStateLog();
            delete simStateHistory;
            simStateHistory = nullptr;
            simStateHistory = new stateHistory[STATEHISTORY_SIZE];
            simStateHistory_index = 0;
        }
    }
}

void initLogs()
{
    int filecount = 1;
    char rocketStateLogName[64] = "rocket_state_log.dat";
    char simStateLogName[64] = "sim_state_log.dat";
    rocketStateLog.open("/");
    if (rocketStateLog.exists(rocketStateLogName)){
        //Serial.println("already exists");
        for (filecount = 1; filecount < 999; filecount++){
            snprintf(rocketStateLogName, 64, "rocket_state_log%d.dat", filecount);
            if (!rocketStateLog.exists(rocketStateLogName)){
                //Serial.println(rocketStateLogName);
                if (!rocketStateLog.open(rocketStateLogName, O_RDWR | O_CREAT)){
                    sd.errorHalt("unable to open rocket state log, not enough space");
                } else {
                    break;
                }
            }
        }
        if (rocketStateLog.isOpen() == false){
            sd.errorHalt("unable to open rocket state log, not open");
        }
    } else {
        if (!rocketStateLog.open(rocketStateLogName, O_RDWR | O_CREAT)){
            sd.errorHalt("unable to open rocket state log, unknown error");
        }
    }

    if (simStateLog.exists(simStateLogName)){
       // Serial.println("sim already exists");
        for (filecount = 1; filecount < 999; filecount++){
            snprintf(simStateLogName, 64, "sim_state_log%d.dat", filecount);
            if (!simStateLog.exists(simStateLogName)){
               // Serial.println(simStateLogName);
                if (!simStateLog.open(simStateLogName, O_RDWR | O_CREAT)){
                    sd.errorHalt("unable to open sim state log, not enough space");
                } else {
                    break;
                }
            }
        }
        if (simStateLog.isOpen() == false){
            sd.errorHalt("unable to open sim state log, not open");
        }
    } else {
        if (!simStateLog.open(simStateLogName, O_RDWR | O_CREAT)){
            sd.errorHalt("unable to open sim state log, unknown error");
        }
    }

    rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
    rocketStateHistory_size = STATEHISTORY_SIZE;
    //Serial.println("log one initialized");

    rocketStateHistoryTemp = new stateHistory[STATEHISTORY_SIZE];
    rocketStateHistoryTemp_size = STATEHISTORY_SIZE;
    //Serial.println("log two initialized");
/*
    simStateHistory = new stateHistory[STATEHISTORY_SIZE];
    simStateHistory_size = STATEHISTORY_SIZE;
    Serial.println("log three initialized");
    simStateHistoryTemp = new stateHistory[STATEHISTORY_SIZE];
    simStateHistoryTemp_size = STATEHISTORY_SIZE;
    Serial.println("log four initialized");
    */
}

void writeRocketStateLog()
{
    
    if (rocketStateHistoryTemp_index > 0){
        for (int i = 0; i < rocketStateHistory_size; i++){
            //Serial.write((uint8_t*)&rocketStateHistoryTemp[i], sizeof(rocketStateHistory));
            rocketStateLog.write(MSG_START_BYTE);
            rocketStateLog.write(MSG_TYPE_TELEMETRY);
            rocketStateLog.write(sizeof(stateHistory));
            rocketStateLog.write((uint8_t*)&rocketStateHistoryTemp[i], sizeof(*rocketStateHistory));
        }
        rocketStateHistoryTemp_index = 0;
        delete rocketStateHistoryTemp;
    }
    if (rocketStateHistoryTemp_index == 0){
        if ((rocketStateHistory_index) > 0){
            for (int i = 0; i < rocketStateHistory_index; i++){
                //Serial.write((uint8_t*)&rocketStateHistory[i], sizeof(rocketStateHistory));
                rocketStateLog.write(MSG_START_BYTE);
                rocketStateLog.write(MSG_TYPE_TELEMETRY);
                rocketStateLog.write(sizeof(stateHistory));
                rocketStateLog.write((uint8_t*)&rocketStateHistory[i], sizeof(*rocketStateHistory));
            }
            rocketStateHistory_index = 0;
        }
    }
    //closeLogs();
   // initLogs();

}

void writeSimStateLog()
{

}

void sendRocketTelemetry(){

    writeSerial((uint8_t)MSG_TYPE_TELEMETRY, (uint8_t)sizeof(rocketStateStruct), (uint8_t*)(&rocketStateStruct), false);
}

void closeLogs()
{
    rocketStateLog.close();
    simStateLog.close();
}

void logState(stateHistory* destHistory, uint destHistory_index, state sourceState){
    destHistory[destHistory_index].time = sourceState.time;

    destHistory[destHistory_index].ax = sourceState.getAX();
    destHistory[destHistory_index].ay = sourceState.getAY();
    destHistory[destHistory_index].az = sourceState.getAZ();

    destHistory[destHistory_index].ax_local = sourceState.getAX_Local();
    destHistory[destHistory_index].ay_local = sourceState.getAY_Local();
    destHistory[destHistory_index].az_local = sourceState.getAZ_Local();

    destHistory[destHistory_index].pitch = sourceState.getPitch();
    destHistory[destHistory_index].roll = sourceState.getRoll();
    destHistory[destHistory_index].yaw = sourceState.getYaw();

    destHistory[destHistory_index].vx = sourceState.getVX();
    destHistory[destHistory_index].vy = sourceState.getVY();
    destHistory[destHistory_index].vz = sourceState.getVZ();

    destHistory[destHistory_index].vx_local = sourceState.getVX_Local();
    destHistory[destHistory_index].vy_local = sourceState.getVY_Local();
    destHistory[destHistory_index].vz_local = sourceState.getVZ_Local();

    destHistory[destHistory_index].x = sourceState.getX();
    destHistory[destHistory_index].y = sourceState.getY();
    destHistory[destHistory_index].z = sourceState.getZ();

    destHistory[destHistory_index].qw = sourceState.getQuatW();
    destHistory[destHistory_index].qx = sourceState.getQuatX();
    destHistory[destHistory_index].qy = sourceState.getQuatY();
    destHistory[destHistory_index].qz = sourceState.getQuatZ();

    destHistory[destHistory_index].baro_altitude = sourceState.getBaroAltitude();

    destHistory[destHistory_index].apogee = sourceState.getApogee();

    destHistory[destHistory_index].altitude = sourceState.getAltitude();

    destHistory[destHistory_index].baro_pressure = sourceState.getBaroPressure();

    destHistory[destHistory_index].baro_temperature = sourceState.getBaroTemperature();

    destHistory[destHistory_index].flightPhase = sourceState.flightPhase;

}

void logTempState(stateHistory* destHistoryTemp, stateHistory* destHistory, uint destHistory_size){
    for (int i = 0; i < destHistory_size; i++){
        destHistoryTemp[i].time = destHistory[i].time;
    destHistoryTemp[i].ax = destHistory[i].ax;
    destHistoryTemp[i].ay = destHistory[i].ay;
    destHistoryTemp[i].az = destHistory[i].az;

    destHistoryTemp[i].ax_local = destHistory[i].ax_local;
    destHistoryTemp[i].ay_local = destHistory[i].ay_local;
    destHistoryTemp[i].az_local = destHistory[i].az_local;

    destHistoryTemp[i].pitch = destHistory[i].pitch;
    destHistoryTemp[i].roll = destHistory[i].roll;
    destHistoryTemp[i].yaw = destHistory[i].yaw;

    destHistoryTemp[i].vx = destHistory[i].vx;
    destHistoryTemp[i].vy = destHistory[i].vy;
    destHistoryTemp[i].vz = destHistory[i].vz;

    destHistoryTemp[i].vx_local = destHistory[i].vx_local;
    destHistoryTemp[i].vy_local = destHistory[i].vy_local;
    destHistoryTemp[i].vz_local = destHistory[i].vz_local;

    destHistoryTemp[i].x = destHistory[i].x;
    destHistoryTemp[i].y = destHistory[i].y;
    destHistoryTemp[i].z = destHistory[i].z;

    destHistoryTemp[i].qw = destHistory[i].qw;
    destHistoryTemp[i].qx = destHistory[i].qx;
    destHistoryTemp[i].qy = destHistory[i].qy;
    destHistoryTemp[i].qz = destHistory[i].qz;

    destHistoryTemp[i].baro_altitude = destHistory[i].baro_altitude;

    destHistoryTemp[i].apogee = destHistory[i].apogee;

    destHistoryTemp[i].altitude = destHistory[i].altitude;

    destHistoryTemp[i].baro_pressure = destHistory[i].baro_pressure;

    destHistoryTemp[i].baro_temperature = destHistory[i].baro_temperature;

    destHistoryTemp[i].flightPhase = destHistory[i].flightPhase;
}
}