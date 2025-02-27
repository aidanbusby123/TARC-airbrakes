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
            rocketStateHistory[rocketStateHistory_index].time = rocketState.time;

            rocketStateHistory[rocketStateHistory_index].ax = rocketState.getAX();
            rocketStateHistory[rocketStateHistory_index].ay = rocketState.getAY();
            rocketStateHistory[rocketStateHistory_index].az = rocketState.getAZ();

            rocketStateHistory[rocketStateHistory_index].ax_local = rocketState.getAX_Local();
            rocketStateHistory[rocketStateHistory_index].ay_local = rocketState.getAY_Local();
            rocketStateHistory[rocketStateHistory_index].az_local = rocketState.getAZ_Local();

            rocketStateHistory[rocketStateHistory_index].pitch = rocketState.getPitch();
            rocketStateHistory[rocketStateHistory_index].roll = rocketState.getRoll();
            rocketStateHistory[rocketStateHistory_index].yaw = rocketState.getYaw();

            rocketStateHistory[rocketStateHistory_index].vx = rocketState.getVX();
            rocketStateHistory[rocketStateHistory_index].vy = rocketState.getVY();
            rocketStateHistory[rocketStateHistory_index].vz = rocketState.getVZ();

            rocketStateHistory[rocketStateHistory_index].vx_local = rocketState.getVX_Local();
            rocketStateHistory[rocketStateHistory_index].vy_local = rocketState.getVY_Local();
            rocketStateHistory[rocketStateHistory_index].vz_local = rocketState.getVZ_Local();

            rocketStateHistory[rocketStateHistory_index].x = rocketState.getX();
            rocketStateHistory[rocketStateHistory_index].y = rocketState.getY();
            rocketStateHistory[rocketStateHistory_index].z = rocketState.getZ();

            rocketStateHistory[rocketStateHistory_index].qw = rocketState.getQuatW();
            rocketStateHistory[rocketStateHistory_index].qx = rocketState.getQuatX();
            rocketStateHistory[rocketStateHistory_index].qy = rocketState.getQuatY();
            rocketStateHistory[rocketStateHistory_index].qz = rocketState.getQuatZ();

            rocketStateHistory[rocketStateHistory_index].baro_altitude = rocketState.getBaroAltitude();

            rocketStateHistory[rocketStateHistory_index].altitude = rocketState.getAltitude();

            rocketStateHistory[rocketStateHistory_index].baro_pressure = rocketState.getBaroPressure();

            rocketStateHistory[rocketStateHistory_index].baro_temperature = rocketState.getBaroTemperature();

            memcpy(&rocketStateStruct, &rocketStateHistory[rocketStateHistory_index], sizeof(rocketStateHistory[rocketStateHistory_index]));
            rocketStateStruct.apogee = rocketStatus.apogee;

            rocketStateHistory_index++;
                    
            if (rocketStateHistory_index == rocketStateHistory_size)
            {
                for (int i = 0; i < rocketStateHistory_size; i++)
                { // copy current simStateHistory to temp
                    rocketStateHistoryTemp[i].time = rocketStateHistory[i].time;
                    rocketStateHistoryTemp[i].ax = rocketStateHistory[i].ax;
                    rocketStateHistoryTemp[i].ay = rocketStateHistory[i].ay;
                    rocketStateHistoryTemp[i].az = rocketStateHistory[i].az;

                    rocketStateHistoryTemp[i].ax_local = rocketStateHistory[i].ax_local;
                    rocketStateHistoryTemp[i].ay_local = rocketStateHistory[i].ay_local;
                    rocketStateHistoryTemp[i].az_local = rocketStateHistory[i].az_local;

                    rocketStateHistoryTemp[i].pitch = rocketStateHistory[i].pitch;
                    rocketStateHistoryTemp[i].roll = rocketStateHistory[i].roll;
                    rocketStateHistoryTemp[i].yaw = rocketStateHistory[i].yaw;

                    rocketStateHistoryTemp[i].vx = rocketStateHistory[i].vx;
                    rocketStateHistoryTemp[i].vy = rocketStateHistory[i].vy;
                    rocketStateHistoryTemp[i].vz = rocketStateHistory[i].vz;

                    rocketStateHistoryTemp[i].vx_local = rocketStateHistory[i].vx_local;
                    rocketStateHistoryTemp[i].vy_local = rocketStateHistory[i].vy_local;
                    rocketStateHistoryTemp[i].vz_local = rocketStateHistory[i].vz_local;

                    rocketStateHistoryTemp[i].x = rocketStateHistory[i].x;
                    rocketStateHistoryTemp[i].y = rocketStateHistory[i].y;
                    rocketStateHistoryTemp[i].z = rocketStateHistory[i].z;

                    rocketStateHistoryTemp[i].qw = rocketStateHistory[i].qw;
                    rocketStateHistoryTemp[i].qx = rocketStateHistory[i].qx;
                    rocketStateHistoryTemp[i].qy = rocketStateHistory[i].qy;
                    rocketStateHistoryTemp[i].qz = rocketStateHistory[i].qz;

                    rocketStateHistoryTemp[i].baro_altitude = rocketStateHistory[i].baro_altitude;

                    rocketStateHistoryTemp[i].altitude = rocketStateHistory[i].altitude;

                    rocketStateHistoryTemp[i].baro_pressure = rocketStateHistory[i].baro_pressure;

                    rocketStateHistoryTemp[i].baro_temperature = rocketStateHistory[i].baro_temperature;
                }
            
            delete rocketStateHistory;
            rocketStateHistory = nullptr;
            rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
            rocketStateHistory_index = 0;
            }
    } else if (rocketStateHistory_index == rocketStateHistory_size)
        {
            for (int i = 0; i < simStateHistory_size; i++)
            { // copy current simStateHistory to temp
                rocketStateHistoryTemp[i].time = rocketStateHistory[i].time;
                rocketStateHistoryTemp[i].ax = rocketStateHistory[i].ax;
                rocketStateHistoryTemp[i].ay = rocketStateHistory[i].ay;
                rocketStateHistoryTemp[i].az = rocketStateHistory[i].az;

                rocketStateHistoryTemp[i].ax_local = rocketStateHistory[i].ax_local;
                rocketStateHistoryTemp[i].ay_local = rocketStateHistory[i].ay_local;
                rocketStateHistoryTemp[i].az_local = rocketStateHistory[i].az_local;

                rocketStateHistoryTemp[i].pitch = rocketStateHistory[i].pitch;
                rocketStateHistoryTemp[i].roll = rocketStateHistory[i].roll;
                rocketStateHistoryTemp[i].yaw = rocketStateHistory[i].yaw;

                rocketStateHistoryTemp[i].vx = rocketStateHistory[i].vx;
                rocketStateHistoryTemp[i].vy = rocketStateHistory[i].vy;
                rocketStateHistoryTemp[i].vz = rocketStateHistory[i].vz;

                rocketStateHistoryTemp[i].vx_local = rocketStateHistory[i].vx_local;
                rocketStateHistoryTemp[i].vy_local = rocketStateHistory[i].vy_local;
                rocketStateHistoryTemp[i].vz_local = rocketStateHistory[i].vz_local;

                rocketStateHistoryTemp[i].x = rocketStateHistory[i].x;
                rocketStateHistoryTemp[i].y = rocketStateHistory[i].y;
                rocketStateHistoryTemp[i].z = rocketStateHistory[i].z;

                rocketStateHistoryTemp[i].qw = rocketStateHistory[i].qw;
                rocketStateHistoryTemp[i].qx = rocketStateHistory[i].qx;
                rocketStateHistoryTemp[i].qy = rocketStateHistory[i].qy;
                rocketStateHistoryTemp[i].qz = rocketStateHistory[i].qz;

                rocketStateHistoryTemp[i].baro_altitude = rocketStateHistory[i].baro_altitude;

                rocketStateHistoryTemp[i].altitude = rocketStateHistory[i].altitude;

                rocketStateHistoryTemp[i].baro_pressure = rocketStateHistory[i].baro_pressure;

                rocketStateHistoryTemp[i].baro_temperature = rocketStateHistory[i].baro_temperature;
            }
            
            delete rocketStateHistory;
            rocketStateHistory = nullptr;
            rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
            rocketStateHistory_index = 0;
        }
    } else { // rocket not on pad
        if (rocketStateHistory_index < rocketStateHistory_size){
            rocketStateHistory[rocketStateHistory_index].time = rocketState.time;

            rocketStateHistory[rocketStateHistory_index].ax = rocketState.getAX();
            rocketStateHistory[rocketStateHistory_index].ay = rocketState.getAY();
            rocketStateHistory[rocketStateHistory_index].az = rocketState.getAZ();

            rocketStateHistory[rocketStateHistory_index].ax_local = rocketState.getAX_Local();
            rocketStateHistory[rocketStateHistory_index].ay_local = rocketState.getAY_Local();
            rocketStateHistory[rocketStateHistory_index].az_local = rocketState.getAZ_Local();

            rocketStateHistory[rocketStateHistory_index].pitch = rocketState.getPitch();
            rocketStateHistory[rocketStateHistory_index].roll = rocketState.getRoll();
            rocketStateHistory[rocketStateHistory_index].yaw = rocketState.getYaw();

            rocketStateHistory[rocketStateHistory_index].vx = rocketState.getVX();
            rocketStateHistory[rocketStateHistory_index].vy = rocketState.getVY();
            rocketStateHistory[rocketStateHistory_index].vz = rocketState.getVZ();

            rocketStateHistory[rocketStateHistory_index].vx_local = rocketState.getVX_Local();
            rocketStateHistory[rocketStateHistory_index].vy_local = rocketState.getVY_Local();
            rocketStateHistory[rocketStateHistory_index].vz_local = rocketState.getVZ_Local();

            rocketStateHistory[rocketStateHistory_index].x = rocketState.getX();
            rocketStateHistory[rocketStateHistory_index].y = rocketState.getY();
            rocketStateHistory[rocketStateHistory_index].z = rocketState.getZ();

            rocketStateHistory[rocketStateHistory_index].qw = rocketState.getQuatW();
            rocketStateHistory[rocketStateHistory_index].qx = rocketState.getQuatX();
            rocketStateHistory[rocketStateHistory_index].qy = rocketState.getQuatY();
            rocketStateHistory[rocketStateHistory_index].qz = rocketState.getQuatZ();

            rocketStateHistory[rocketStateHistory_index].baro_altitude = rocketState.getBaroAltitude();

            rocketStateHistory[rocketStateHistory_index].altitude = rocketState.getAltitude();

            rocketStateHistory[rocketStateHistory_index].baro_pressure = rocketState.getBaroPressure();

            rocketStateHistory[rocketStateHistory_index].baro_temperature = rocketState.getBaroTemperature();
            
            memcpy(&rocketStateStruct, &rocketStateHistory[rocketStateHistory_index], sizeof(rocketStateHistory[rocketStateHistory_index]));
            rocketStateStruct.apogee = rocketStatus.apogee;

            rocketStateHistory_index++;
            if (rocketStateHistory_index == rocketStateHistory_size){
                writeRocketStateLog();
                delete rocketStateHistory;
                rocketStateHistory = nullptr;
                rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
                rocketStateHistory_index = 0;
            }
        } else if (rocketStateHistory_index == rocketStateHistory_size){
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
            simStateHistory[simStateHistory_index].time = simState.time;
    
            simStateHistory[simStateHistory_index].ax = simState.getAX();
            simStateHistory[simStateHistory_index].ay = simState.getAY();
            simStateHistory[simStateHistory_index].az = simState.getAZ();

            simStateHistory[simStateHistory_index].ax_local = simState.getAX_Local();
            simStateHistory[simStateHistory_index].ay_local = simState.getAY_Local();
            simStateHistory[simStateHistory_index].az_local = simState.getAZ_Local();

            simStateHistory[simStateHistory_index].pitch = simState.getPitch();
            simStateHistory[simStateHistory_index].roll = simState.getRoll();
            simStateHistory[simStateHistory_index].yaw = simState.getYaw();

            simStateHistory[simStateHistory_index].vx = simState.getVX();
            simStateHistory[simStateHistory_index].vy = simState.getVY();
            simStateHistory[simStateHistory_index].vz = simState.getVZ();

            simStateHistory[simStateHistory_index].vx_local = simState.getVX_Local();
            simStateHistory[simStateHistory_index].vy_local = simState.getVY_Local();
            simStateHistory[simStateHistory_index].vz_local = simState.getVZ_Local();

            simStateHistory[simStateHistory_index].x = simState.getX();
            simStateHistory[simStateHistory_index].y = simState.getY();
            simStateHistory[simStateHistory_index].z = simState.getZ();

            simStateHistory[simStateHistory_index].qw = simState.getQuatW();
            simStateHistory[simStateHistory_index].qx = simState.getQuatX();
            simStateHistory[simStateHistory_index].qy = simState.getQuatY();
            simStateHistory[simStateHistory_index].qz = simState.getQuatZ();

            simStateHistory[simStateHistory_index].baro_altitude = simState.getBaroAltitude();

            simStateHistory[simStateHistory_index].altitude = simState.getAltitude();

            simStateHistory[simStateHistory_index].baro_pressure = simState.getBaroPressure();

            simStateHistory[simStateHistory_index].baro_temperature = simState.getBaroTemperature();

            simStateHistory_index++;

            if (simStateHistory_index == simStateHistory_size)
            {
                for (int i = 0; i < simStateHistory_size; i++)
                { // copy current simStateHistory to temp
                    simStateHistoryTemp[i].time = simStateHistory[i].time;
                    simStateHistoryTemp[i].ax = simStateHistory[i].ax;
                    simStateHistoryTemp[i].ay = simStateHistory[i].ay;
                    simStateHistoryTemp[i].az = simStateHistory[i].az;

                    simStateHistoryTemp[i].ax_local = simStateHistory[i].ax_local;
                    simStateHistoryTemp[i].ay_local = simStateHistory[i].ay_local;
                    simStateHistoryTemp[i].az_local = simStateHistory[i].az_local;

                    simStateHistoryTemp[i].pitch = simStateHistory[i].pitch;
                    simStateHistoryTemp[i].roll = simStateHistory[i].roll;
                    simStateHistoryTemp[i].yaw = simStateHistory[i].yaw;

                    simStateHistoryTemp[i].vx = simStateHistory[i].vx;
                    simStateHistoryTemp[i].vy = simStateHistory[i].vy;
                    simStateHistoryTemp[i].vz = simStateHistory[i].vz;

                    simStateHistoryTemp[i].vx_local = simStateHistory[i].vx_local;
                    simStateHistoryTemp[i].vy_local = simStateHistory[i].vy_local;
                    simStateHistoryTemp[i].vz_local = simStateHistory[i].vz_local;

                    simStateHistoryTemp[i].x = simStateHistory[i].x;
                    simStateHistoryTemp[i].y = simStateHistory[i].y;
                    simStateHistoryTemp[i].z = simStateHistory[i].z;

                    simStateHistoryTemp[i].qw = simStateHistory[i].qw;
                    simStateHistoryTemp[i].qx = simStateHistory[i].qx;
                    simStateHistoryTemp[i].qy = simStateHistory[i].qy;
                    simStateHistoryTemp[i].qz = simStateHistory[i].qz;

                    simStateHistoryTemp[i].baro_altitude = simStateHistory[i].baro_altitude;

                    simStateHistoryTemp[i].altitude = simStateHistory[i].altitude;

                    simStateHistoryTemp[i].baro_pressure = simStateHistory[i].baro_pressure;

                    simStateHistoryTemp[i].baro_temperature = simStateHistory[i].baro_temperature;
                }
                simStateHistoryTemp_index = simStateHistory_size;
            }
        } else if (simStateHistory_index == simStateHistory_size)
        {
            for (int i = 0; i < simStateHistory_size; i++)
            { // copy current simStateHistory to temp
                simStateHistoryTemp[i].time = simStateHistory[i].time;
                simStateHistoryTemp[i].ax = simStateHistory[i].ax;
                simStateHistoryTemp[i].ay = simStateHistory[i].ay;
                simStateHistoryTemp[i].az = simStateHistory[i].az;

                simStateHistoryTemp[i].ax_local = simStateHistory[i].ax_local;
                simStateHistoryTemp[i].ay_local = simStateHistory[i].ay_local;
                simStateHistoryTemp[i].az_local = simStateHistory[i].az_local;

                simStateHistoryTemp[i].pitch = simStateHistory[i].pitch;
                simStateHistoryTemp[i].roll = simStateHistory[i].roll;
                simStateHistoryTemp[i].yaw = simStateHistory[i].yaw;

                simStateHistoryTemp[i].vx = simStateHistory[i].vx;
                simStateHistoryTemp[i].vy = simStateHistory[i].vy;
                simStateHistoryTemp[i].vz = simStateHistory[i].vz;

                simStateHistoryTemp[i].vx_local = simStateHistory[i].vx_local;
                simStateHistoryTemp[i].vy_local = simStateHistory[i].vy_local;
                simStateHistoryTemp[i].vz_local = simStateHistory[i].vz_local;

                simStateHistoryTemp[i].x = simStateHistory[i].x;
                simStateHistoryTemp[i].y = simStateHistory[i].y;
                simStateHistoryTemp[i].z = simStateHistory[i].z;

                simStateHistoryTemp[i].qw = simStateHistory[i].qw;
                simStateHistoryTemp[i].qx = simStateHistory[i].qx;
                simStateHistoryTemp[i].qy = simStateHistory[i].qy;
                simStateHistoryTemp[i].qz = simStateHistory[i].qz;

                simStateHistoryTemp[i].baro_altitude = simStateHistory[i].baro_altitude;

                simStateHistoryTemp[i].altitude = simStateHistory[i].altitude;

                simStateHistoryTemp[i].baro_pressure = simStateHistory[i].baro_pressure;

                simStateHistoryTemp[i].baro_temperature = simStateHistory[i].baro_temperature;
            }
            simStateHistoryTemp_index = simStateHistoryTemp_size;
        }
    }
    else
    {
        if (simStateHistory_index <= simStateHistory_size - 1)
        {
            simStateHistory[simStateHistory_index].time = simState.time;

            simStateHistory[simStateHistory_index].ax = simState.getAX();
            simStateHistory[simStateHistory_index].ay = simState.getAY();
            simStateHistory[simStateHistory_index].az = simState.getAZ();

            simStateHistory[simStateHistory_index].ax_local = simState.getAX_Local();
            simStateHistory[simStateHistory_index].ay_local = simState.getAY_Local();
            simStateHistory[simStateHistory_index].az_local = simState.getAZ_Local();

            simStateHistory[simStateHistory_index].pitch = simState.getPitch();
            simStateHistory[simStateHistory_index].roll = simState.getRoll();
            simStateHistory[simStateHistory_index].yaw = simState.getYaw();

            simStateHistory[simStateHistory_index].vx = simState.getVX();
            simStateHistory[simStateHistory_index].vy = simState.getVY();
            simStateHistory[simStateHistory_index].vz = simState.getVZ();

            simStateHistory[simStateHistory_index].vx_local = simState.getVX_Local();
            simStateHistory[simStateHistory_index].vy_local = simState.getVY_Local();
            simStateHistory[simStateHistory_index].vz_local = simState.getVZ_Local();

            simStateHistory[simStateHistory_index].x = simState.getX();
            simStateHistory[simStateHistory_index].y = simState.getY();
            simStateHistory[simStateHistory_index].z = simState.getZ();

            simStateHistory[simStateHistory_index].qw = simState.getQuatW();
            simStateHistory[simStateHistory_index].qx = simState.getQuatX();
            simStateHistory[simStateHistory_index].qy = simState.getQuatY();
            simStateHistory[simStateHistory_index].qz = simState.getQuatZ();

            simStateHistory[simStateHistory_index].baro_altitude = simState.getBaroAltitude();

            simStateHistory[simStateHistory_index].altitude = simState.getAltitude();

            simStateHistory[simStateHistory_index].baro_pressure = simState.getBaroPressure();

            simStateHistory[simStateHistory_index].baro_temperature = simState.getBaroTemperature();

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
    if (rocketStateHistoryTemp_index>0){
        for (int i = 0; i < rocketStateHistory_size; i++){
            //Serial.write((uint8_t*)&rocketStateHistoryTemp[i], sizeof(rocketStateHistory));
            rocketStateLog.write((uint8_t*)&rocketStateHistoryTemp[i], sizeof(*rocketStateHistory));
        }
        rocketStateHistoryTemp_index = 0;
        delete rocketStateHistoryTemp;
    }
    if (rocketStateHistoryTemp_index == 0){
        if ((rocketStateHistory_index) > 0){
            for (int i = 0; i < rocketStateHistory_index; i++){
                //Serial.write((uint8_t*)&rocketStateHistory[i], sizeof(rocketStateHistory));
                rocketStateLog.write((uint8_t*)&rocketStateHistory[i], sizeof(*rocketStateHistory));
            }
            rocketStateHistory_index = 0;
        }
    }
    //closeLogs();
   // initLogs();
/*V1
        if (rocketStateHistoryTemp_index > 0){
            for (uint i = 0; i < rocketStateHistory_size; i++) {
                rocketStateLog.print(rocketStateHistoryTemp[i].ax, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].ay, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].az, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].ax_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].ay_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].az_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].vx, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].vy, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].vz, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].vx_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].vy_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].vz_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].x, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].y, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].z, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].qw, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].qx, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].qy, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].qz, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].fx_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].fy_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].fz_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistoryTemp[i].baroAltitude, 4);
                rocketStateLog.print(", ");
                rocketStateLog.println(rocketStateHistoryTemp[i].altitude, 4);
                
            }
            delete rocketStateHistoryTemp;
            rocketStateHistoryTemp_index = 0;
        } if (rocketStateHistory_size > 0) {
            for (uint i = 0; i < rocketStateHistory_size; i++){
                rocketStateLog.print(rocketStateHistory[i].ax, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].ay, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].az, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].ax_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].ay_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].az_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].vx, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].vy, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].vz, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].vx_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].vy_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].vz_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].x, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].y, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].z, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].qw, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].qx, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].qy, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].qz, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].fx_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].fy_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].fz_local, 4);
                rocketStateLog.print(", ");
                rocketStateLog.print(rocketStateHistory[i].baroAltitude, 4);
                rocketStateLog.print(", ");
                rocketStateLog.println(rocketStateHistory[i].altitude, 4);
        }
    }
    */
}

void writeSimStateLog()
{
    /* V1
    if (simStateHistoryTemp_index > 0){
        for (uint i = 0; i < simStateHistoryTemp_size; i++){
            simStateLog.print(simStateHistoryTemp[i].ax);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].ay);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].az);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].ax_local);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].ay_local);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].az_local);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].vx);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].vy);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].vz);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].vx_local);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].vy_local);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].vz_local);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].x);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].y);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].z);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].qw);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].qx);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].qy);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].qz);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].fx_local);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].fy_local);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].fz_local);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].baroAltitude);
            simStateLog.print(", ");
            simStateLog.print(simStateHistoryTemp[i].altitude);
            delete simStateHistoryTemp;
            simStateHistoryTemp_index = 0;
        }
    }

    if (simStateHistory_index > 0){
        for (uint i = 0; i < simStateHistory_size; i++)
        {
        simStateLog.print(simStateHistory[i].ax);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].ay);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].az);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].ax_local);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].ay_local);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].az_local);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].vx);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].vy);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].vz);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].vx_local);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].vy_local);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].vz_local);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].x);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].y);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].z);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].qw);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].qx);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].qy);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].qz);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].fx_local);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].fy_local);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].fz_local);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].baroAltitude);
        simStateLog.print(", ");
        simStateLog.print(simStateHistory[i].altitude);
    }
    }
    // rocketStateLog.close();
    */
}

void sendRocketTelemetry(){

    writeSerial(MSG_TYPE_TELEMETRY, sizeof(rocketStateStruct), (uint8_t*)(&rocketStateStruct));
}

void closeLogs()
{
    rocketStateLog.close();
    simStateLog.close();
}