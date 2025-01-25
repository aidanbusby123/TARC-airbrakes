#include "main.h"
#include <wiring_private.h>
#include <RTClib.h>

Uart BTSerial(&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

SdFile rocketStateLog;
SdFile simStateLog;

stateHistory *rocketStateHistoryTemp;
stateHistory *simStateHistoryTemp;

uint rocketStateHistoryTemp_index = 0;
uint simStateHistoryTemp_index = 0;
uint rocketStateHistoryTemp_size = 0;
uint simStateHistoryTemp_size = 0;

void SERCOM1_Handler()
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
}

void logRocketState()
{
    if (rocketState.flightPhase == PAD){

        if (rocketStateHistory_index <= rocketStateHistory_size - 1)
        {
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

            rocketStateHistory[rocketStateHistory_index].fx_local = rocketState.getFX_Local();
            rocketStateHistory[rocketStateHistory_index].fy_local = rocketState.getFY_Local();
            rocketStateHistory[rocketStateHistory_index].fz_local = rocketState.getFZ_Local();

            rocketStateHistory[rocketStateHistory_index].baroAltitude = rocketState.getBaroAltitude();

            rocketStateHistory[rocketStateHistory_index].altitude = rocketState.getAltitude();

            rocketStateHistory[rocketStateHistory_index].baroPressure = rocketState.getBaroPressure();

            rocketStateHistory[rocketStateHistory_index].baroTemperature = rocketState.getBaroTemperature();

            rocketStateHistory_index++;
                    
            if (rocketStateHistory_index == rocketStateHistory_size)
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

                    rocketStateHistoryTemp[i].fx_local = rocketStateHistory[i].fx_local;
                    rocketStateHistoryTemp[i].fy_local = rocketStateHistory[i].fy_local;
                    rocketStateHistoryTemp[i].fz_local = rocketStateHistory[i].fz_local;

                    rocketStateHistoryTemp[i].baroAltitude = rocketStateHistory[i].baroAltitude;

                    rocketStateHistoryTemp[i].altitude = rocketStateHistory[i].altitude;

                    rocketStateHistoryTemp[i].baroPressure = rocketStateHistory[i].baroPressure;

                    rocketStateHistoryTemp[i].baroTemperature = rocketStateHistory[i].baroTemperature;
                }
            
            delete rocketStateHistory;
            rocketStateHistory = nullptr;
            rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
            rocketStateHistory_index = 0;
            }
        } else         if (rocketStateHistory_index == rocketStateHistory_size)
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

                rocketStateHistoryTemp[i].fx_local = rocketStateHistory[i].fx_local;
                rocketStateHistoryTemp[i].fy_local = rocketStateHistory[i].fy_local;
                rocketStateHistoryTemp[i].fz_local = rocketStateHistory[i].fz_local;

                rocketStateHistoryTemp[i].baroAltitude = rocketStateHistory[i].baroAltitude;

                rocketStateHistoryTemp[i].altitude = rocketStateHistory[i].altitude;

                rocketStateHistoryTemp[i].baroPressure = rocketStateHistory[i].baroPressure;

                rocketStateHistoryTemp[i].baroTemperature = rocketStateHistory[i].baroTemperature;
            }
            
            delete rocketStateHistory;
            rocketStateHistory = nullptr;
            rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
            rocketStateHistory_index = 0;
        }
    } else { // rocket not on pad
        if (rocketStateHistory_index <= rocketStateHistory_size){
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

            rocketStateHistory[rocketStateHistory_index].fx_local = rocketState.getFX_Local();
            rocketStateHistory[rocketStateHistory_index].fy_local = rocketState.getFY_Local();
            rocketStateHistory[rocketStateHistory_index].fz_local = rocketState.getFZ_Local();

            rocketStateHistory[rocketStateHistory_index].baroAltitude = rocketState.getBaroAltitude();

            rocketStateHistory[rocketStateHistory_index].altitude = rocketState.getAltitude();

            rocketStateHistory[rocketStateHistory_index].baroPressure = rocketState.getBaroPressure();

            rocketStateHistory[rocketStateHistory_index].baroTemperature = rocketState.getBaroTemperature();

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
            simStateHistory[simStateHistory_index].time = micros()/1000000.0f;
    
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

            simStateHistory[simStateHistory_index].fx_local = simState.getFX_Local();
            simStateHistory[simStateHistory_index].fy_local = simState.getFY_Local();
            simStateHistory[simStateHistory_index].fz_local = simState.getFZ_Local();

            simStateHistory[simStateHistory_index].baroAltitude = simState.getBaroAltitude();

            simStateHistory[simStateHistory_index].altitude = simState.getAltitude();

            simStateHistory[simStateHistory_index].baroPressure = simState.getBaroPressure();

            simStateHistory[simStateHistory_index].baroTemperature = simState.getBaroTemperature();

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

                    simStateHistoryTemp[i].fx_local = simStateHistory[i].fx_local;
                    simStateHistoryTemp[i].fy_local = simStateHistory[i].fy_local;
                    simStateHistoryTemp[i].fz_local = simStateHistory[i].fz_local;

                    simStateHistoryTemp[i].baroAltitude = simStateHistory[i].baroAltitude;

                    simStateHistoryTemp[i].altitude = simStateHistory[i].altitude;

                    simStateHistoryTemp[i].baroPressure = simStateHistory[i].baroPressure;

                    simStateHistoryTemp[i].baroTemperature = simStateHistory[i].baroTemperature;
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

                simStateHistoryTemp[i].fx_local = simStateHistory[i].fx_local;
                simStateHistoryTemp[i].fy_local = simStateHistory[i].fy_local;
                simStateHistoryTemp[i].fz_local = simStateHistory[i].fz_local;

                simStateHistoryTemp[i].baroAltitude = simStateHistory[i].baroAltitude;

                simStateHistoryTemp[i].altitude = simStateHistory[i].altitude;

                simStateHistoryTemp[i].baroPressure = simStateHistory[i].baroPressure;

                simStateHistoryTemp[i].baroTemperature = simStateHistory[i].baroTemperature;
            }
            simStateHistoryTemp_index = simStateHistoryTemp_size;
        }
    }
    else
    {
        if (simStateHistory_index <= simStateHistory_size - 1)
        {
            simStateHistory[simStateHistory_index].time = micros()/1000000.0f;

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

            simStateHistory[simStateHistory_index].fx_local = simState.getFX_Local();
            simStateHistory[simStateHistory_index].fy_local = simState.getFY_Local();
            simStateHistory[simStateHistory_index].fz_local = simState.getFZ_Local();

            simStateHistory[simStateHistory_index].baroAltitude = simState.getBaroAltitude();

            simStateHistory[simStateHistory_index].altitude = simState.getAltitude();

            simStateHistory[simStateHistory_index].baroPressure = simState.getBaroPressure();

            simStateHistory[simStateHistory_index].baroTemperature = simState.getBaroTemperature();

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
    char num[4] = {0};
    for (int i = 0; i <= 999; i++){
        itoa(i, num, 10);
        if (rocketStateLog.exists(strcat(strcat("rocket_state_log", num), ".csv"))){
            continue;
        } else {
            if (!rocketStateLog.open("rocket_state_log.csv", O_RDWR | O_CREAT))
            {
                sd.errorHalt("opening rocket state log failed");
            }
        }
    }
    for (int i = 0; i <= 999; i++){
        itoa(i, num, 10);
        if (simStateLog.exists(strcat(strcat("sim_state_log", num), ".csv"))){
            continue;
        } else {
            if (!rocketStateLog.open("sim_state_log.csv", O_RDWR | O_CREAT))
            {
                sd.errorHalt("opening sim state log failed");
            }
        }
    }

    rocketStateHistory = new stateHistory[STATEHISTORY_SIZE];
    rocketStateHistory_size = STATEHISTORY_SIZE;

    rocketStateHistoryTemp = new stateHistory[STATEHISTORY_SIZE];
    rocketStateHistoryTemp_size = STATEHISTORY_SIZE;

    simStateHistory = new stateHistory[STATEHISTORY_SIZE];
    simStateHistory_size = STATEHISTORY_SIZE;

    simStateHistoryTemp = new stateHistory[STATEHISTORY_SIZE];
    simStateHistoryTemp_size = STATEHISTORY_SIZE;
}

void writeRocketStateLog()
{

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
                delete rocketStateHistoryTemp;
                rocketStateHistoryTemp_index = 0;
            }
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
}

void writeSimStateLog()
{
    if (simStateHistoryTemp_index > 0){

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
}

void closeLogs()
{
    rocketStateLog.close();
    simStateLog.close();
}