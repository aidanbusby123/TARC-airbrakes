#include "main.h"

#define SIM_REPS 100
#define SIM_TIME_S 50

extern state simState;

void runSim();
void runTestSim();
void calcForces();
float getAirDensity();
void copyState(state& newState, state& curState);