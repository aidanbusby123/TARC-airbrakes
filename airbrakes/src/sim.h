#include "main.h"

#define SIM_REPS 100
#define SIM_TIME_S 20
#define DRAG_COEFFICIENT 0.75

extern state simState;

void updateSim();
void stepSim();
void runTestSim();
void calcForces();
float getAirDensity();
float getThrust();
void copyState(state& newState, state& curState);