#pragma once
#include "globals.h"

void retractAxisFromLimit(Axis &a);
void startAxisHoming(int axisIdx);   // no guards — for internal calls from auto sequences
void initHoming(int axisIdx);        // user-facing: includes guards and always sequences Z first
void serviceAxes();
void serviceScrew();

// ================== POSITION ROUTINE ==================
void startPositionRoutine();
void servicePositionRoutine();
