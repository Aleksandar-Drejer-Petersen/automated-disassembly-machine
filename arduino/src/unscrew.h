#pragma once
#include "globals.h"

float chooseClosestPitch(float measuredPitch);
void  releaseUnscrewControlAfterError(const char* message);
void  startUnscrewSequence();
void  serviceUnscrew();
