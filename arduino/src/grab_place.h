#pragma once
#include "globals.h"

void startGrabSequence(int bitIdx, bool internalCall = false);
void startPlaceSequence(int bitIdx, bool internalCall = false);
void serviceGrabPlace();
