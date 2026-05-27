#pragma once
#include "globals.h"

void startMoveToScrewIndex(int idx);
void startAutoUnscrewJobMulti(int* screwIndices, int count, bool internalCall = false);
void startAutoAllJob();
void serviceAutoUnscrewJob();
void serviceAutoAllJob();
