#include "auto_jobs.h"
#include "motion.h"
#include "homing.h"
#include "unscrew.h"
#include "grab_place.h"

static bool typeMatches(const char* a, const char* b) { return strcmp(a, b) == 0; }

static int findFirstBitForType(const char* requiredType) {
  for (int i = 0; i < BIT_POSITION_COUNT; i++) {
    if (!bitPositions[i].bitPlaced) continue;
    if (typeMatches(bitPositions[i].bitType, requiredType)) return i;
  }
  return -1;
}

static int findNextNeededBit() {
  for (int bitIdx = 0; bitIdx < BIT_POSITION_COUNT; bitIdx++) {
    if (!bitPositions[bitIdx].bitPlaced) continue;
    const char* bitType = bitPositions[bitIdx].bitType;
    for (int screwIdx = 0; screwIdx < SCREW_POSITION_COUNT; screwIdx++) {
      if (!screwPositions[screwIdx].screwPlaced) continue;
      if (autoAllScrewDone[screwIdx]) continue;
      if (typeMatches(screwPositions[screwIdx].screwType, bitType)) return bitIdx;
    }
  }
  return -1;
}

static int buildScrewQueueForBit(int bitIdx, int* queueOut) {
  int count = 0;
  const char* bitType = bitPositions[bitIdx].bitType;
  for (int screwIdx = 0; screwIdx < SCREW_POSITION_COUNT; screwIdx++) {
    if (!screwPositions[screwIdx].screwPlaced) continue;
    if (autoAllScrewDone[screwIdx]) continue;
    if (!typeMatches(screwPositions[screwIdx].screwType, bitType)) continue;
    if (count < AUTO_UNSCREW_MAX_SCREWS) queueOut[count++] = screwIdx;
  }
  return count;
}

static void printMissingBitWarnings() {
  for (int screwIdx = 0; screwIdx < SCREW_POSITION_COUNT; screwIdx++) {
    if (!screwPositions[screwIdx].screwPlaced) continue;
    int bitIdx = findFirstBitForType(screwPositions[screwIdx].screwType);
    if (bitIdx < 0) {
      Serial.print("AUTO ALL WARNING: no placed bit found for ");
      Serial.print(screwPositions[screwIdx].name);
      Serial.print(" type ");
      Serial.println(screwPositions[screwIdx].screwType);
    }
  }
}

void startMoveToScrewIndex(int idx) {
  Serial.print("AUTO UNSCREW: moving to ");
  Serial.print(screwPositions[idx].name);
  Serial.print(" type ");
  Serial.println(screwPositions[idx].screwType);
  startAbsoluteMoveAll(screwPositions[idx].xMm, screwPositions[idx].yMm, 0.0);
  autoUnscrewState = AUTO_UNSCREW_MOVING_TO_SCREW;
}

void startAutoUnscrewJobMulti(int* screwIndices, int count, bool internalCall) {
  if (!internalCall && autoAllActive()) { Serial.println("AUTO ALL is active. Send S to cancel."); return; }
  if (autoUnscrewActive()) { Serial.println("AUTO UNSCREW: already active. Send S to cancel."); return; }
  if (grabPlaceActive())   { Serial.println("Grab/place is active. Send S to cancel."); return; }
  if (positionActive())    { Serial.println("Position routine is active. Send S to cancel."); return; }
  if (unscrewState != UNSCREW_IDLE) { Serial.println("Unscrew sequence is active. Send S to cancel."); return; }
  if (anyMotionActive())   { Serial.println("Motion already active. Wait or send S."); return; }

  for (int i = 0; i < count; i++) autoUnscrewQueue[i] = screwIndices[i];
  autoUnscrewCount     = count;
  autoUnscrewIndex     = 0;
  autoUnscrewLastJobOk = false;

  Serial.print("AUTO UNSCREW: job started. Screws queued: "); Serial.println(count);
  Serial.println("AUTO UNSCREW: homing Z before first move.");
  startAxisHoming(2);
  autoUnscrewState = AUTO_UNSCREW_WAIT_Z_HOME;
}

void startAutoAllJob() {
  if (autoAllActive())     { Serial.println("AUTO ALL is already active. Send S to cancel."); return; }
  if (autoUnscrewActive()) { Serial.println("AUTO UNSCREW is active. Send S to cancel."); return; }
  if (grabPlaceActive())   { Serial.println("Grab/place is active. Send S to cancel."); return; }
  if (positionActive())    { Serial.println("Position routine is active. Send S to cancel."); return; }
  if (unscrewState != UNSCREW_IDLE) { Serial.println("Unscrew sequence is active. Send S to cancel."); return; }
  if (anyMotionActive())   { Serial.println("Motion already active. Wait or send S."); return; }
  if (!machineHomed)       { Serial.println("AUTO ALL ERROR: machine is not homed yet. Send h first."); return; }

  printMissingBitWarnings();

  bool anyScrew = false;
  for (int i = 0; i < SCREW_POSITION_COUNT; i++) {
    autoAllScrewDone[i] = false;
    if (screwPositions[i].screwPlaced) anyScrew = true;
  }

  if (!anyScrew) { Serial.println("AUTO ALL: no screws are marked as placed."); return; }

  autoAllCurrentBitIdx    = -1;
  autoAllCurrentScrewCount = 0;
  autoAllState             = AUTO_ALL_FIND_NEXT_BIT;
  Serial.println("AUTO ALL: started.");
}

void serviceAutoUnscrewJob() {
  runMotorsNow();
  if (!autoUnscrewActive()) return;

  Axis &z = axes[2];

  if (autoUnscrewState == AUTO_UNSCREW_MOVING_TO_SCREW) {
    if (!axisIsMoving()) autoUnscrewState = AUTO_UNSCREW_START_UNSCREW;
    return;
  }

  if (autoUnscrewState == AUTO_UNSCREW_START_UNSCREW) {
    Serial.print("AUTO UNSCREW: starting unscrew at ");
    Serial.println(screwPositions[autoUnscrewQueue[autoUnscrewIndex]].name);
    startUnscrewSequence();
    autoUnscrewState = (unscrewState != UNSCREW_IDLE) ? AUTO_UNSCREW_RUNNING : AUTO_UNSCREW_ERROR;
    return;
  }

  if (autoUnscrewState == AUTO_UNSCREW_RUNNING) {
    if (unscrewState == UNSCREW_IDLE) {
      Serial.println("AUTO UNSCREW: screw finished.");
      autoUnscrewIndex++;
      autoUnscrewState = (autoUnscrewIndex >= autoUnscrewCount) ? AUTO_UNSCREW_DONE : AUTO_UNSCREW_HOME_Z;
    }
    return;
  }

  if (autoUnscrewState == AUTO_UNSCREW_HOME_Z) {
    Serial.println("AUTO UNSCREW: homing Z before next screw.");
    startAxisHoming(2);
    autoUnscrewState = AUTO_UNSCREW_WAIT_Z_HOME;
    return;
  }

  if (autoUnscrewState == AUTO_UNSCREW_WAIT_Z_HOME) {
    if (!z.homing && !z.retracting && z.motor->distanceToGo() == 0) {
      Serial.println("AUTO UNSCREW: Z homed.");
      startMoveToScrewIndex(autoUnscrewQueue[autoUnscrewIndex]);
    }
    return;
  }

  if (autoUnscrewState == AUTO_UNSCREW_DONE) {
    autoUnscrewLastJobOk = true;
    Serial.println("AUTO UNSCREW: all requested screws completed.");
    clearAutoUnscrewState();
    return;
  }

  if (autoUnscrewState == AUTO_UNSCREW_ERROR) {
    autoUnscrewLastJobOk = false;
    Serial.println("AUTO UNSCREW ERROR: job stopped.");
    clearAutoUnscrewState();
    return;
  }
}

void serviceAutoAllJob() {
  runMotorsNow();
  if (!autoAllActive()) return;

  if (autoAllState == AUTO_ALL_FIND_NEXT_BIT) {
    autoAllCurrentBitIdx = findNextNeededBit();
    if (autoAllCurrentBitIdx < 0) { autoAllState = AUTO_ALL_DONE; return; }

    BitPosition &t      = bitPositions[autoAllCurrentBitIdx];
    autoAllCurrentScrewCount = buildScrewQueueForBit(autoAllCurrentBitIdx, autoAllCurrentScrewQueue);

    if (autoAllCurrentScrewCount <= 0) {
      autoAllState = AUTO_ALL_ERROR;
      Serial.println("AUTO ALL ERROR: selected bit had no screws queued.");
      return;
    }

    Serial.print("AUTO ALL: next bit is "); Serial.print(t.name);
    Serial.print(" type "); Serial.print(t.bitType);
    Serial.print(" for "); Serial.print(autoAllCurrentScrewCount); Serial.println(" screw(s).");
    autoAllState = AUTO_ALL_START_GRAB_BIT;
    return;
  }

  if (autoAllState == AUTO_ALL_START_GRAB_BIT) {
    startGrabSequence(autoAllCurrentBitIdx, true);
    autoAllState = grabPlaceActive() ? AUTO_ALL_WAIT_GRAB_BIT : AUTO_ALL_ERROR;
    return;
  }

  if (autoAllState == AUTO_ALL_WAIT_GRAB_BIT) {
    if (!grabPlaceActive() && !anyMotionActive()) {
      Serial.println("AUTO ALL: bit grabbed.");
      autoAllState = AUTO_ALL_START_SCREWS;
    }
    return;
  }

  if (autoAllState == AUTO_ALL_START_SCREWS) {
    startAutoUnscrewJobMulti(autoAllCurrentScrewQueue, autoAllCurrentScrewCount, true);
    autoAllState = autoUnscrewActive() ? AUTO_ALL_WAIT_SCREWS : AUTO_ALL_ERROR;
    return;
  }

  if (autoAllState == AUTO_ALL_WAIT_SCREWS) {
    if (!autoUnscrewActive() && unscrewState == UNSCREW_IDLE && !anyMotionActive()) {
      if (!autoUnscrewLastJobOk) {
        Serial.println("AUTO ALL ERROR: screw batch did not finish correctly.");
        autoAllState = AUTO_ALL_ERROR;
        return;
      }
      for (int i = 0; i < autoAllCurrentScrewCount; i++) autoAllScrewDone[autoAllCurrentScrewQueue[i]] = true;
      Serial.println("AUTO ALL: screw batch finished.");
      autoAllState = AUTO_ALL_START_PLACE_BIT;
    }
    return;
  }

  if (autoAllState == AUTO_ALL_START_PLACE_BIT) {
    startPlaceSequence(autoAllCurrentBitIdx, true);
    autoAllState = grabPlaceActive() ? AUTO_ALL_WAIT_PLACE_BIT : AUTO_ALL_ERROR;
    return;
  }

  if (autoAllState == AUTO_ALL_WAIT_PLACE_BIT) {
    if (!grabPlaceActive() && !anyMotionActive()) {
      Serial.println("AUTO ALL: bit placed.");
      autoAllCurrentBitIdx    = -1;
      autoAllCurrentScrewCount = 0;
      autoAllState             = AUTO_ALL_FIND_NEXT_BIT;
    }
    return;
  }

  if (autoAllState == AUTO_ALL_DONE) {
    Serial.println("AUTO ALL: complete.");
    clearAutoAllState();
    return;
  }

  if (autoAllState == AUTO_ALL_ERROR) {
    Serial.println("AUTO ALL ERROR: job stopped. Send S or inspect machine.");
    clearAutoAllState();
    return;
  }
}
