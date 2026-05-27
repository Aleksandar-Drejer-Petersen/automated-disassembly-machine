#include "homing.h"
#include "motion.h"

void retractAxisFromLimit(Axis &a) {
  a.motor->setMaxSpeed(HOME_SPEED);
  a.motor->setAcceleration(HOME_ACCEL);
  // setCurrentPosition stops the motor and resets the position counter to 0
  a.motor->setCurrentPosition(0);
  a.homing     = false;
  a.retracting = true;
  long retractSteps = mmToSteps(HOMING_RETRACT_MM, a.pitchMm);
  // Move away from home switch (opposite direction to homing)
  long retractTarget = a.dirTowardHome ? -(long)retractSteps : (long)retractSteps;
  a.motor->moveTo(retractTarget);
  Serial.print("LIMIT HIT: Axis "); Serial.print(a.name);
  Serial.println(" stopped. Retracting and resetting to 0.");
}

// No guards — safe for calls from within active auto sequences.
void startAxisHoming(int axisIdx) {
  Axis &a = axes[axisIdx];
  a.motor->setMaxSpeed(HOME_SPEED);
  a.motor->setAcceleration(HOME_ACCEL);
  if (digitalRead(a.limitPin) == LOW) {
    retractAxisFromLimit(a);
  } else {
    a.homing     = true;
    a.retracting = false;
    long hugeMove = a.dirTowardHome ? 1000000L : -1000000L;
    a.motor->move(hugeMove);
  }
}

// User-facing homing command.  Always homes Z first; X and Y start only after Z has
// finished so the axle is never moving sideways while it is still inside a fixture.
void initHoming(int axisIdx) {
  if (autoAllActive())     { Serial.println("AUTO ALL is active. Send S to cancel."); return; }
  if (grabPlaceActive())   { Serial.println("Grab/place is active. Send S to cancel."); return; }
  if (positionActive())    { Serial.println("Position routine is active. Send S to cancel."); return; }
  if (unscrewState != UNSCREW_IDLE) { Serial.println("Unscrew sequence is active. Send S to cancel."); return; }
  if (axisIsMoving())      { Serial.println("Axis already moving. Send S first."); return; }

  machineHomed        = false;
  homingAllInProgress = (axisIdx == -1);

  // Track which X/Y axes follow after Z is done.
  pendingHomingMask = 0;
  if (axisIdx == -1 || axisIdx == 0) pendingHomingMask |= 1;  // X
  if (axisIdx == -1 || axisIdx == 1) pendingHomingMask |= 2;  // Y

  // Always start Z immediately (lifts axle before any horizontal motion).
  startAxisHoming(2);
  Serial.println("Homing: Z first.");
}

void serviceAxes() {
  for (int i = 0; i < 3; i++) {
    Axis &a = axes[i];

    bool skipNormalLimitHandling =
      positionActive() && positionState == POSITION_MOVING_TO_SWITCH &&
      (positionAxisIdx == i || (positionAxisIdx == 3 && i < 2));

    bool limitHit = digitalRead(a.limitPin) == LOW;

    long dist = a.motor->distanceToGo();
    bool movingTowardHome = stepsMoveTowardHome(a, dist);

    if (!skipNormalLimitHandling && limitHit && movingTowardHome && !a.retracting) {
      retractAxisFromLimit(a);
    }

    if (a.retracting && a.motor->distanceToGo() == 0) {
      a.retracting = false;
      a.homing     = false;
      a.motor->setCurrentPosition(0);
      Serial.print("Axis "); Serial.print(a.name); Serial.println(" safe. Position = 0.");
    }
  }

  // Once Z finishes, release any X/Y axes that were deferred.
  if (pendingHomingMask != 0) {
    Axis &z = axes[2];
    if (!z.homing && !z.retracting && z.motor->distanceToGo() == 0) {
      Serial.println("Homing: Z done. Starting X/Y.");
      for (int i = 0; i < 2; i++) {
        if (pendingHomingMask & (1 << i)) startAxisHoming(i);
      }
      pendingHomingMask = 0;
    }
  }

  // Only declare "all homed" once no axes are pending and all have finished.
  if (homingAllInProgress && pendingHomingMask == 0) {
    bool allDone = true;
    for (int i = 0; i < 3; i++) {
      if (axes[i].homing || axes[i].retracting || axes[i].motor->distanceToGo() != 0)
        allDone = false;
    }
    if (allDone) {
      homingAllInProgress = false;
      machineHomed        = true;
      Serial.println("HOME: all axes homed.");
      Serial.println("SYSTEM IS HOMED");
    }
  }
}

void serviceScrew() {
  screwStepper.run();
  static bool wasMoving = false;
  bool isMoving = screwStepper.distanceToGo() != 0;
  if (wasMoving && !isMoving && unscrewState == UNSCREW_IDLE && !grabPlaceActive()) {
    Serial.println("Screw done.");
  }
  wasMoving = isMoving;
}

// ================== POSITION ROUTINE ==================
static void printPositionResults() {
  Serial.println();
  Serial.println("POSITION RESULT");
  Serial.println("Movement to home switch:");
  Serial.print("  X moved "); Serial.print(positionMoveToSwitchMm[0], 3); Serial.println(" mm");
  Serial.print("  Y moved "); Serial.print(positionMoveToSwitchMm[1], 3); Serial.println(" mm");
  Serial.print("  Z moved "); Serial.print(positionMoveToSwitchMm[2], 3); Serial.println(" mm");
  Serial.println("Suggested coordinates to return to the original start position after homing:");
  Serial.print("  to ");
  Serial.print(positionReturnCoordinateMm[0], 3); Serial.print(" ");
  Serial.print(positionReturnCoordinateMm[1], 3); Serial.print(" ");
  Serial.println(positionReturnCoordinateMm[2], 3);
  Serial.println();
}

static void startNextPositionAxis() {
  // After Z (index 0) finishes, home X and Y simultaneously (positionAxisIdx = 3 = parallel sentinel).
  if (positionOrderIndex == 1) {
    positionAxisIdx          = 3;
    positionParallelDone     = 0;
    positionParallelMeasured = 0;
    Serial.println("POSITION: Z done. Starting X and Y simultaneously.");
    for (int i = 0; i < 2; i++) {
      Axis &a = axes[i];
      positionStartSteps[i] = a.motor->currentPosition();
      a.motor->setMaxSpeed(HOME_SPEED);
      a.motor->setAcceleration(HOME_ACCEL);
      if (digitalRead(a.limitPin) == LOW) {
        positionMoveToSwitchMm[i]     = 0.0;
        positionReturnCoordinateMm[i] = 0.0;
        retractAxisFromLimit(a);
        positionParallelMeasured |= (1 << i);
      } else {
        a.homing     = true;
        a.retracting = false;
        long hugeMove = a.dirTowardHome ? 1000000L : -1000000L;
        a.motor->move(hugeMove);
      }
    }
    positionState = POSITION_MOVING_TO_SWITCH;
    return;
  }

  if (positionOrderIndex >= 3) {
    printPositionResults();
    clearPositionState();
    return;
  }

  positionAxisIdx = positionAxisOrder[positionOrderIndex];
  Axis &a         = axes[positionAxisIdx];

  positionStartSteps[positionAxisIdx] = a.motor->currentPosition();
  a.motor->setMaxSpeed(HOME_SPEED);
  a.motor->setAcceleration(HOME_ACCEL);
  Serial.print("POSITION: Homing "); Serial.println(a.name);

  if (digitalRead(a.limitPin) == LOW) {
    positionMoveToSwitchMm[positionAxisIdx]     = 0.0;
    positionReturnCoordinateMm[positionAxisIdx] = 0.0;
    retractAxisFromLimit(a);
    positionState = POSITION_WAIT_RETRACT;
    return;
  }

  a.homing     = true;
  a.retracting = false;
  long hugeMove = a.dirTowardHome ? 1000000L : -1000000L;
  a.motor->move(hugeMove);
  positionState = POSITION_MOVING_TO_SWITCH;
}

void startPositionRoutine() {
  if (autoAllActive())     { Serial.println("AUTO ALL is active. Send S to cancel."); return; }
  if (autoUnscrewActive()) { Serial.println("AUTO UNSCREW is active. Send S to cancel."); return; }
  if (grabPlaceActive())   { Serial.println("Grab/place is active. Send S to cancel."); return; }
  if (positionActive())    { Serial.println("Position routine is already active."); return; }
  if (unscrewState != UNSCREW_IDLE) { Serial.println("Unscrew sequence is active. Send S to cancel."); return; }
  if (anyMotionActive())   { Serial.println("Motion already active. Wait or send S."); return; }

  for (int i = 0; i < 3; i++) {
    positionStartSteps[i]           = 0;
    positionMoveToSwitchMm[i]       = 0.0;
    positionReturnCoordinateMm[i]   = 0.0;
  }

  positionOrderIndex = 0;
  positionAxisIdx    = -1;
  positionState      = POSITION_MOVING_TO_SWITCH;
  Serial.println("POSITION: Starting position detection.");
  Serial.println("POSITION: Sequence = Z, then Y, then X.");
  startNextPositionAxis();
}

void servicePositionRoutine() {
  runMotorsNow();
  if (!positionActive()) return;

  // ── Parallel X+Y phase (positionAxisIdx == 3) ──────────────────────────────
  if (positionAxisIdx == 3) {
    for (int i = 0; i < 2; i++) {
      if (positionParallelDone & (1 << i)) continue;
      Axis &a = axes[i];

      if (!(positionParallelMeasured & (1 << i))) {
        if (digitalRead(a.limitPin) == LOW && !a.retracting) {
          long  movedSteps = labs(a.motor->currentPosition() - positionStartSteps[i]);
          float movedMm    = stepsToMm(movedSteps, a.pitchMm);
          positionMoveToSwitchMm[i] = movedMm;
          float returnCoord = movedMm - HOMING_RETRACT_MM;
          if (returnCoord < 0.0)    returnCoord = 0.0;
          if (returnCoord > a.maxMm) returnCoord = a.maxMm;
          positionReturnCoordinateMm[i] = returnCoord;
          Serial.print("POSITION: "); Serial.print(a.name);
          Serial.print(" moved "); Serial.print(movedMm, 3); Serial.println(" mm to switch.");
          retractAxisFromLimit(a);
          positionParallelMeasured |= (1 << i);
        } else if (a.homing && a.motor->distanceToGo() == 0) {
          Serial.print("POSITION ERROR: "); Serial.print(a.name);
          Serial.println(" reached commanded travel without hitting switch.");
          positionState = POSITION_ERROR;
        }
      }

      if ((positionParallelMeasured & (1 << i)) && !a.retracting && a.motor->distanceToGo() == 0) {
        positionParallelDone |= (1 << i);
        Serial.print("POSITION: "); Serial.print(a.name); Serial.println(" homed and retracted.");
      }
    }

    if (positionParallelDone == 0x3) {
      printPositionResults();
      clearPositionState();
      return;
    }
  }

  // ── Single-axis phase (Z only) ─────────────────────────────────────────────
  if (positionAxisIdx >= 0 && positionAxisIdx <= 2) {
    Axis &a = axes[positionAxisIdx];

    if (positionState == POSITION_MOVING_TO_SWITCH) {
      if (digitalRead(a.limitPin) == LOW) {
        long  movedSteps = labs(a.motor->currentPosition() - positionStartSteps[positionAxisIdx]);
        float movedMm    = stepsToMm(movedSteps, a.pitchMm);
        positionMoveToSwitchMm[positionAxisIdx] = movedMm;
        float returnCoord = movedMm - HOMING_RETRACT_MM;
        if (returnCoord < 0.0)     returnCoord = 0.0;
        if (returnCoord > a.maxMm) returnCoord = a.maxMm;
        positionReturnCoordinateMm[positionAxisIdx] = returnCoord;
        Serial.print("POSITION: "); Serial.print(a.name);
        Serial.print(" moved "); Serial.print(movedMm, 3); Serial.println(" mm to switch.");
        retractAxisFromLimit(a);
        positionState = POSITION_WAIT_RETRACT;
        return;
      }
      if (a.motor->distanceToGo() == 0) {
        Serial.print("POSITION ERROR: "); Serial.print(a.name);
        Serial.println(" reached commanded travel without hitting switch.");
        positionState = POSITION_ERROR;
      }
    }

    if (positionState == POSITION_WAIT_RETRACT) {
      if (!a.retracting && a.motor->distanceToGo() == 0) {
        Serial.print("POSITION: "); Serial.print(a.name); Serial.println(" homed and retracted.");
        positionOrderIndex++;
        startNextPositionAxis();
        return;
      }
    }
  }

  if (positionState == POSITION_ERROR) {
    for (int i = 0; i < 3; i++) {
      axes[i].motor->stop();
      axes[i].homing     = false;
      axes[i].retracting = false;
    }
    clearPositionState();
    Serial.println("POSITION: stopped after error.");
  }
}
