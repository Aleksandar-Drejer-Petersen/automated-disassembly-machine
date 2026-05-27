#include "grab_place.h"
#include "motion.h"

static bool grabPlacePrecheck(bool internalCall) {
  if (grabPlaceActive())   { Serial.println("Grab/place already active. Send S to cancel."); return false; }
  if (!internalCall && autoAllActive()) { Serial.println("AUTO ALL is active. Send S to cancel."); return false; }
  if (autoUnscrewActive()) { Serial.println("AUTO UNSCREW is active. Send S to cancel."); return false; }
  if (positionActive())    { Serial.println("Position routine is active. Send S to cancel."); return false; }
  if (unscrewState != UNSCREW_IDLE) { Serial.println("Unscrew sequence is active. Send S to cancel."); return false; }
  if (anyMotionActive())   { Serial.println("Motion already active. Wait or send S."); return false; }
  return true;
}

void startGrabSequence(int bitIdx, bool internalCall) {
  if (!grabPlacePrecheck(internalCall)) return;

  if (!bitPositions[bitIdx].bitPlaced) {
    Serial.print("GRAB ERROR: "); Serial.print(bitPositions[bitIdx].name);
    Serial.println(" is marked empty.");
    return;
  }

  grabPlaceBitIdx = bitIdx;
  grabPlaceIsGrab  = true;

  BitPosition &t = bitPositions[bitIdx];
  moveXYAbsInternal(t.xMm, t.yMm);
  Axis &z = axes[2];
  z.motor->setMaxSpeed(z.maxSpeed);
  z.motor->setAcceleration(z.accel);
  z.motor->move(z.dirTowardHome ? 1000000L : -1000000L);
  grabPlaceState = GRAB_HOME_Z_WITH_XY;
}

void startPlaceSequence(int bitIdx, bool internalCall) {
  if (!grabPlacePrecheck(internalCall)) return;

  grabPlaceBitIdx = bitIdx;
  grabPlaceIsGrab  = false;

  BitPosition &t  = bitPositions[bitIdx];
  float yOffset    = t.yMm - GRAB_Y_OFFSET_MM;
  moveXYAbsInternal(t.xMm, yOffset);
  Axis &z = axes[2];
  z.motor->setMaxSpeed(z.maxSpeed);
  z.motor->setAcceleration(z.accel);
  z.motor->move(z.dirTowardHome ? 1000000L : -1000000L);
  grabPlaceState = GRAB_HOME_Z_WITH_XY;
}

void serviceGrabPlace() {
  runMotorsNow();
  if (grabPlaceState == GRAB_PLACE_IDLE) return;

  if (grabPlaceState == GRAB_HOME_Z_WITH_XY) {
    Axis &z = axes[2];
    bool xyDone = axes[0].motor->distanceToGo() == 0 && axes[1].motor->distanceToGo() == 0;
    bool zDone  = z.motor->distanceToGo() == 0 && !z.retracting;
    if (xyDone && zDone) {
      if (grabPlaceIsGrab) {
        moveZAbsInternal(GRAB_Z_ALIGN_MM);
        grabPlaceState = GRAB_DESCEND_Z47;
      } else {
        moveZAbsInternal(GRAB_Z_PULL_MM);
        grabPlaceState = PLACE_ASCEND_Z52;
      }
    }
    return;
  }

  if (grabPlaceState == GRAB_MOVE_TO_XY) {
    if (!axisIsMoving()) {
      moveZAbsInternal(GRAB_Z_ALIGN_MM);
      grabPlaceState = GRAB_DESCEND_Z47;
    }
    return;
  }

  if (grabPlaceState == GRAB_DESCEND_Z47) {
    if (!axisIsMoving()) {
      screwRevInternal(GRAB_SCREW_REVS);
      grabPlaceState = GRAB_SCREW_2REV;
    }
    return;
  }

  if (grabPlaceState == GRAB_SCREW_2REV) {
    if (screwStepper.distanceToGo() == 0) {
      moveZAbsInternal(GRAB_Z_MAGNETIC_MM);
      grabPlaceState = GRAB_ASCEND_Z53;
    }
    return;
  }

  if (grabPlaceState == GRAB_ASCEND_Z53) {
    if (!axisIsMoving()) {
      moveZAbsInternal(GRAB_Z_PULL_MM);
      grabPlaceState = GRAB_DESCEND_Z52;
    }
    return;
  }

  if (grabPlaceState == GRAB_DESCEND_Z52) {
    if (!axisIsMoving()) {
      moveYRelInternal(-GRAB_Y_OFFSET_MM);
      grabPlaceState = GRAB_MOVE_Y_MINUS30;
    }
    return;
  }

  if (grabPlaceState == GRAB_MOVE_Y_MINUS30) {
    if (!axisIsMoving()) {
      moveZAbsInternal(0.0);
      grabPlaceState = GRAB_FINAL_Z0;
    }
    return;
  }

  if (grabPlaceState == GRAB_FINAL_Z0) {
    if (!axisIsMoving()) grabPlaceState = GRAB_PLACE_DONE;
    return;
  }

  if (grabPlaceState == PLACE_MOVE_TO_OFFSET) {
    if (!axisIsMoving()) {
      moveZAbsInternal(GRAB_Z_PULL_MM);
      grabPlaceState = PLACE_ASCEND_Z52;
    }
    return;
  }

  if (grabPlaceState == PLACE_ASCEND_Z52) {
    if (!axisIsMoving()) {
      moveYRelInternal(GRAB_Y_OFFSET_MM);
      grabPlaceState = PLACE_MOVE_Y_PLUS30;
    }
    return;
  }

  if (grabPlaceState == PLACE_MOVE_Y_PLUS30) {
    if (!axisIsMoving()) {
      moveZAbsInternal(0.0);
      grabPlaceState = PLACE_FINAL_Z0;
    }
    return;
  }

  if (grabPlaceState == PLACE_FINAL_Z0) {
    if (!axisIsMoving()) grabPlaceState = GRAB_PLACE_DONE;
    return;
  }

  if (grabPlaceState == GRAB_PLACE_DONE) {
    Serial.println(grabPlaceIsGrab ? "GRAB: complete." : "PLACE: complete.");
    clearGrabPlaceState();
    return;
  }
}
