#include "press.h"
#include "motion.h"

static void releasePressAfterError(const char* message) {
  Axis &z = axes[2];
  z.motor->stop();
  z.motor->setCurrentPosition(z.motor->currentPosition());
  Serial.println(message);

  z.motor->setMaxSpeed(PRESS_Z_RETRACT_SPEED);
  z.motor->setAcceleration(PRESS_Z_RETRACT_ACCEL);
  z.motor->move(z.dirTowardHome ? 1000000L : -1000000L);
  pressState = PRESS_RETRACT;
}

void startPressSequence() {
  if (grabPlaceActive())       { Serial.println("Grab/place is active. Send S to cancel."); return; }
  if (positionActive())        { Serial.println("Position routine is active. Send S to cancel."); return; }
  if (pressState != PRESS_IDLE)  { Serial.println("Press sequence already active."); return; }
  if (unscrewState != UNSCREW_IDLE) { Serial.println("Unscrew sequence is active. Send S to cancel."); return; }
  if (anyMotionActive())       { Serial.println("Motion already active. Wait or send S."); return; }
  if (!laserReady)             { Serial.println("Laser not ready yet. Wait for one DATA line first."); return; }

  Axis &z     = axes[2];
  float zCurr = getAxisPositionMm(z);
  float zTgt  = clampAxisTarget(z, zCurr + Z_DOWN_SIGN * z.maxMm);
  long  zSteps = userMmToMotorSteps(z, zTgt - zCurr);

  if (zSteps == 0) { Serial.println("Cannot start press. Z has no available downward travel."); return; }

  // Same laser baseline check as unscrew — rejects bad readings before descending.
  {
    int   stable   = 0;
    unsigned long deadline = millis() + UNSCREW_BASELINE_TIMEOUT_MS;
    while (millis() < deadline && stable < UNSCREW_BASELINE_STABLE_COUNT) {
      delay(LASER_READ_INTERVAL_MS + 1);
      if (fabs(latestLaserDistanceMm - UNSCREW_BASELINE_TARGET_MM) <= UNSCREW_BASELINE_TOLERANCE_MM) {
        stable++;
      } else {
        stable = 0;
      }
    }
    if (stable < UNSCREW_BASELINE_STABLE_COUNT) {
      Serial.print("PRESS ERROR: laser baseline not near ");
      Serial.print(UNSCREW_BASELINE_TARGET_MM, 0);
      Serial.print(" mm (last=");
      Serial.print(latestLaserDistanceMm, 1);
      Serial.println(" mm). Moving on.");
      forceStreaming = false;
      laserStreaming = false;
      return;
    }
  }

  pressProbeStartDistance  = latestLaserDistanceMm;
  pressContactConfirmCount = 0;

  z.motor->setMaxSpeed(PRESS_Z_SPEED);
  z.motor->setAcceleration(PRESS_Z_ACCEL);
  z.motor->move(zSteps);
  pressState = PRESS_DESCEND;

  Serial.print("PRESS: started. probe_baseline=");
  Serial.print(pressProbeStartDistance, 3);
  Serial.println(" mm");
}

void servicePress() {
  runMotorsNow();
  if (pressState == PRESS_IDLE) return;

  Axis &z = axes[2];

  // ── DESCEND: wait for laser contact ───────────────────────────────────────
  if (pressState == PRESS_DESCEND) {
    float dropMm = pressProbeStartDistance - latestLaserDistanceMm;

    if (dropMm >= PRESS_CONTACT_DROP_MM) {
      pressContactConfirmCount++;
      if (pressContactConfirmCount >= PRESS_CONTACT_CONFIRM) {
        pressContactDistance = latestLaserDistanceMm;
        pressZAtContact      = getAxisPositionMm(z);
        pressState           = PRESS_PRESSING;

        Serial.print("PRESS: contact detected at ");
        Serial.print(millis());
        Serial.print(" ms. Z=");
        Serial.print(pressZAtContact, 3);
        Serial.println(" mm");
      }
    } else {
      pressContactConfirmCount = 0;
    }

    if (z.motor->distanceToGo() == 0 && pressState == PRESS_DESCEND) {
      releasePressAfterError("PRESS ERROR: Z reached travel limit before contact.");
    }
    return;
  }

  // ── PRESSING: monitor force and depth ─────────────────────────────────────
  if (pressState == PRESS_PRESSING) {
    float zNow      = getAxisPositionMm(z);
    float zDepth    = (zNow - pressZAtContact) * Z_DOWN_SIGN;   // mm below contact point

    bool forceLimit = (latestForceN > PRESS_MAX_FORCE_N);
    bool depthLimit = (zDepth       > PRESS_MAX_DEPTH_MM);
    bool zHitLimit  = (z.motor->distanceToGo() == 0);

    if (forceLimit || depthLimit || zHitLimit) {
      z.motor->stop();
      z.motor->setCurrentPosition(z.motor->currentPosition());

      float actualDrop  = pressContactDistance - latestLaserDistanceMm;
      float surfaceMove = actualDrop - zDepth;   // positive = surface moved away (button pressed)

      if (forceLimit) {
        Serial.print("PRESS: force limit reached at ");
        Serial.print(millis());
        Serial.print(" ms. Force="); Serial.print(latestForceN, 3);
        Serial.print(" N. Z_depth="); Serial.print(zDepth, 3);
        Serial.print(" mm. Surface_moved="); Serial.print(surfaceMove, 3);
        Serial.println(" mm");
      } else if (depthLimit) {
        Serial.print("PRESS: depth limit reached at ");
        Serial.print(millis());
        Serial.print(" ms. Z_depth="); Serial.print(zDepth, 3);
        Serial.print(" mm. Surface_moved="); Serial.print(surfaceMove, 3);
        Serial.println(" mm");
      } else {
        Serial.println("PRESS: Z travel limit hit during press.");
      }

      pressFinalDepthMm = zDepth;
      pressSucceeded    = (forceLimit || depthLimit);

      z.motor->setMaxSpeed(PRESS_Z_RETRACT_SPEED);
      z.motor->setAcceleration(PRESS_Z_RETRACT_ACCEL);
      z.motor->move(z.dirTowardHome ? 1000000L : -1000000L);
      pressState = PRESS_RETRACT;
    }
    return;
  }

  // ── RETRACT: wait for Z to reach home ────────────────────────────────────
  if (pressState == PRESS_RETRACT) {
    if (!z.retracting && z.motor->distanceToGo() == 0) {
      pressState = PRESS_DONE;
      Serial.print("PRESS: Z retracted at ");
      Serial.print(millis());
      Serial.println(" ms");
    }
    return;
  }

  // ── DONE ─────────────────────────────────────────────────────────────────
  if (pressState == PRESS_DONE) {
    if (z.motor->distanceToGo() == 0) {
      bool succeeded = pressSucceeded;
      float depth    = pressFinalDepthMm;
      clearPressState();
      Serial.print("PRESS: done at "); Serial.print(millis()); Serial.println(" ms");
      Serial.print("PRESS: depth="); Serial.print(depth, 2); Serial.println(" mm");
      if (succeeded) {
        Serial.println("PRESS: complete");
      } else {
        Serial.println("PRESS: failed");
      }
    }
  }
}
