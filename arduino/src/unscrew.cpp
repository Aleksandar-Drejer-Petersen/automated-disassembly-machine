#include "unscrew.h"
#include "motion.h"
#include <math.h>

float chooseClosestPitch(float measuredPitch) {
  float bestPitch = KNOWN_PITCHES_MM[0];
  float bestError = fabs(measuredPitch - bestPitch);
  for (int i = 1; i < KNOWN_PITCH_COUNT; i++) {
    float error = fabs(measuredPitch - KNOWN_PITCHES_MM[i]);
    if (error < bestError) { bestError = error; bestPitch = KNOWN_PITCHES_MM[i]; }
  }
  return bestPitch;
}

void releaseUnscrewControlAfterError(const char* message) {
  Axis &z = axes[2];
  z.motor->stop();
  screwStepper.stop();
  z.motor->setCurrentPosition(z.motor->currentPosition());
  screwStepper.setCurrentPosition(screwStepper.currentPosition());
  clearUnscrewState();
  Serial.println(message);
  Serial.println("UNSCREW: control released. You can now move axes manually.");
}

static void beginActiveUnscrew() {
  Axis &z = axes[2];
  unscrewActiveStartScrewSteps   = screwStepper.currentPosition();
  unscrewActiveStartZ            = getAxisPositionMm(z);
  unscrewActiveReferenceDistance = latestLaserDistanceMm;
  unscrewActiveMinDistance       = latestLaserDistanceMm;

  float screwSpd = unscrewAnalysisMode ? UNSCREW_ANALYSIS_SPEED : UNSCREW_SCREW_SPEED;
  float screwAcc = unscrewAnalysisMode ? UNSCREW_ANALYSIS_ACCEL : UNSCREW_SCREW_ACCEL;
  float zSpd     = unscrewAnalysisMode ? UNSCREW_ANALYSIS_SPEED : UNSCREW_Z_FOLLOW_SPEED;
  float zAcc     = unscrewAnalysisMode ? UNSCREW_ANALYSIS_ACCEL : UNSCREW_Z_FOLLOW_ACCEL;

  screwStepper.setMaxSpeed(screwSpd);
  screwStepper.setAcceleration(screwAcc);
  screwStepper.move(SCREW_UNSCREW_DIR * revToSteps(1000.0, SCREW_STEPS_PER_REV));

  z.motor->setMaxSpeed(zSpd);
  z.motor->setAcceleration(zAcc);

  unscrewState = UNSCREW_ACTIVE;
  Serial.print("UNSCREW: active unscrewing started at "); Serial.print(millis()); Serial.println(" ms");
}

void startUnscrewSequence() {
  if (grabPlaceActive())   { Serial.println("Grab/place is active. Send S to cancel."); return; }
  if (positionActive())    { Serial.println("Position routine is active. Send S to cancel."); return; }
  if (unscrewState != UNSCREW_IDLE) { Serial.println("Unscrew sequence is already active."); return; }
  if (anyMotionActive())   { Serial.println("Motion already active. Wait or send S."); return; }
  if (!laserReady)         { Serial.println("Laser not ready yet. Wait for one DATA line first."); return; }

  Axis &z    = axes[2];
  float zCurrent = getAxisPositionMm(z);
  float zTarget  = clampAxisTarget(z, zCurrent + Z_DOWN_SIGN * z.maxMm);
  long  zSteps   = userMmToMotorSteps(z, zTarget - zCurrent);

  if (zSteps == 0) { Serial.println("Cannot start unscrew. Z has no available downward travel."); return; }

  // Require UNSCREW_BASELINE_STABLE_COUNT consecutive readings within
  // UNSCREW_BASELINE_TOLERANCE_MM of UNSCREW_BASELINE_TARGET_MM before starting.
  {
    int   stable   = 0;
    unsigned long deadline = millis() + UNSCREW_BASELINE_TIMEOUT_MS;
    while (millis() < deadline && stable < UNSCREW_BASELINE_STABLE_COUNT) {
      delay(LASER_READ_INTERVAL_MS + 1);
      runMotorsNow();
      if (fabs(latestLaserDistanceMm - UNSCREW_BASELINE_TARGET_MM) <= UNSCREW_BASELINE_TOLERANCE_MM) {
        stable++;
      } else {
        stable = 0;
      }
    }
    if (stable < UNSCREW_BASELINE_STABLE_COUNT) {
      Serial.print("UNSCREW ERROR: laser baseline not near ");
      Serial.print(UNSCREW_BASELINE_TARGET_MM, 0);
      Serial.print(" mm (last=");
      Serial.print(latestLaserDistanceMm, 1);
      Serial.println(" mm). Moving on.");
      return;
    }
  }
  unscrewProbeStartDistance      = latestLaserDistanceMm;
  unscrewMeasuredPitch           = 0.0;
  unscrewChosenPitch             = 0.0;
  unscrewPitchMeasurementStarted = false;
  unscrewPitchAttempt            = 0;

  float probeSpd = unscrewAnalysisMode ? UNSCREW_ANALYSIS_SPEED : UNSCREW_Z_PROBE_SPEED;
  float probeAcc = unscrewAnalysisMode ? UNSCREW_ANALYSIS_ACCEL : UNSCREW_Z_PROBE_ACCEL;
  z.motor->setMaxSpeed(probeSpd);
  z.motor->setAcceleration(probeAcc);
  z.motor->move(zSteps);
  unscrewState = UNSCREW_PROBE_DOWN;
  Serial.print("UNSCREW: started. probe_baseline="); Serial.print(unscrewProbeStartDistance, 3); Serial.println(" mm");
}

void serviceUnscrew() {
  runMotorsNow();
  if (unscrewState == UNSCREW_IDLE) return;

  Axis &z = axes[2];

  if (unscrewState == UNSCREW_PROBE_DOWN) {
    float dropMm = unscrewProbeStartDistance - latestLaserDistanceMm;

    if (dropMm >= UNSCREW_CONTACT_DROP_MM) {
      unscrewContactConfirmCount++;
      if (unscrewContactConfirmCount >= UNSCREW_CONTACT_CONFIRM_COUNT) {
        z.motor->setCurrentPosition(z.motor->currentPosition());
        unscrewContactDistance    = latestLaserDistanceMm;
        unscrewPitchStartDistance = latestLaserDistanceMm;
        Serial.print("UNSCREW: contact detected at "); Serial.print(millis()); Serial.println(" ms");
        unscrewState = UNSCREW_START_PITCH_ROTATION;
      }
    } else {
      unscrewContactConfirmCount = 0;
    }
    if (z.motor->distanceToGo() == 0 && unscrewState == UNSCREW_PROBE_DOWN) {
      releaseUnscrewControlAfterError("UNSCREW ERROR: Z reached travel limit before contact.");
      return;
    }
    return;
  }

  if (unscrewState == UNSCREW_START_PITCH_ROTATION) {
    if (z.motor->distanceToGo() != 0) return;
    unscrewPitchStartScrewSteps    = screwStepper.currentPosition();
    unscrewPitchStartDistance      = latestLaserDistanceMm;
    unscrewPitchMinDistance        = latestLaserDistanceMm;
    unscrewPitchMeasurementStarted = false;
    float pitchSpd = unscrewAnalysisMode ? UNSCREW_ANALYSIS_SPEED : UNSCREW_SCREW_SPEED;
    float pitchAcc = unscrewAnalysisMode ? UNSCREW_ANALYSIS_ACCEL : UNSCREW_SCREW_ACCEL;
    screwStepper.setMaxSpeed(pitchSpd);
    screwStepper.setAcceleration(pitchAcc);
    screwStepper.move(SCREW_UNSCREW_DIR * revToSteps(
      UNSCREW_PITCH_PRE_ROTATIONS + UNSCREW_PITCH_MEASURE_ROTATIONS + 1.0,
      SCREW_STEPS_PER_REV
    ));
    unscrewState = UNSCREW_MEASURE_PITCH;
    return;
  }

  if (unscrewState == UNSCREW_MEASURE_PITCH) {
    long  totalDeltaSteps = screwStepper.currentPosition() - unscrewPitchStartScrewSteps;
    float totalRotations  = fabs((float)totalDeltaSteps) / (float)SCREW_STEPS_PER_REV;

    if (!unscrewPitchMeasurementStarted) {
      if (latestLaserDistanceMm < unscrewPitchMinDistance)
        unscrewPitchMinDistance = latestLaserDistanceMm;

      float socketBounce = latestLaserDistanceMm - unscrewPitchMinDistance;
      bool socketEngaged = totalRotations >= UNSCREW_SOCKET_MIN_ROTATIONS
                           && socketBounce >= UNSCREW_SOCKET_ENGAGE_MM;
      bool preRotsDone   = totalRotations >= UNSCREW_PITCH_PRE_ROTATIONS;

      if (socketEngaged || preRotsDone) {
        unscrewPitchMeasurementStarted = true;
        unscrewPitchStartScrewSteps    = screwStepper.currentPosition();
        unscrewPitchStartDistance      = latestLaserDistanceMm;
        if (socketEngaged) {
          Serial.print("UNSCREW: socket engaged. Starting pitch measurement early at "); Serial.print(millis()); Serial.println(" ms");
        } else {
          Serial.print("UNSCREW: pitch measurement started at "); Serial.print(millis()); Serial.println(" ms");
        }
      }
      return;
    }

    long  measureDeltaSteps = screwStepper.currentPosition() - unscrewPitchStartScrewSteps;
    float measureRotations  = fabs((float)measureDeltaSteps) / (float)SCREW_STEPS_PER_REV;

    if (measureRotations >= UNSCREW_PITCH_MEASURE_ROTATIONS) {
      screwStepper.stop();
      float measuredTravel = unscrewPitchStartDistance - latestLaserDistanceMm;
      if (measuredTravel <= UNSCREW_MIN_PITCH_TRAVEL_MM) {
        if (unscrewPitchAttempt < 1) {
          unscrewPitchAttempt++;
          Serial.print("UNSCREW: pitch measurement failed. Retrying (attempt 2)...");
          unscrewState = UNSCREW_START_PITCH_ROTATION;
        } else {
          releaseUnscrewControlAfterError("UNSCREW ERROR: pitch measurement failed after retry.");
        }
        return;
      }
      unscrewMeasuredPitch = (measuredTravel / measureRotations) * UNSCREW_PITCH_CORRECTION_FACTOR;
      unscrewChosenPitch   = chooseClosestPitch(unscrewMeasuredPitch);
      Serial.print("UNSCREW: measured pitch = "); Serial.print(unscrewMeasuredPitch, 4); Serial.print(" mm per rotation at "); Serial.print(millis()); Serial.println(" ms");
      Serial.print("UNSCREW: selected pitch = "); Serial.print(unscrewChosenPitch,  3);  Serial.print(" mm per rotation at "); Serial.print(millis()); Serial.println(" ms");
      beginActiveUnscrew();
      return;
    }
  }

  if (unscrewState == UNSCREW_ACTIVE) {
    long  screwDeltaSteps = screwStepper.currentPosition() - unscrewActiveStartScrewSteps;
    float rotations       = fabs((float)screwDeltaSteps) / (float)SCREW_STEPS_PER_REV;  // used for Z follow below

    // Force-declare exit if Z is nearly home — prevents driving into the limit switch
    float zNow = getAxisPositionMm(z);
    if (zNow <= UNSCREW_Z_SAFETY_STOP_MM) {
      screwStepper.stop();
      z.motor->setMaxSpeed(UNSCREW_Z_RETRACT_SPEED);
      z.motor->setAcceleration(UNSCREW_Z_RETRACT_ACCEL);
      z.motor->move(z.dirTowardHome ? 1000000L : -1000000L);
      unscrewState = UNSCREW_RETRACT_Z;
      Serial.print("UNSCREW: safety stop — Z near home ("); Serial.print(zNow, 1); Serial.println(" mm). Retracting.");
      return;
    }

    if (latestLaserDistanceMm >= unscrewProbeStartDistance - UNSCREW_EXIT_NEAR_BASELINE_MM) {
      unscrewExitConfirmCount++;
      if (unscrewExitConfirmCount >= UNSCREW_EXIT_CONFIRM_COUNT) {
        screwStepper.stop();
        z.motor->setMaxSpeed(UNSCREW_Z_RETRACT_SPEED);
        z.motor->setAcceleration(UNSCREW_Z_RETRACT_ACCEL);
        z.motor->move(z.dirTowardHome ? 1000000L : -1000000L);
        unscrewState = UNSCREW_RETRACT_Z;
        Serial.print("UNSCREW: screw exit detected. Retracting Z at "); Serial.print(millis()); Serial.println(" ms");
        return;
      }
    } else {
      unscrewExitConfirmCount = 0;
    }

    float zTarget = clampAxisTarget(z, unscrewActiveStartZ + Z_UP_SIGN * rotations * unscrewChosenPitch);
    z.motor->moveTo(userPositionMmToMotorSteps(z, zTarget));

    if (zTarget <= 0.0 || zTarget >= z.maxMm) {
      releaseUnscrewControlAfterError("UNSCREW ERROR: Z reached travel limit during active unscrewing.");
      return;
    }
  }

  if (unscrewState == UNSCREW_RETRACT_Z) {
    if (screwStepper.distanceToGo() != 0) screwStepper.stop();
    if (!z.retracting && z.motor->distanceToGo() == 0) {
      unscrewState = UNSCREW_DONE;
      Serial.println("UNSCREW: Z fully retracted.");
      return;
    }
  }

  if (unscrewState == UNSCREW_DONE) {
    if (z.motor->distanceToGo() == 0 && screwStepper.distanceToGo() == 0) {
      clearUnscrewState();
      Serial.print("UNSCREW: done at "); Serial.print(millis()); Serial.println(" ms");
      Serial.println("UNSCREW: complete");
    }
  }
}
