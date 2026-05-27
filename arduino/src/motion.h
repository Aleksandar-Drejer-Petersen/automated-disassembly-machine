#pragma once
#include "globals.h"

// ================== STEP CONVERSION ==================
long  mmToSteps(float mm, float pitchMm);
float stepsToMm(long steps, float pitchMm);
long  revToSteps(float rev, long stepsPerRev);
float getAxisPositionMm(Axis &a);
long  userMmToMotorSteps(Axis &a, float mm);
long  userPositionMmToMotorSteps(Axis &a, float positionMm);

// ================== MOTOR KEEPALIVE ==================
void runMotorsNow();

// ================== HELPERS ==================
bool  axisIsMoving();
bool  anyMotionActive();
void  setupMotor(AccelStepper &motor, float maxSpeed, float accel);
float clampAxisTarget(Axis &a, float targetMm);
bool  stepsMoveTowardHome(Axis &a, long steps);

// ================== INTERNAL MOVERS (used by grab/place, unscrew) ==================
void moveXYZAbsInternal(float xMm, float yMm, float zMm);
void moveXYAbsInternal(float xMm, float yMm);
void moveZAbsInternal(float zMm);
void moveYRelInternal(float mm);
void screwRevInternal(float revs);

// ================== USER COMMANDS ==================
bool startRelativeMove(int axisIdx, float moveMm);
bool startAbsoluteMoveAll(float xTargetMm, float yTargetMm, float zTargetMm);
void startScrewMove(float revolutions);

// ================== STOP ==================
void stopAll();
void hardStopAll();
