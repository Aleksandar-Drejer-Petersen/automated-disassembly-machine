#pragma once
#include <AccelStepper.h>
#include <FastAccelStepper.h>
#include "HX711.h"
#include "types.h"

// ================== STEPPERS ==================
// XYZ: FastAccelStepper via wrapper (hardware Timer1 interrupts — no step skipping)
extern FastAccelStepperEngine stepperEngine;
extern StepperWrapper stepperX;
extern StepperWrapper stepperY;
extern StepperWrapper stepperZ;
// Screw: AccelStepper (pin 50 is not a Timer1 compare output)
extern AccelStepper screwStepper;

// ================== SCALE ==================
extern HX711  scale;
extern float  counts_per_N;
extern long   zero_offset;

// ================== AXES / POSITIONS ==================
extern Axis           axes[3];
extern ScrewPosition  screwPositions[SCREW_POSITION_COUNT];
extern BitPosition   bitPositions[BIT_POSITION_COUNT];
extern CameraPosition cameraPositions[CAMERA_POSITION_COUNT];

// ================== AUTO UNSCREW JOB ==================
extern AutoUnscrewState autoUnscrewState;
extern int  autoUnscrewQueue[AUTO_UNSCREW_MAX_SCREWS];
extern int  autoUnscrewCount;
extern int  autoUnscrewIndex;
extern bool autoUnscrewLastJobOk;

// ================== AUTO ALL JOB ==================
extern AutoAllState autoAllState;
extern bool autoAllScrewDone[SCREW_POSITION_COUNT];
extern int  autoAllCurrentBitIdx;
extern int  autoAllCurrentScrewQueue[AUTO_UNSCREW_MAX_SCREWS];
extern int  autoAllCurrentScrewCount;

// ================== TIMING ==================
extern unsigned long lastUpdateMs;
extern unsigned long lastForceReadMs;
extern unsigned long lastLaserReadMs;

// ================== SENSOR ACCUMULATORS ==================
extern double laserSum;
extern long   laserCount;
extern double forceSum;
extern long   forceCount;

// ================== LASER STATE ==================
extern float latestLaserDistanceMm;
extern float latestLaserVoltage;
extern bool  laserReady;
extern bool  laserStreaming;

// ================== FORCE STATE ==================
extern float latestForceN;
extern bool  forceStreaming;

// ================== PRESS STATE ==================
extern PressState pressState;
extern float     pressProbeStartDistance;
extern float     pressContactDistance;
extern float     pressZAtContact;
extern float     pressFinalDepthMm;
extern bool      pressSucceeded;
extern int       pressContactConfirmCount;
extern bool      pressAnalysisMode;

// ================== MACHINE STATE ==================
extern bool machineHomed;
extern bool homingAllInProgress;
extern int  pendingHomingMask;   // bits 0/1 = X/Y axes to home once Z finishes

// ================== POSITION ROUTINE STATE ==================
extern PositionState positionState;
extern int   positionAxisOrder[3];
extern int   positionOrderIndex;
extern int   positionAxisIdx;
extern long  positionStartSteps[3];
extern float positionMoveToSwitchMm[3];
extern float positionReturnCoordinateMm[3];
extern int   positionParallelDone;     // bitmask: bit0=X done, bit1=Y done
extern int   positionParallelMeasured; // bitmask: bit0=X measured, bit1=Y measured

// ================== UNSCREW STATE ==================
extern UnscrewState unscrewState;
extern float unscrewProbeStartDistance;
extern float unscrewContactDistance;
extern float unscrewPitchStartDistance;
extern float unscrewActiveReferenceDistance;
extern float unscrewActiveMinDistance;
extern long  unscrewPitchStartScrewSteps;
extern long  unscrewActiveStartScrewSteps;
extern float unscrewMeasuredPitch;
extern float unscrewChosenPitch;
extern float unscrewActiveStartZ;
extern bool  unscrewPitchMeasurementStarted;
extern bool  unscrewAnalysisMode;
extern int   unscrewPitchAttempt;
extern float unscrewPitchMinDistance;
extern int   unscrewContactConfirmCount;
extern int   unscrewExitConfirmCount;

// ================== GRAB / PLACE STATE ==================
extern GrabPlaceState grabPlaceState;
extern int  grabPlaceBitIdx;
extern bool grabPlaceIsGrab;

// ================== SERIAL BUFFER ==================
extern char serialBuffer[SERIAL_BUFFER_SIZE];
extern int  serialIndex;

// ================== PENDING MOVE COMPLETION MESSAGE ==================
// Set by serial_cmd when a Pi-commanded move starts; cleared by main loop on completion.
#define MOVE_MSG_LEN 32
extern char pendingMoveMsg[MOVE_MSG_LEN];

// ================== STATE CHECKS (defined in globals.cpp) ==================
bool positionActive();
bool autoUnscrewActive();
bool grabPlaceActive();
bool autoAllActive();
bool pressActive();

// ================== STATE RESETS (defined in globals.cpp) ==================
void clearUnscrewState();
void clearPositionState();
void clearAutoUnscrewState();
void clearAutoAllState();
void clearGrabPlaceState();
void clearPressState();
