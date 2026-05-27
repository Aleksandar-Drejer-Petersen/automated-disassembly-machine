#include "globals.h"

// XYZ steppers: wrappers around FastAccelStepper, initialised in setup()
FastAccelStepperEngine stepperEngine;
StepperWrapper stepperX;
StepperWrapper stepperY;
StepperWrapper stepperZ;
// Screw stepper: stays on AccelStepper (pin 50 is not a Timer1 compare output)
AccelStepper screwStepper(AccelStepper::DRIVER, SCREW_STEP_PIN, SCREW_DIR_PIN);

HX711  scale;
float  counts_per_N = -2738.12;
long   zero_offset  = 0;

Axis axes[3] = {
  {&stepperX, X_LIMIT_PIN, 'X', false,  1, false, false, 5.0, X_MAX_MM, X_MAX_SPEED, X_ACCEL},
  {&stepperY, Y_LIMIT_PIN, 'Y', true,  -1, false, false, 5.0, Y_MAX_MM, Y_MAX_SPEED, Y_ACCEL},
  {&stepperZ, Z_LIMIT_PIN, 'Z', true,  -1, false, false, 4.0, Z_MAX_MM, Z_MAX_SPEED, Z_ACCEL}
};

ScrewPosition screwPositions[SCREW_POSITION_COUNT] = {
  {"SCREW1", "M4U", 1, 83.213,  86.988},
  {"SCREW2", "M4U", 1, 103.213, 86.988},
  {"SCREW3", "M4U", 1, 123.213, 86.988},
  {"SCREW4", "M5U", 1, 83.213,  126.988},
  {"SCREW5", "M5U", 1, 103.213, 126.988},
  {"SCREW6", "M5U", 1, 123.213, 126.988}
};

BitPosition bitPositions[BIT_POSITION_COUNT] = {
  {"BIT1", "M4U", 1, 63.525,  361.462, 20.0},
  {"BIT2", "M5U", 1, 99.525,  361.462, 20.0},
  {"BIT3", "M6U", 0, 135.525, 361.462, 20.0},
  {"BIT4", "M8U", 0, 171.525, 361.462, 20.0}
};

CameraPosition cameraPositions[CAMERA_POSITION_COUNT] = {
  {"CAM1", 95.0,    0.0},
  {"CAM2", 95.0,    235.038},
  {"CAM3", 219.988, 0.0},
  {"CAM4", 50.0,    200.0},
  {"CAM5", 250.0,   200.0},
  {"CAM6", 450.0,   200.0}
};

AutoUnscrewState autoUnscrewState     = AUTO_UNSCREW_IDLE;
int  autoUnscrewQueue[AUTO_UNSCREW_MAX_SCREWS];
int  autoUnscrewCount        = 0;
int  autoUnscrewIndex        = 0;
bool autoUnscrewLastJobOk    = false;

AutoAllState autoAllState              = AUTO_ALL_IDLE;
bool autoAllScrewDone[SCREW_POSITION_COUNT];
int  autoAllCurrentBitIdx             = -1;
int  autoAllCurrentScrewQueue[AUTO_UNSCREW_MAX_SCREWS];
int  autoAllCurrentScrewCount          = 0;

unsigned long lastUpdateMs    = 0;
unsigned long lastForceReadMs = 0;
unsigned long lastLaserReadMs = 0;

double laserSum   = 0;
long   laserCount = 0;
double forceSum   = 0;
long   forceCount = 0;

float latestLaserDistanceMm = 0.0;
float latestLaserVoltage    = 0.0;
bool  laserReady            = false;
bool  laserStreaming         = false;

float latestForceN   = 0.0;
bool  forceStreaming  = false;

PressState pressState              = PRESS_IDLE;
float     pressProbeStartDistance = 0.0;
float     pressContactDistance    = 0.0;
float     pressZAtContact         = 0.0;
float     pressFinalDepthMm       = 0.0;
bool      pressSucceeded          = false;
int       pressContactConfirmCount = 0;
bool      pressAnalysisMode       = false;

bool machineHomed        = false;
bool homingAllInProgress = false;
int  pendingHomingMask   = 0;

PositionState positionState           = POSITION_IDLE;
int   positionAxisOrder[3]            = {2, 1, 0};
int   positionOrderIndex              = 0;
int   positionAxisIdx                 = -1;
long  positionStartSteps[3]           = {0, 0, 0};
float positionMoveToSwitchMm[3]       = {0.0, 0.0, 0.0};
float positionReturnCoordinateMm[3]   = {0.0, 0.0, 0.0};
int   positionParallelDone            = 0;
int   positionParallelMeasured        = 0;

UnscrewState unscrewState                   = UNSCREW_IDLE;
float unscrewProbeStartDistance             = 0.0;
float unscrewContactDistance                = 0.0;
float unscrewPitchStartDistance             = 0.0;
float unscrewActiveReferenceDistance        = 0.0;
float unscrewActiveMinDistance              = 0.0;
long  unscrewPitchStartScrewSteps           = 0;
long  unscrewActiveStartScrewSteps          = 0;
float unscrewMeasuredPitch                  = 0.0;
float unscrewChosenPitch                    = 0.0;
float unscrewActiveStartZ                   = 0.0;
bool  unscrewPitchMeasurementStarted        = false;
bool  unscrewAnalysisMode                   = false;
int   unscrewPitchAttempt                   = 0;
float unscrewPitchMinDistance               = 0.0;
int   unscrewContactConfirmCount            = 0;
int   unscrewExitConfirmCount               = 0;

GrabPlaceState grabPlaceState = GRAB_PLACE_IDLE;
int  grabPlaceBitIdx         = -1;
bool grabPlaceIsGrab          = true;

char serialBuffer[SERIAL_BUFFER_SIZE];
int  serialIndex = 0;

char pendingMoveMsg[MOVE_MSG_LEN] = {0};

// ================== STATE CHECKS ==================
bool positionActive()    { return positionState    != POSITION_IDLE;      }
bool autoUnscrewActive() { return autoUnscrewState != AUTO_UNSCREW_IDLE;  }
bool grabPlaceActive()   { return grabPlaceState   != GRAB_PLACE_IDLE;    }
bool autoAllActive()     { return autoAllState     != AUTO_ALL_IDLE;      }
bool pressActive()        { return pressState        != PRESS_IDLE;          }

// ================== STATE RESETS ==================
void clearUnscrewState() {
  unscrewState                  = UNSCREW_IDLE;
  unscrewMeasuredPitch          = 0.0;
  unscrewChosenPitch            = 0.0;
  unscrewPitchMeasurementStarted = false;
  unscrewAnalysisMode           = false;
  unscrewPitchMinDistance       = 0.0;
  unscrewContactConfirmCount    = 0;
  unscrewExitConfirmCount       = 0;
  unscrewPitchAttempt           = 0;
}

void clearPositionState() {
  positionState            = POSITION_IDLE;
  positionOrderIndex       = 0;
  positionAxisIdx          = -1;
  positionParallelDone     = 0;
  positionParallelMeasured = 0;
}

void clearAutoUnscrewState() {
  autoUnscrewState = AUTO_UNSCREW_IDLE;
  autoUnscrewCount = 0;
  autoUnscrewIndex = 0;
}

void clearAutoAllState() {
  autoAllState             = AUTO_ALL_IDLE;
  autoAllCurrentBitIdx    = -1;
  autoAllCurrentScrewCount = 0;
}

void clearGrabPlaceState() {
  grabPlaceState   = GRAB_PLACE_IDLE;
  grabPlaceBitIdx = -1;
}

void clearPressState() {
  pressState               = PRESS_IDLE;
  pressContactConfirmCount = 0;
  pressAnalysisMode        = false;
  pressFinalDepthMm        = 0.0;
  pressSucceeded           = false;
  forceStreaming           = false;
  laserStreaming           = false;
}
