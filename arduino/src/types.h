#pragma once
#include "stepper_wrapper.h"
#include "config.h"

struct Axis {
  StepperWrapper* motor;
  uint8_t limitPin;
  char    name;
  bool    dirTowardHome;
  int     userDirSign;
  bool    homing;
  bool    retracting;
  float   pitchMm;
  float   maxMm;
  float   maxSpeed;
  float   accel;
};

struct ScrewPosition {
  const char* name;
  const char* screwType;
  bool  screwPlaced;
  float xMm;
  float yMm;
};

struct BitPosition {
  const char* name;
  const char* bitType;
  bool  bitPlaced;
  float xMm;
  float yMm;
  float zMm;
};

struct CameraPosition {
  const char* name;
  float xMm;
  float yMm;
};

// ================== STATE ENUMS ==================
enum AutoUnscrewState {
  AUTO_UNSCREW_IDLE,
  AUTO_UNSCREW_MOVING_TO_SCREW,
  AUTO_UNSCREW_START_UNSCREW,
  AUTO_UNSCREW_RUNNING,
  AUTO_UNSCREW_HOME_Z,
  AUTO_UNSCREW_WAIT_Z_HOME,
  AUTO_UNSCREW_DONE,
  AUTO_UNSCREW_ERROR
};

enum AutoAllState {
  AUTO_ALL_IDLE,
  AUTO_ALL_FIND_NEXT_BIT,
  AUTO_ALL_START_GRAB_BIT,
  AUTO_ALL_WAIT_GRAB_BIT,
  AUTO_ALL_START_SCREWS,
  AUTO_ALL_WAIT_SCREWS,
  AUTO_ALL_START_PLACE_BIT,
  AUTO_ALL_WAIT_PLACE_BIT,
  AUTO_ALL_DONE,
  AUTO_ALL_ERROR
};

enum PositionState {
  POSITION_IDLE,
  POSITION_MOVING_TO_SWITCH,
  POSITION_WAIT_RETRACT,
  POSITION_DONE,
  POSITION_ERROR
};

enum UnscrewState {
  UNSCREW_IDLE,
  UNSCREW_PROBE_DOWN,
  UNSCREW_START_PITCH_ROTATION,
  UNSCREW_MEASURE_PITCH,
  UNSCREW_ACTIVE,
  UNSCREW_RETRACT_Z,
  UNSCREW_DONE,
  UNSCREW_ERROR
};

enum PressState {
  PRESS_IDLE,
  PRESS_DESCEND,
  PRESS_PRESSING,
  PRESS_RETRACT,
  PRESS_DONE
};

enum GrabPlaceState {
  GRAB_PLACE_IDLE,
  GRAB_HOME_Z_WITH_XY,
  GRAB_MOVE_TO_XY,
  GRAB_DESCEND_Z47,
  GRAB_SCREW_2REV,
  GRAB_ASCEND_Z53,
  GRAB_DESCEND_Z52,
  GRAB_MOVE_Y_MINUS30,
  GRAB_FINAL_Z0,
  PLACE_MOVE_TO_OFFSET,
  PLACE_ASCEND_Z52,
  PLACE_MOVE_Y_PLUS30,
  PLACE_FINAL_Z0,
  GRAB_PLACE_DONE,
  GRAB_PLACE_ERROR
};
