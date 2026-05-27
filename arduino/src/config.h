#pragma once

// ================== PIN MAP ==================
// X/Y/Z step pins MUST be Timer1 output-compare pins (OC1A/B/C) for FastAccelStepper.
// Limit switch pins moved to free digital pins to vacate 11/12/13.
//  Pins 8-13 form one physical header block on the Mega — all axis wires here.
//  STEP pins must be OC1A/B/C (11/12/13) for FastAccelStepper hardware timing.
//  DIR pins use the adjacent 8/9/10.
#define X_STEP_PIN   11      // OC1A — FastAccelStepper hardware pin
#define X_DIR_PIN    10
#define X_LIMIT_PIN  30      // moved from 11

#define Y_STEP_PIN   12      // OC1B — FastAccelStepper hardware pin
#define Y_DIR_PIN    9
#define Y_LIMIT_PIN  31      // moved from 12

#define Z_STEP_PIN   13      // OC1C — FastAccelStepper hardware pin
#define Z_DIR_PIN    8
#define Z_LIMIT_PIN  32      // moved from 13

#define SCREW_STEP_PIN 50
#define SCREW_DIR_PIN  51

#define LASER_PIN  A0
#define HX711_DT   39
#define HX711_SCK  40

// ================== MOTION SETTINGS ==================
const long  AXIS_STEPS_PER_REV    = 400;
const float HOMING_RETRACT_MM     = 5.0;
const long  SCREW_STEPS_PER_REV   = 1600;

// FastAccelStepper on Timer1 — hardware-interrupt driven, no software timing risk.
// DM542 handles up to 200 kHz. NEMA 23 reliable torque band: up to ~900 RPM.
// 400 steps/rev × 5mm pitch → 6000 steps/s = 75 mm/s = 900 RPM (safe ceiling).
// KEY: accel must be proportionally high — low accel at high speed = missed steps
// because the motor spends too long in the low-torque high-speed region.
const float X_MAX_SPEED = 6000.0;   // was 2500 — 900 RPM, safe for DM542 + NEMA23
const float X_ACCEL     = 5000.0;   // was 2000 — ramps fast, minimises time at peak speed
const float Y_MAX_SPEED = 6000.0;   // was 2500
const float Y_ACCEL     = 5000.0;   // was 2000
const float Z_MAX_SPEED = 5000.0;   // was 2500 — Z is lighter, 750 RPM fine
const float Z_ACCEL     = 4000.0;   // was 2000
const float HOME_SPEED  = 4000.0;   // was 2500 — homing happens once, still saves ~5 s
const float HOME_ACCEL  = 4000.0;   // was 2000

const float SCREW_MAX_SPEED = 5000.0;   // AccelStepper software limit ~5000 on 16 MHz AVR
const float SCREW_ACCEL     = 5000.0;   // was 5000 — unchanged

const unsigned int STEP_PULSE_US         = 5;
const float        VOLTAGE_DIVIDER_FACTOR = 1.2;

// ================== AXIS TRAVEL LIMITS ==================
const float X_MAX_MM = 540.0;
const float Y_MAX_MM = 371.0;
const float Z_MAX_MM = 90.0;

// ================== LASER CALIBRATION ==================
const float LASER_CAL_A = -5.263157895;
const float LASER_CAL_B = 68.052631579;
const float LASER_SPIKE_FILTER_MM = 2.0;

// ================== UNSCREW SETTINGS ==================
const int   Z_DOWN_SIGN  = 1;
const int   Z_UP_SIGN    = -Z_DOWN_SIGN;
const int   SCREW_MANUAL_DIR   = -1;
const int   SCREW_UNSCREW_DIR  = -1;

const float UNSCREW_CONTACT_DROP_MM        = 2.5;
const int   UNSCREW_CONTACT_CONFIRM_COUNT  = 4;
const float UNSCREW_PITCH_PRE_ROTATIONS    = 3.0;
const float UNSCREW_PITCH_MEASURE_ROTATIONS = 4.0;
const float UNSCREW_PITCH_CORRECTION_FACTOR = 1.05;
const float UNSCREW_EXIT_NEAR_BASELINE_MM  = 2.0;   // laser must return within this of probe baseline to declare exit
const int   UNSCREW_EXIT_CONFIRM_COUNT     = 3;     // consecutive samples required to confirm screw exit
const float UNSCREW_Z_SAFETY_STOP_MM       = 8.0;   // force-declare exit if Z retracts this close to home
const float UNSCREW_BASELINE_TARGET_MM     = 59.0;  // expected laser distance to open surface (mm)
const float UNSCREW_BASELINE_TOLERANCE_MM  = 1.0;   // reading must be within this of target to count
const int   UNSCREW_BASELINE_STABLE_COUNT  = 3;     // consecutive good readings required before starting
const unsigned long UNSCREW_BASELINE_TIMEOUT_MS = 1500; // max wait for stable baseline
const float UNSCREW_MIN_PITCH_TRAVEL_MM    = 0.20;
const float UNSCREW_SOCKET_ENGAGE_MM       = 1.5;
const float UNSCREW_SOCKET_MIN_ROTATIONS   = 0.1;

const float UNSCREW_Z_PROBE_SPEED   = 1500.0;   // keep slow — laser contact detection is sensitive
const float UNSCREW_Z_PROBE_ACCEL   = 3000.0;   // unchanged
const float UNSCREW_Z_FOLLOW_SPEED  = 4000.0;   // was 2500 — Z follows screw rotation upward
const float UNSCREW_Z_FOLLOW_ACCEL  = 6000.0;   // unchanged — already high for responsive follow
const float UNSCREW_Z_RETRACT_SPEED = 5000.0;   // was 2500 — retract is pure time, go fast
const float UNSCREW_Z_RETRACT_ACCEL = 4000.0;   // was 2000
const float UNSCREW_SCREW_SPEED     = 5000.0;   // AccelStepper ceiling — do not raise
const float UNSCREW_SCREW_ACCEL     = 5000.0;   // was 3500 — faster ramp into unscrewing
const float UNSCREW_ANALYSIS_SPEED  = 300.0;
const float UNSCREW_ANALYSIS_ACCEL  = 200.0;

const float KNOWN_PITCHES_MM[] = {
  0.50, 0.70, 0.80, 1.00, 1.25, 1.50, 1.75, 2.00, 2.50, 3.00, 3.50, 4.00
};
const int KNOWN_PITCH_COUNT = sizeof(KNOWN_PITCHES_MM) / sizeof(KNOWN_PITCHES_MM[0]);

// ================== PRESS SETTINGS ==================
const float PRESS_CONTACT_DROP_MM   = 2.5;   // laser drop to confirm surface contact
const int   PRESS_CONTACT_CONFIRM   = 3;     // consecutive samples required to confirm
const float PRESS_MAX_FORCE_N       = 5.0;   // stop when force exceeds this (N)
const float PRESS_MAX_DEPTH_MM      = 15.0;  // stop if Z travels more than this past contact (mm)
const float PRESS_Z_SPEED           = 500.0; // slower = less spring overshoot at force detection
const float PRESS_Z_ACCEL           = 1500.0;
const float PRESS_Z_RETRACT_SPEED   = 3000.0;
const float PRESS_Z_RETRACT_ACCEL   = 1500.0;

// ================== GRAB / PLACE SETTINGS ==================
const float GRAB_Z_ALIGN_MM    = 47.0;
const float GRAB_Z_MAGNETIC_MM = 55.0;
const float GRAB_Z_PULL_MM     = 51.0;
const float GRAB_Y_OFFSET_MM   = 30.0;
const float GRAB_SCREW_REVS    = 2.0;

// ================== SENSOR TIMING ==================
const unsigned long LASER_READ_INTERVAL_MS = 5;
const unsigned long FORCE_READ_INTERVAL_MS = 15;
const unsigned long DATA_PRINT_INTERVAL_MS = 10000;

// ================== DATA PRINT TOGGLES ==================
const bool PRINT_FORCE         = 0;
const bool PRINT_LASER_MM      = 1;
const bool PRINT_LASER_VOLTAGE = 0;
const bool PRINT_X             = 1;
const bool PRINT_Y             = 1;
const bool PRINT_Z             = 1;

// ================== SERIAL INPUT ==================
const int SERIAL_BUFFER_SIZE = 80;

// ================== SAFETY ==================
const bool SAFE_Z_FIRST = true;   // Z must be at home (step 0) before any XY motion is allowed

// ================== POSITION COUNTS ==================
const int SCREW_POSITION_COUNT  = 6;
const int BIT_POSITION_COUNT   = 4;
const int CAMERA_POSITION_COUNT = 6;
const int AUTO_UNSCREW_MAX_SCREWS = 6;
