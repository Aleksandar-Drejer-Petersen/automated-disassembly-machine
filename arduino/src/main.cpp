#include <Arduino.h>
#include "globals.h"
#include "motion.h"
#include "homing.h"
#include "sensors.h"
#include "unscrew.h"
#include "press.h"
#include "grab_place.h"
#include "auto_jobs.h"
#include "serial_cmd.h"

void setup() {
  Serial.begin(115200);

  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);

  // XYZ: FastAccelStepper — hardware Timer1 interrupts, no software run() needed
  stepperEngine.init();

  FastAccelStepper* fasX = stepperEngine.stepperConnectToPin(X_STEP_PIN);
  fasX->setDirectionPin(X_DIR_PIN);
  stepperX.init(fasX);
  stepperX.setCap(X_MAX_SPEED, X_ACCEL);

  FastAccelStepper* fasY = stepperEngine.stepperConnectToPin(Y_STEP_PIN);
  fasY->setDirectionPin(Y_DIR_PIN);
  stepperY.init(fasY);
  stepperY.setCap(Y_MAX_SPEED, Y_ACCEL);

  FastAccelStepper* fasZ = stepperEngine.stepperConnectToPin(Z_STEP_PIN);
  fasZ->setDirectionPin(Z_DIR_PIN);
  stepperZ.init(fasZ);
  stepperZ.setCap(Z_MAX_SPEED, Z_ACCEL);

  // Screw: AccelStepper (software-driven, called in runMotorsNow)
  setupMotor(screwStepper, SCREW_MAX_SPEED, SCREW_ACCEL);

  // ── Force sensor ────────────────────────────────────────────────────────────
  scale.begin(HX711_DT, HX711_SCK);

  Serial.println();
  Serial.println("SYSTEM READY");
  Serial.println("Commands:");
  Serial.println("  x 10");
  Serial.println("  y 250");
  Serial.println("  z 5");
  Serial.println("  screw 10");
  Serial.println("  to 12 30 40");
  Serial.println("  move to screw1");
  Serial.println("  screw1");
  Serial.println("  unscrew");
  Serial.println("  unscrew screws 1 2 3");
  Serial.println("  unscrew all");
  Serial.println("  grab bit1");
  Serial.println("  place bit1");
  Serial.println("  position");
  Serial.println("  home / h");
  Serial.println("  s  = smooth stop");
  Serial.println("  ss = hard stop");
  Serial.println("  t  = tare force sensor");
  Serial.println("  set screw <n> <type> <x> <y>");
  Serial.println("  clear screws");

  Serial.print("  X max = "); Serial.print(X_MAX_MM, 3); Serial.println(" mm");
  Serial.print("  Y max = "); Serial.print(Y_MAX_MM, 3); Serial.println(" mm");
  Serial.print("  Z max = "); Serial.print(Z_MAX_MM, 3); Serial.println(" mm");
  Serial.print("Axis steps per revolution = "); Serial.println(AXIS_STEPS_PER_REV);
  Serial.print("Data print interval = "); Serial.print(DATA_PRINT_INTERVAL_MS); Serial.println(" ms");

  Serial.println("Bits:");
  for (int i = 0; i < BIT_POSITION_COUNT; i++) {
    Serial.print("  "); Serial.print(bitPositions[i].name);
    Serial.print(" = "); Serial.print(bitPositions[i].bitType);
    Serial.print(" placed "); Serial.println(bitPositions[i].bitPlaced ? 1 : 0);
  }

  Serial.println("Screws:");
  for (int i = 0; i < SCREW_POSITION_COUNT; i++) {
    Serial.print("  "); Serial.print(screwPositions[i].name);
    Serial.print(" = "); Serial.print(screwPositions[i].screwType);
    Serial.print(" placed "); Serial.println(screwPositions[i].screwPlaced ? 1 : 0);
  }

  unsigned long startWait = millis();
  while (!scale.is_ready() && millis() - startWait < 5000) delay(10);

  if (scale.is_ready()) {
    zero_offset = scale.read_average(30);
    Serial.println("Sensor tared.");
  } else {
    Serial.println("WARNING: HX711 not responding.");
  }
}

void loop() {
  servicePositionRoutine();
  serviceAxes();

  // Print completion message for Pi-commanded X / Y / CAM moves once axes stop.
  if (pendingMoveMsg[0] != '\0' && !axisIsMoving()) {
    Serial.println(pendingMoveMsg);
    pendingMoveMsg[0] = '\0';
  }

  runMotorsNow();
  readSensors();

  runMotorsNow();
  Serial.flush();   // drain TX to 16U2 before reading RX — prevents USB bridge receive-dead state
  handleSerial();

  runMotorsNow();
  serviceUnscrew();

  runMotorsNow();
  servicePress();

  runMotorsNow();
  serviceAutoUnscrewJob();

  runMotorsNow();
  serviceGrabPlace();

  runMotorsNow();
  serviceAutoAllJob();

  runMotorsNow();
}
