#include "serial_cmd.h"
#include "motion.h"
#include "homing.h"
#include "unscrew.h"
#include "press.h"
#include "grab_place.h"
#include "auto_jobs.h"
#include <string.h>
#include <stdlib.h>

static String getToken(String cmd, int tokenIndex) {
  int start = 0;
  for (int i = 0; i < tokenIndex; i++) {
    start = cmd.indexOf(' ', start);
    if (start == -1) return "";
    start++;
  }
  int end = cmd.indexOf(' ', start);
  if (end == -1) end = cmd.length();
  return cmd.substring(start, end);
}

static int findScrewPosition(String screwName) {
  screwName.toUpperCase();
  for (int i = 0; i < SCREW_POSITION_COUNT; i++) {
    if (screwName == screwPositions[i].name) return i;
  }
  return -1;
}

static void moveToScrew(String screwName) {
  int idx = findScrewPosition(screwName);
  if (idx < 0) { Serial.print("Unknown screw position: "); Serial.println(screwName); return; }
  Serial.print("Moving to "); Serial.print(screwPositions[idx].name);
  Serial.print(" type "); Serial.print(screwPositions[idx].screwType);
  Serial.print(" placed "); Serial.print(screwPositions[idx].screwPlaced ? 1 : 0);
  Serial.print(" at X "); Serial.print(screwPositions[idx].xMm, 3);
  Serial.print(" mm, Y "); Serial.print(screwPositions[idx].yMm, 3); Serial.println(" mm.");
  startAbsoluteMoveAll(screwPositions[idx].xMm, screwPositions[idx].yMm, 0.0);
  String msg = "SYSTEM IS AT " + String(screwPositions[idx].name);
  strncpy(pendingMoveMsg, msg.c_str(), MOVE_MSG_LEN - 1);
  pendingMoveMsg[MOVE_MSG_LEN - 1] = '\0';
}

static int findBitPosition(String bitName) {
  bitName.toUpperCase();
  for (int i = 0; i < BIT_POSITION_COUNT; i++) {
    if (bitName == bitPositions[i].name) return i;
  }
  return -1;
}

static void moveToBit(String bitName) {
  int idx = findBitPosition(bitName);
  if (idx < 0) { Serial.print("Unknown bit position: "); Serial.println(bitName); return; }
  Serial.print("Moving to "); Serial.print(bitPositions[idx].name);
  Serial.print(" type "); Serial.print(bitPositions[idx].bitType);
  Serial.print(" placed "); Serial.print(bitPositions[idx].bitPlaced ? 1 : 0);
  Serial.print(" at X "); Serial.print(bitPositions[idx].xMm, 3);
  Serial.print(" mm, Y "); Serial.print(bitPositions[idx].yMm, 3);
  Serial.print(" mm, Z "); Serial.print(bitPositions[idx].zMm, 3); Serial.println(" mm.");
  startAbsoluteMoveAll(bitPositions[idx].xMm, bitPositions[idx].yMm, bitPositions[idx].zMm);
}

static int findCameraPosition(String camName) {
  camName.toUpperCase();
  for (int i = 0; i < CAMERA_POSITION_COUNT; i++) {
    if (camName == cameraPositions[i].name) return i;
  }
  return -1;
}

static void moveToCameraPosition(String camName) {
  int idx = findCameraPosition(camName);
  if (idx < 0) { Serial.print("Unknown camera position: "); Serial.println(camName); return; }
  Serial.print("Moving to "); Serial.print(cameraPositions[idx].name);
  Serial.print(" at X "); Serial.print(cameraPositions[idx].xMm, 3);
  Serial.print(" mm, Y "); Serial.print(cameraPositions[idx].yMm, 3); Serial.println(" mm.");
  startAbsoluteMoveAll(cameraPositions[idx].xMm, cameraPositions[idx].yMm, 0.0);
}

void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String originalCmd = cmd;
  cmd.toUpperCase();
  while (cmd.indexOf("  ") >= 0) cmd.replace("  ", " ");


  String token0 = getToken(cmd, 0);
  String token1 = getToken(cmd, 1);
  String token2 = getToken(cmd, 2);
  String token3 = getToken(cmd, 3);

  if (cmd == "STATUS") {
    if (anyMotionActive() || positionActive() || autoUnscrewActive() || autoAllActive() ||
        grabPlaceActive() || unscrewState != UNSCREW_IDLE || pressActive()) {
      Serial.println("STATUS BUSY");
    } else {
      Serial.println("STATUS IDLE");
    }
    Serial.print("HOMED: "); Serial.println(machineHomed ? 1 : 0);
    return;
  }

  if (cmd == "S")  { stopAll();     return; }
  if (cmd == "SS") { hardStopAll(); return; }

  if (cmd == "LASER STREAM") { laserStreaming = true;  Serial.println("LASER STREAM ON");  return; }
  if (cmd == "LASER STOP")   { laserStreaming = false; Serial.println("LASER STREAM OFF"); return; }
  if (cmd == "FORCE STREAM") { forceStreaming = true;  Serial.println("FORCE STREAM ON");  return; }
  if (cmd == "FORCE STOP")   { forceStreaming = false; Serial.println("FORCE STREAM OFF"); return; }

  if (autoAllActive()) { Serial.println("AUTO ALL is active. Send S to cancel."); return; }

  if (cmd == "T") {
    if (scale.is_ready()) { zero_offset = scale.read_average(20); Serial.println("Tared."); }
    return;
  }

  if (cmd == "POSITION") { startPositionRoutine(); return; }
  if (cmd == "HOME" || cmd == "H")           { initHoming(-1); return; }
  if (cmd == "HOME X" || cmd == "H X")      { initHoming(0);  return; }
  if (cmd == "HOME Y" || cmd == "H Y")      { initHoming(1);  return; }
  if (cmd == "HOME Z" || cmd == "H Z")      { initHoming(2);  return; }

  if (cmd == "UNSCREW ALL") { startAutoAllJob(); return; }

  // ================== SET / CLEAR SCREW (from Raspberry Pi) ==================
  if (cmd == "CLEAR SCREWS") {
    for (int i = 0; i < SCREW_POSITION_COUNT; i++) screwPositions[i].screwPlaced = false;
    Serial.println("SCREWS CLEARED");
    return;
  }

  // SET SCREW <index 1-6> <type> <x> <y>
  if (token0 == "SET" && token1 == "SCREW") {
    int idx = token2.toInt() - 1;
    if (idx < 0 || idx >= SCREW_POSITION_COUNT) {
      Serial.print("SET SCREW: invalid index "); Serial.println(token2);
      return;
    }
    String screwType = getToken(cmd, 3);
    float  xMm       = getToken(cmd, 4).toFloat();
    float  yMm       = getToken(cmd, 5).toFloat();

    static char typeBuffer[SCREW_POSITION_COUNT][8];
    strncpy(typeBuffer[idx], screwType.c_str(), 7);
    typeBuffer[idx][7]            = '\0';
    screwPositions[idx].screwType = typeBuffer[idx];
    screwPositions[idx].xMm       = xMm;
    screwPositions[idx].yMm       = yMm;
    screwPositions[idx].screwPlaced = true;

    Serial.print("SCREW SET: "); Serial.print(idx + 1);
    Serial.print(" type="); Serial.print(screwType);
    Serial.print(" x="); Serial.print(xMm, 3);
    Serial.print(" y="); Serial.println(yMm, 3);
    return;
  }

  if (token0 == "GRAB" && token1.startsWith("BIT")) {
    int idx = findBitPosition(token1);
    if (idx < 0) { Serial.print("Unknown bit: "); Serial.println(token1); return; }
    startGrabSequence(idx, false);
    return;
  }

  if (token0 == "PLACE" && token1.startsWith("BIT")) {
    int idx = findBitPosition(token1);
    if (idx < 0) { Serial.print("Unknown bit: "); Serial.println(token1); return; }
    startPlaceSequence(idx, false);
    return;
  }

  if (token0 == "UNSCREW") {
    if (token1.length() == 0) { startUnscrewSequence(); return; }

    if (token1 == "ANALYSE") {
      unscrewAnalysisMode = true;
      startUnscrewSequence();
      return;
    }

    if (token1 == "SCREWS") {
      int indices[AUTO_UNSCREW_MAX_SCREWS];
      int count = 0;
      for (int t = 2; t < 2 + SCREW_POSITION_COUNT; t++) {
        String tok = getToken(cmd, t);
        if (tok.length() == 0) break;
        String screwName = "SCREW" + tok;
        int idx = findScrewPosition(screwName);
        if (idx < 0) { Serial.print("Unknown screw number: "); Serial.println(tok); return; }
        if (count >= AUTO_UNSCREW_MAX_SCREWS) { Serial.println("Too many screws in one job."); return; }
        indices[count++] = idx;
      }
      if (count == 0) { Serial.println("Format: unscrew screws 1 2 3"); return; }
      startAutoUnscrewJobMulti(indices, count, false);
      return;
    }

    Serial.println("Format: unscrew  OR  unscrew screws 1 2 3  OR  unscrew all");
    return;
  }

  if (token0 == "PRESS") {
    if (token1 == "ANALYSE") {
      pressAnalysisMode = true;
      laserStreaming   = true;
      forceStreaming   = true;
    }
    startPressSequence();
    return;
  }

  if (token0 == "X") {
    if (token1.length() == 0) { Serial.println("Format: x 10"); return; }
    if (startRelativeMove(0, token1.toFloat())) {
      strncpy(pendingMoveMsg, "SYSTEM IS AT X", MOVE_MSG_LEN - 1);
      pendingMoveMsg[MOVE_MSG_LEN - 1] = '\0';
    } else {
      Serial.println("SYSTEM IS AT X");
    }
    return;
  }

  if (token0 == "Y") {
    if (token1.length() == 0) { Serial.println("Format: y 10"); return; }
    if (startRelativeMove(1, token1.toFloat())) {
      strncpy(pendingMoveMsg, "SYSTEM IS AT Y", MOVE_MSG_LEN - 1);
      pendingMoveMsg[MOVE_MSG_LEN - 1] = '\0';
    } else {
      Serial.println("SYSTEM IS AT Y");
    }
    return;
  }

  if (token0 == "Z") {
    if (token1.length() == 0) { Serial.println("Format: z 10"); return; }
    startRelativeMove(2, token1.toFloat());
    return;
  }

  if (token0 == "SCREW" && token1.length() > 0) { startScrewMove(token1.toFloat()); return; }

  if (token0 == "MOVE" && token1 == "TO") {
    if (token2.length() == 0) { Serial.println("Format: move to screw1 OR move to bit1 OR move to cam1"); return; }
    if (token2.startsWith("SCREW")) { moveToScrew(token2); return; }
    if (token2.startsWith("BIT"))  { moveToBit(token2);  return; }
    if (token2.startsWith("CAM"))   { moveToCameraPosition(token2); return; }
    Serial.println("Unknown position type. Use screw1, bit1, or cam1.");
    return;
  }

  if (token0.startsWith("SCREW") && token1.length() == 0) { moveToScrew(token0); return; }
  if (token0.startsWith("BIT")  && token1.length() == 0) { moveToBit(token0);  return; }
  if (token0.startsWith("CAM")   && token1.length() == 0) {
    int camIdx = findCameraPosition(token0);
    if (camIdx < 0) { Serial.print("Unknown camera position: "); Serial.println(token0); return; }
    String completionMsg = "SYSTEM IS AT " + token0;
    if (startAbsoluteMoveAll(cameraPositions[camIdx].xMm, cameraPositions[camIdx].yMm, 0.0)) {
      strncpy(pendingMoveMsg, completionMsg.c_str(), MOVE_MSG_LEN - 1);
      pendingMoveMsg[MOVE_MSG_LEN - 1] = '\0';
    } else {
      Serial.println(completionMsg);
    }
    return;
  }

  if (token0 == "TO") {
    if (token1.length() == 0 || token2.length() == 0 || token3.length() == 0) {
      Serial.println("Format: to 12 30 40");
      return;
    }
    if (startAbsoluteMoveAll(token1.toFloat(), token2.toFloat(), token3.toFloat())) {
      strncpy(pendingMoveMsg, "SYSTEM IS AT POSITION", MOVE_MSG_LEN - 1);
      pendingMoveMsg[MOVE_MSG_LEN - 1] = '\0';
    } else {
      Serial.println("SYSTEM IS AT POSITION");
    }
    return;
  }

  Serial.print("Unknown command received: "); Serial.println(originalCmd);
}

void handleSerial() {
  while (true) {
    if (Serial.available() == 0) {
      // Mid-command gap: wait up to 2 ms for the rest of the bytes before giving up.
      if (serialIndex == 0) break;
      unsigned long t = millis();
      while (Serial.available() == 0 && (millis() - t) < 2) {}
      if (Serial.available() == 0) break;
    }
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        parseCommand(String(serialBuffer));
        serialIndex     = 0;
        serialBuffer[0] = '\0';
      }
    } else {
      if (serialIndex < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[serialIndex] = c;
        serialIndex++;
        serialBuffer[serialIndex] = '\0';
      } else {
        serialIndex     = 0;
        serialBuffer[0] = '\0';
        Serial.println("Serial input too long. Buffer cleared.");
      }
    }
  }
}
