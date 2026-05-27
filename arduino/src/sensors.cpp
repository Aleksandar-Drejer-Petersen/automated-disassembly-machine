#include "sensors.h"
#include "motion.h"

// Direct HX711 read — no internal blocking wait loop.
// Must only be called immediately after scale.is_ready() returns true.
// Clocks out 24 data bits + 1 gain pulse (channel A, gain 128).
// Disables interrupts for ~50 µs to protect bit-bang timing from Timer1 ISR.
static long hx711_read_nonblocking() {
  long value = 0;
  noInterrupts();
  for (int i = 23; i >= 0; i--) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    if (digitalRead(HX711_DT)) value |= (1L << i);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);
  }
  // One extra pulse sets gain 128, channel A for the next conversion
  digitalWrite(HX711_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX711_SCK, LOW);
  delayMicroseconds(1);
  interrupts();
  // Sign-extend 24-bit two's complement to 32 bits
  if (value & 0x800000L) value |= 0xFF000000L;
  return value;
}

void readSensors() {
  unsigned long now = millis();

  if (now - lastLaserReadMs >= LASER_READ_INTERVAL_MS) {
    lastLaserReadMs = now;

    int laserRaw = analogRead(LASER_PIN);
    runMotorsNow();

    float arduinoV        = laserRaw * (5.0 / 1023.0);
    latestLaserVoltage    = arduinoV * VOLTAGE_DIVIDER_FACTOR;
    float newDistanceMm   = LASER_CAL_A * latestLaserVoltage + LASER_CAL_B;
    if (!laserReady || abs(newDistanceMm - latestLaserDistanceMm) <= LASER_SPIKE_FILTER_MM)
      latestLaserDistanceMm = newDistanceMm;
    laserReady            = true;

    laserSum += laserRaw;
    laserCount++;

    if (laserStreaming) {
      static unsigned long lastStreamMs = 0;
      if (now - lastStreamMs >= 200) {
        lastStreamMs = now;
        Serial.print("L:");
        Serial.print(now);
        Serial.print(":");
        Serial.println(latestLaserDistanceMm, 3);
      }
    }
  }

  if (now - lastForceReadMs >= FORCE_READ_INTERVAL_MS) {
    lastForceReadMs = now;
    // Skip force read while unscrew is active — laser timing is critical there
    // and any HX711 delay would starve laser reads and the screw AccelStepper.
    if (unscrewState == UNSCREW_IDLE && scale.is_ready()) {
      latestForceN  = (hx711_read_nonblocking() - zero_offset) / counts_per_N;
      forceSum     += latestForceN;
      runMotorsNow();
      forceCount++;

      if (forceStreaming) {
        float zNow = getAxisPositionMm(axes[2]);
        Serial.print("F:");
        Serial.print(now);
        Serial.print(":");
        Serial.print(latestForceN, 3);
        Serial.print(":");
        Serial.println(zNow, 3);
      }
    }
  }

  if (now - lastUpdateMs >= DATA_PRINT_INTERVAL_MS) {
    lastUpdateMs = now;

    float avgLaserRaw        = laserCount > 0 ? (float)laserSum  / laserCount : 0;
    float avgForce           = forceCount > 0 ? (float)forceSum  / forceCount : 0;
    float avgArduinoV        = avgLaserRaw * (5.0 / 1023.0);
    float avgBoxV            = avgArduinoV * VOLTAGE_DIVIDER_FACTOR;
    float avgLaserDistanceMm = LASER_CAL_A * avgBoxV + LASER_CAL_B;

    laserSum = 0; laserCount = 0;
    forceSum = 0; forceCount = 0;

    if (anyMotionActive()) return;

    bool anyField = PRINT_FORCE || PRINT_LASER_MM || PRINT_LASER_VOLTAGE ||
                    PRINT_X || PRINT_Y || PRINT_Z;
    if (!anyField) return;

    Serial.print("DATA");
    if (PRINT_FORCE)         { Serial.print(" | Force: ");         Serial.print(avgForce,           3); Serial.print(" N");  }
    if (PRINT_LASER_MM)      { Serial.print(" | Laser: ");         Serial.print(avgLaserDistanceMm, 3); Serial.print(" mm"); }
    if (PRINT_LASER_VOLTAGE) { Serial.print(" | Laser voltage: "); Serial.print(avgBoxV,            4); Serial.print(" V");  }
    if (PRINT_X) { Serial.print(" | X: "); Serial.print(getAxisPositionMm(axes[0]), 3); Serial.print(" mm"); }
    if (PRINT_Y) { Serial.print(" | Y: "); Serial.print(getAxisPositionMm(axes[1]), 3); Serial.print(" mm"); }
    if (PRINT_Z) { Serial.print(" | Z: "); Serial.print(getAxisPositionMm(axes[2]), 3); Serial.print(" mm"); }
    Serial.println();
  }
}
