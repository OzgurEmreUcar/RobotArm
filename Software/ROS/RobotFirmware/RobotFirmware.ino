#include <Arduino.h>
#include "Config.h"
#include "MotionCommand.h"
#include "CommandBuffer.h"
#include "StepperEngine.h"
#include "SerialProtocol.h"

// ============================================================
// 6-axis motion controller firmware
//   Axes 0-2: RAMPS 1.4 stepper drivers (X, Y, Z)
//   Axes 3-5: ULN2003 + 28BYJ-48 steppers (U, V, W)
//
// See SerialProtocol.h for the host command format and
// StepperEngine.h for how motion segments are executed.
// ============================================================

void setup() {
  Serial.begin(Config::SERIAL_BAUD_RATE);

  StepperEngine::initPorts();
  StepperEngine::initTimer();

  Serial.println("READY");
}

void loop() {
  SerialProtocol::poll();

  if (!StepperEngine::robotPowerEnabled) return;

  if (StepperEngine::consumeSegmentFinishedFlag()) {
    StepperEngine::SegmentDebugInfo dbg = StepperEngine::getLastSegmentDebugInfo();

    // Timing diagnostics: requested vs. measured segment duration,
    // sent right before the "DON" (done) acknowledgement.
    Serial.print("DBG,");
    Serial.print(dbg.expectedDurationMs);       Serial.print(",");
    Serial.print(dbg.actualDurationMs);         Serial.print(",");
    Serial.print(dbg.maxSteps);                 Serial.print(",");
    Serial.print(dbg.naturalStepIntervalUs);    Serial.print(",");
    Serial.println(dbg.wasClamped ? 1 : 0);

    Serial.println("DON");
  }

  if (!StepperEngine::isSegmentActive() && !CommandBuffer::isEmpty()) {
    StepperEngine::loadNextSegment();
  }
}
