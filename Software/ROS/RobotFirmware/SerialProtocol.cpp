#include "SerialProtocol.h"
#include <Arduino.h>
#include "Config.h"
#include "CommandBuffer.h"
#include "MotionCommand.h"
#include "StepperEngine.h"

namespace SerialProtocol {

static char lineBuffer[Config::SERIAL_LINE_BUFFER_SIZE];
static bool frameReady = false;

// Reads bytes between Config::FRAME_START_MARKER and FRAME_END_MARKER
// into lineBuffer. Sets frameReady once a full frame has arrived.
static void receiveFrame() {
  static bool receiving = false;
  static byte idx = 0;

  while (Serial.available() > 0 && !frameReady) {
    char c = Serial.read();
    if (receiving) {
      if (c != Config::FRAME_END_MARKER) {
        lineBuffer[idx++] = c;
        if (idx >= Config::SERIAL_LINE_BUFFER_SIZE) idx = Config::SERIAL_LINE_BUFFER_SIZE - 1;
      } else {
        lineBuffer[idx] = '\0';
        receiving = false;
        idx = 0;
        frameReady = true;
      }
    } else if (c == Config::FRAME_START_MARKER) {
      receiving = true;
    }
  }
}

static void handleAbort() {
  StepperEngine::abortMotion();
  CommandBuffer::clear();
  // Enable pins are intentionally left untouched: motors keep full
  // torque and hold the robot's current position.
  Serial.println("OK");
}

static void handleFreeze() {
  StepperEngine::robotPowerEnabled = false;
  StepperEngine::abortMotion();
  CommandBuffer::clear();
  StepperEngine::disableRampsAxes();
  StepperEngine::disableULNCoils();
  Serial.println("OK");
}

static void handleEngage() {
  StepperEngine::robotPowerEnabled = true;
  StepperEngine::enableRampsAxes();
  Serial.println("OK");
}

// Parses "d1,d2,d3,d4,d5,d6,dur_ms" and queues it as a MotionCommand.
static void handleMotionCommand() {
  if (CommandBuffer::isFull()) {
    Serial.println("DROP");
    return;
  }

  MotionCommand cmd;
  char* tok = strtok(lineBuffer, ",");
  for (int i = 0; i < 6; i++) {
    if (!tok) return;
    cmd.delta[i] = atol(tok);
    tok = strtok(NULL, ",");
  }
  if (!tok) return;
  cmd.durationMs = atol(tok);

  if (!StepperEngine::robotPowerEnabled) {
    // Silently accept-and-discard while frozen, matching prior behavior.
    Serial.println("OK");
    return;
  }

  CommandBuffer::push(cmd);
  Serial.println("OK");
}

static void dispatchFrame() {
  if (strcmp(lineBuffer, "ABORT") == 0)  { handleAbort();  return; }
  if (strcmp(lineBuffer, "FREEZE") == 0) { handleFreeze(); return; }
  if (strcmp(lineBuffer, "ENGAGE") == 0) { handleEngage(); return; }
  handleMotionCommand();
}

void poll() {
  receiveFrame();
  if (!frameReady) return;
  frameReady = false;
  dispatchFrame();
}

} // namespace SerialProtocol
