#include "StepperEngine.h"
#include "Config.h"
#include "CommandBuffer.h"
#include "MotionCommand.h"

namespace StepperEngine {

// ---- Fast port/bitmask lookups (avoids digitalWrite overhead in the ISR) ----
static volatile uint8_t* stepPortReg[3];
static uint8_t stepBitMask[3];

static volatile uint8_t* ulnPortReg[3][4];
static uint8_t ulnBitMask[3][4];

static const byte* const ULN_AXIS_PINS[3] = {
  Config::AXIS_U_PINS, Config::AXIS_V_PINS, Config::AXIS_W_PINS
};

static volatile int ulnStepIndex[3] = {0, 0, 0};

// ---- Segment state, shared between the main loop and the ISR ----
static volatile bool segmentActive = false;
static volatile bool segmentJustFinished = false;

static volatile long segStepsAbs[6];
static volatile long segRemaining[6];
static volatile long segError[6];
static volatile bool segDir[6];
static volatile long segMaxSteps;

static volatile bool prevStepPinHigh[3];

long currentPosition[6] = {0, 0, 0, 0, 0, 0};
bool robotPowerEnabled = true;

// ---- Debug: last-segment timing, sampled by the main loop after each DON ----
static volatile unsigned long segStartMs = 0;
static volatile long segExpectedDurationDbg = 0;
static volatile long segActualDurationDbg = 0;
static volatile long segMaxStepsDbg = 0;
static volatile unsigned long segNaturalUsDbg = 0;
static volatile bool segClampedDbg = false;

// ============================================================
void initPorts() {
  for (int i = 0; i < 3; i++) {
    pinMode(Config::STEP_PIN[i], OUTPUT);   digitalWrite(Config::STEP_PIN[i], LOW);
    pinMode(Config::DIR_PIN[i], OUTPUT);    digitalWrite(Config::DIR_PIN[i], LOW);
    pinMode(Config::ENABLE_PIN[i], OUTPUT); digitalWrite(Config::ENABLE_PIN[i], LOW);

    stepPortReg[i] = portOutputRegister(digitalPinToPort(Config::STEP_PIN[i]));
    stepBitMask[i] = digitalPinToBitMask(Config::STEP_PIN[i]);
  }

  for (int u = 0; u < 3; u++) {
    for (int p = 0; p < 4; p++) {
      byte pin = ULN_AXIS_PINS[u][p];
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
      ulnPortReg[u][p] = portOutputRegister(digitalPinToPort(pin));
      ulnBitMask[u][p] = digitalPinToBitMask(pin);
    }
  }
}

void initTimer() {
  // Timer1, CTC mode, prescaler = 8 -> 2 MHz tick (0.5us per tick).
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  OCR1A = 0xFFFF;
  interrupts();
}

// ============================================================
void stopAllStepPins() {
  for (int i = 0; i < 3; i++) {
    *stepPortReg[i] &= ~stepBitMask[i];
    prevStepPinHigh[i] = false;
  }
}

void disableULNCoils() {
  for (int u = 0; u < 3; u++) {
    for (int p = 0; p < 4; p++) {
      *ulnPortReg[u][p] &= ~ulnBitMask[u][p];
    }
  }
}

void enableRampsAxes() {
  for (int i = 0; i < 3; i++) digitalWrite(Config::ENABLE_PIN[i], LOW);
}

void disableRampsAxes() {
  for (int i = 0; i < 3; i++) digitalWrite(Config::ENABLE_PIN[i], HIGH);
}

void abortMotion() {
  noInterrupts();
  segmentActive = false;
  segmentJustFinished = false;
  interrupts();
  stopAllStepPins();
}

bool isSegmentActive() {
  return segmentActive;
}

bool consumeSegmentFinishedFlag() {
  if (!segmentJustFinished) return false;
  segmentJustFinished = false;
  return true;
}

SegmentDebugInfo getLastSegmentDebugInfo() {
  noInterrupts();
  SegmentDebugInfo info;
  info.expectedDurationMs = segExpectedDurationDbg;
  info.actualDurationMs   = segActualDurationDbg;
  info.maxSteps           = segMaxStepsDbg;
  info.naturalStepIntervalUs = segNaturalUsDbg;
  info.wasClamped         = segClampedDbg;
  interrupts();
  return info;
}

// ============================================================
void loadNextSegment() {
  MotionCommand cmd;
  if (!CommandBuffer::pop(cmd)) return;

  Serial.println("ACK");

  long durationMs = cmd.durationMs;
  if (durationMs <= 0) durationMs = 2;

  long localSteps[6];
  bool localDir[6];
  long maxAbsSteps = 0;

  for (int i = 0; i < 6; i++) {
    long delta = cmd.delta[i];
    localSteps[i] = labs(delta);
    localDir[i] = (delta > 0);
    if (localSteps[i] > maxAbsSteps) maxAbsSteps = localSteps[i];

    if (delta != 0 && i < 3) {
      bool pinState = localDir[i];
      if (Config::DIR_INVERT[i]) pinState = !pinState;
      digitalWrite(Config::DIR_PIN[i], pinState ? HIGH : LOW);
    }
  }

  if (maxAbsSteps == 0) {
    segmentJustFinished = true;
    segExpectedDurationDbg = durationMs;
    segActualDurationDbg = 0;
    segMaxStepsDbg = 0;
    segNaturalUsDbg = 0;
    segClampedDbg = false;
    return;
  }

  unsigned long naturalUs = (durationMs * 1000UL) / maxAbsSteps;
  bool clampedNow = (naturalUs < 10);
  if (clampedNow) naturalUs = 10;
  unsigned long naturalTicks = naturalUs * 2UL; // 2 ticks per us at 2MHz

  if (naturalTicks < 4) naturalTicks = 4;
  if (naturalTicks > 65535) naturalTicks = 65535;

  segExpectedDurationDbg = durationMs;
  segMaxStepsDbg = maxAbsSteps;
  segNaturalUsDbg = naturalUs;
  segClampedDbg = clampedNow;
  segStartMs = millis();

  noInterrupts();
  for (int i = 0; i < 6; i++) {
    segStepsAbs[i] = localSteps[i];
    segRemaining[i] = localSteps[i];
    segError[i] = 0;
    segDir[i] = localDir[i];
  }
  segMaxSteps = maxAbsSteps;

  prevStepPinHigh[0] = prevStepPinHigh[1] = prevStepPinHigh[2] = false;
  segmentActive = true;

  TCNT1 = 0;
  OCR1A = (uint16_t)(naturalTicks - 1);
  interrupts();
}

} // namespace StepperEngine

// ============================================================
// TIMER1 COMPARE-A ISR
//
// Fires at the segment's step rate. Each tick:
//   1. Clears any RAMPS STEP pins that were pulsed high last tick.
//   2. For each axis with remaining steps, accumulates error and
//      emits a step pulse (RAMPS) or half-step coil update (ULN2003)
//      whenever the accumulator overflows segMaxSteps - this is a
//      Bresenham-style algorithm that keeps every axis's steps
//      evenly spread across the segment relative to the busiest axis.
//   3. Marks the segment finished once every axis has no steps left.
// ============================================================
ISR(TIMER1_COMPA_vect) {
  using namespace StepperEngine;

  for (int i = 0; i < 3; i++) {
    if (prevStepPinHigh[i]) {
      *stepPortReg[i] &= ~stepBitMask[i];
      prevStepPinHigh[i] = false;
    }
  }

  if (!segmentActive) return;

  for (int i = 0; i < 6; i++) {
    if (segRemaining[i] > 0) {
      segError[i] += segStepsAbs[i];
      if (segError[i] >= segMaxSteps) {
        segError[i] -= segMaxSteps;
        segRemaining[i]--;

        if (i < 3) {
          *stepPortReg[i] |= stepBitMask[i];
          prevStepPinHigh[i] = true;
        } else {
          int u = i - 3;
          bool dir = segDir[i];
          if (u == 1) dir = !dir; // axis V is mechanically mirrored

          if (dir) ulnStepIndex[u] = (ulnStepIndex[u] + 1) & 7;
          else     ulnStepIndex[u] = (ulnStepIndex[u] - 1 + 8) & 7;

          for (int p = 0; p < 4; p++) {
            if (Config::HALF_STEP_SEQUENCE[ulnStepIndex[u]][p]) {
              *ulnPortReg[u][p] |= ulnBitMask[u][p];
            } else {
              *ulnPortReg[u][p] &= ~ulnBitMask[u][p];
            }
          }
        }
        currentPosition[i] += (segDir[i] ? 1 : -1);
      }
    }
  }

  bool allDone = true;
  for (int i = 0; i < 6; i++) {
    if (segRemaining[i] > 0) { allDone = false; break; }
  }

  if (allDone) {
    segmentActive = false;
    segmentJustFinished = true;
    segActualDurationDbg = (long)(millis() - segStartMs);
  }
}
