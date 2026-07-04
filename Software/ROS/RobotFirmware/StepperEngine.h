#pragma once
#include <Arduino.h>

// ============================================================
// StepperEngine
//
// Owns all hardware step-pulse generation:
//   - RAMPS 1.4 axes (0,1,2) via direct port writes
//   - ULN2003 axes   (3,4,5) via half-step coil sequencing
//
// Step pulses are generated in the Timer1 Compare-A ISR using
// a Bresenham-style error accumulator so all six axes finish
// their segment at (as close as possible to) the same time,
// with the busiest axis defining the step rate.
// ============================================================
namespace StepperEngine {

struct SegmentDebugInfo {
  long expectedDurationMs;          // duration requested by the host
  long actualDurationMs;            // duration actually measured by the ISR
  long maxSteps;                    // step count of the busiest axis
  unsigned long naturalStepIntervalUs; // computed time between steps
  bool wasClamped;                  // true if naturalStepIntervalUs hit the 10us floor
};

// ---- Lifecycle ----
void initPorts();  // configure pin modes + cache fast port/bitmask lookups
void initTimer();  // configure Timer1 (CTC, prescaler 8 -> 2MHz tick)

// ---- Segment control (called from the main loop) ----
// Pops one command from CommandBuffer and begins executing it.
// No-op if CommandBuffer is empty.
void loadNextSegment();

bool isSegmentActive();

// Returns true exactly once per finished segment, then clears the flag.
bool consumeSegmentFinishedFlag();

SegmentDebugInfo getLastSegmentDebugInfo();

// ---- Direct hardware control (used by serial command handlers) ----
void stopAllStepPins();     // force RAMPS STEP pins low immediately
void disableULNCoils();     // de-energize all ULN2003 coils

void enableRampsAxes();     // EN pins LOW  (motors driven / holding torque)
void disableRampsAxes();    // EN pins HIGH (motors free / no torque)

// Immediately halts any in-progress segment (ISR-safe). Does NOT
// touch enable pins, so RAMPS motors keep holding torque and the
// ULN2003 coils stay in their last energized state.
void abortMotion();

// ---- Shared state ----
extern long currentPosition[6]; // absolute step position per axis
extern bool robotPowerEnabled;  // false while FREEZE is in effect

} // namespace StepperEngine
