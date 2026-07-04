#pragma once

// A single multi-axis motion segment received from the host.
// All six axes move simultaneously; the axis with the most
// steps defines the segment's step rate (see StepperEngine).
struct MotionCommand {
  long delta[6];     // Signed step delta for each of the 6 axes
  long durationMs;   // Requested duration for this segment, in ms
};
