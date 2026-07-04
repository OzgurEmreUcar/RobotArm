#pragma once

// ============================================================
// SerialProtocol
//
// Frame format: "<...>" where "..." is one of:
//   - "ABORT"                     stop current motion, keep torque
//   - "FREEZE"                    stop motion + de-energize all motors
//   - "ENGAGE"                    re-enable motors after FREEZE
//   - "d1,d2,d3,d4,d5,d6,dur_ms"  queue a motion segment
//
// Every recognized frame gets exactly one reply line: "OK", "ACK"
// (from StepperEngine::loadNextSegment), or "DROP" if the command
// buffer is full.
// ============================================================
namespace SerialProtocol {

// Call once per main-loop iteration. Reads any available bytes,
// and once a full "<...>" frame has arrived, parses and executes it.
void poll();

} // namespace SerialProtocol
