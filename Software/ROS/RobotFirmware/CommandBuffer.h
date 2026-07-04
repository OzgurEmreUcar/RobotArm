#pragma once
#include <Arduino.h>
#include "Config.h"
#include "MotionCommand.h"

// Lock-free single-producer / single-consumer ring buffer.
// Producer: SerialProtocol::poll(), running in the main loop.
// Consumer: StepperEngine::loadNextSegment(), also from the main loop.
// (Neither side runs inside the ISR, so no locking is required
// beyond the usual noInterrupts()/interrupts() pairing already
// used where head/tail are reset from ABORT/FREEZE handlers.)
namespace CommandBuffer {

extern volatile MotionCommand buffer[Config::COMMAND_BUFFER_SIZE];
extern volatile uint8_t head;
extern volatile uint8_t tail;

inline uint8_t count() {
  return (head - tail) & (Config::COMMAND_BUFFER_SIZE - 1);
}

inline bool isFull() {
  return ((head + 1) & (Config::COMMAND_BUFFER_SIZE - 1)) == tail;
}

inline uint8_t freeSlots() {
  return Config::COMMAND_BUFFER_SIZE - 1 - count();
}

inline bool isEmpty() {
  return head == tail;
}

// Discards all pending (not-yet-started) commands.
inline void clear() {
  head = tail;
}

// Pushes a command. Caller must have already checked !isFull().
// Fields are copied individually because `buffer` is volatile and
// MotionCommand's implicit copy-assignment can't bind to a volatile object.
inline void push(const MotionCommand& cmd) {
  uint8_t idx = head;
  for (int i = 0; i < 6; i++) buffer[idx].delta[i] = cmd.delta[i];
  buffer[idx].durationMs = cmd.durationMs;
  head = (head + 1) & (Config::COMMAND_BUFFER_SIZE - 1);
}

// Pops the next command into `out`. Returns false if the buffer was empty.
inline bool pop(MotionCommand& out) {
  if (isEmpty()) return false;
  uint8_t idx = tail;
  out.durationMs = buffer[idx].durationMs;
  for (int i = 0; i < 6; i++) out.delta[i] = buffer[idx].delta[i];
  tail = (tail + 1) & (Config::COMMAND_BUFFER_SIZE - 1);
  return true;
}

} // namespace CommandBuffer
