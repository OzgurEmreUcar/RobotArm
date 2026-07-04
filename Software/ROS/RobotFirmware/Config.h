#pragma once
#include <Arduino.h>

// ============================================================
// Hardware configuration: pins, wiring options, buffer sizes.
// Nothing in here is timing-critical; only StepperEngine and
// the ISR touch hardware registers directly.
// ============================================================
namespace Config {

// ---- RAMPS 1.4 driver pins (X, Y, Z -> axis 0, 1, 2) ----
constexpr byte STEP_PIN[3]   = {54, 60, 46};
constexpr byte DIR_PIN[3]    = {55, 61, 48};
constexpr byte ENABLE_PIN[3] = {38, 56, 62};

// Set true for any axis whose physical direction is reversed
// relative to the requested step sign.
constexpr bool DIR_INVERT[3] = {true, false, true};

// ---- ULN2003 / 28BYJ-48 pins (U, V, W -> axis 3, 4, 5) ----
constexpr byte AXIS_U_PINS[4] = {16, 17, 23, 25};
constexpr byte AXIS_V_PINS[4] = {27, 29, 31, 33};
constexpr byte AXIS_W_PINS[4] = {35, 37, 39, 41};

// Half-step drive sequence for 28BYJ-48 steppers via ULN2003.
constexpr int HALF_STEP_SEQUENCE[8][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

// ---- Motion command ring buffer ----
constexpr uint8_t COMMAND_BUFFER_SIZE = 32; // must be a power of two

// ---- Serial protocol ----
constexpr byte SERIAL_LINE_BUFFER_SIZE = 128;
constexpr unsigned long SERIAL_BAUD_RATE = 115200;
constexpr char FRAME_START_MARKER = '<';
constexpr char FRAME_END_MARKER   = '>';

} // namespace Config
