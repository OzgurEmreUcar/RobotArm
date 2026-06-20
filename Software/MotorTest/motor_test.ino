#include <Arduino.h>

// ========================================================
// ORIGINAL STEPPER PINS AND VARIABLES (X, Y, Z)
// ========================================================
const byte X_PINS[] = {54, 55, 38}; // Step, Dir, En
const byte Y_PINS[] = {60, 61, 56}; // Step, Dir, En
const byte Z_PINS[] = {46, 48, 62}; // Step, Dir, En

bool moving[3] = {false, false, false}; // 0:X, 1:Y, 2:Z
const unsigned int motorSpeed = 500;

// ========================================================
// NEW ULN2003 MOTOR PINS AND VARIABLES (U, V, W)
// ========================================================
// Pins on RAMPS AUX-3 and AUX-4 {IN1, IN2, IN3, IN4}
const byte U_PINS[] = {16, 17, 23, 25}; // U Axis
const byte V_PINS[] = {27, 29, 31, 33}; // V Axis
const byte W_PINS[] = {35, 37, 39, 41}; // W Axis

bool uln_moving[3] = {false, false, false};   // 0:U, 1:V, 2:W
bool uln_direction[3] = {true, true, true};   // true: Clockwise, false: Counter-Clockwise
int uln_stepIndex[3] = {0, 0, 0};             // Step counter for each motor
unsigned long uln_lastStepTime[3] = {0, 0, 0};

const int uln_stepDelay = 2; // 2ms step delay for ULN2003

// 8-Step Half-Step Sequence
const int halfStepSequence[8][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

// ========================================================
// FUNCTIONS
// ========================================================

// Function to Drive ULN Motors (Non-blocking Structure)
void driveULN(int axis, const byte pins[]) {
  if (millis() - uln_lastStepTime[axis] >= uln_stepDelay) {
    uln_lastStepTime[axis] = millis();

    // Trigger pins according to the active step using a loop
    for (int i = 0; i < 4; i++) {
      digitalWrite(pins[i], halfStepSequence[uln_stepIndex[axis]][i]);
    }

    // Increment or decrement step index based on direction (0-7 loop)
    if (uln_direction[axis]) {
      uln_stepIndex[axis] = (uln_stepIndex[axis] + 1) % 8;
    } else {
      uln_stepIndex[axis] = (uln_stepIndex[axis] - 1 + 8) % 8;
    }
  }
}

// Function to stop ULN motors completely
void stopULN(const byte pins[]) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pins[i], LOW);
  }
}

// ========================================================
// SETUP
// ========================================================
void setup() {
  Serial.begin(9600);

  // Initialize Original X, Y, Z Pins
  for(int i = 0; i < 3; i++) {
    pinMode(X_PINS[i], OUTPUT);
    pinMode(Y_PINS[i], OUTPUT);
    pinMode(Z_PINS[i], OUTPUT);
  }

  // Enable pins LOW (Active)
  digitalWrite(X_PINS[2], LOW);
  digitalWrite(Y_PINS[2], LOW);
  digitalWrite(Z_PINS[2], LOW);

  // Initialize New ULN Pins
  for(int i = 0; i < 4; i++) {
    pinMode(U_PINS[i], OUTPUT);
    pinMode(V_PINS[i], OUTPUT);
    pinMode(W_PINS[i], OUTPUT);
  }

  // Cut power to ULN motors at startup
  stopULN(U_PINS);
  stopULN(V_PINS);
  stopULN(W_PINS);
}

// ========================================================
// MAIN LOOP
// ========================================================
void loop() {
  // Process Serial Commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Command must be at least 2 characters (e.g., "XL", "US")
    if (cmd.length() >= 2) {
      char axis = cmd[0];   // First character: Axis (X, Y, Z, U, V, W)
      char action = cmd[1]; // Second character: Action (L, R, S)

      switch (axis) {
        // Original Steppers
        case 'X':
          if (action == 'L') { digitalWrite(X_PINS[1], LOW);  moving[0] = true; }
          else if (action == 'R') { digitalWrite(X_PINS[1], HIGH); moving[0] = true; }
          else if (action == 'S') { moving[0] = false; }
          break;

        case 'Y':
          if (action == 'L') { digitalWrite(Y_PINS[1], LOW);  moving[1] = true; }
          else if (action == 'R') { digitalWrite(Y_PINS[1], HIGH); moving[1] = true; }
          else if (action == 'S') { moving[1] = false; }
          break;

        case 'Z':
          if (action == 'L') { digitalWrite(Z_PINS[1], LOW);  moving[2] = true; }
          else if (action == 'R') { digitalWrite(Z_PINS[1], HIGH); moving[2] = true; }
          else if (action == 'S') { moving[2] = false; }
          break;

          // ULN Steppers
        case 'U':
          if (action == 'L') { uln_direction[0] = false; uln_moving[0] = true; }
          else if (action == 'R') { uln_direction[0] = true;  uln_moving[0] = true; }
          else if (action == 'S') { uln_moving[0] = false; stopULN(U_PINS); }
          break;

        case 'V':
          if (action == 'L') { uln_direction[1] = false; uln_moving[1] = true; }
          else if (action == 'R') { uln_direction[1] = true;  uln_moving[1] = true; }
          else if (action == 'S') { uln_moving[1] = false; stopULN(V_PINS); }
          break;

        case 'W':
          if (action == 'L') { uln_direction[2] = false; uln_moving[2] = true; }
          else if (action == 'R') { uln_direction[2] = true;  uln_moving[2] = true; }
          else if (action == 'S') { uln_moving[2] = false; stopULN(W_PINS); }
          break;
      }
    }
  }

  // Drive Original Motors
  if (moving[0]) { digitalWrite(X_PINS[0], HIGH); delayMicroseconds(motorSpeed); digitalWrite(X_PINS[0], LOW); }
  if (moving[1]) { digitalWrite(Y_PINS[0], HIGH); delayMicroseconds(motorSpeed); digitalWrite(Y_PINS[0], LOW); }
  if (moving[2]) { digitalWrite(Z_PINS[0], HIGH); delayMicroseconds(motorSpeed); digitalWrite(Z_PINS[0], LOW); }

  // Drive New ULN Motors
  if (uln_moving[0]) driveULN(0, U_PINS);
  if (uln_moving[1]) driveULN(1, V_PINS);
  if (uln_moving[2]) driveULN(2, W_PINS);
}
