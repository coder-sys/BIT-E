/******************************************************************************
  6-DOF Robot Arm Controller (Arduino + PCA9685 + 6 Servos)
  - PCA9685 I2C: SDA=A4, SCL=A5, VCC=5V, GND=GND
  - Servos on PCA9685 channels 0..5
  - Receives single-character commands over Serial (HC-05 recommended)
  - Baud: 4800

  IMPORTANT:
  - Power servos from an EXTERNAL supply via PCA9685 V+
  - Tie all grounds together (Arduino GND, PCA9685 GND, Servo PSU GND)

******************************************************************************/

#include <Wire.h>
#include "HCPCA9685.h"

#define I2CAdd 0x40
HCPCA9685 pca(I2CAdd);

// PCA9685 channel assignments
const uint8_t CH_BASE      = 0;
const uint8_t CH_SHOULDER  = 1;
const uint8_t CH_ELBOW     = 2;
const uint8_t CH_WRIST_P   = 3;
const uint8_t CH_WRIST_R   = 4;
const uint8_t CH_GRIPPER   = 5;

// Parking angles (tune for your arm)
int pos_base     = 90;
int pos_shoulder = 90;
int pos_elbow    = 90;
int pos_wrist_p  = 90;
int pos_wrist_r  = 90;
int pos_gripper  = 90;

// Limits (tune to avoid collisions)
const int MIN_BASE     = 0,   MAX_BASE     = 180;
const int MIN_SHOULDER = 10,  MAX_SHOULDER = 170;
const int MIN_ELBOW    = 10,  MAX_ELBOW    = 170;
const int MIN_WRIST_P  = 10,  MAX_WRIST_P  = 170;
const int MIN_WRIST_R  = 0,   MAX_WRIST_R  = 180;
const int MIN_GRIPPER  = 20,  MAX_GRIPPER  = 140;

// Step sizes (feel)
const int STEP_BASE     = 4;
const int STEP_SHOULDER = 4;
const int STEP_ELBOW    = 4;
const int STEP_WRIST_P  = 4;
const int STEP_WRIST_R  = 6;
const int STEP_GRIPPER  = 6;

const int BAUD = 4800;
const int RESPONSE_DELAY_MS = 5;

static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void writeServo(uint8_t ch, int angle) {
  angle = clampi(angle, 0, 180);
  pca.Servo(ch, angle);
}

void applyAll() {
  writeServo(CH_BASE,     pos_base);
  writeServo(CH_SHOULDER, pos_shoulder);
  writeServo(CH_ELBOW,    pos_elbow);
  writeServo(CH_WRIST_P,  pos_wrist_p);
  writeServo(CH_WRIST_R,  pos_wrist_r);
  writeServo(CH_GRIPPER,  pos_gripper);
}

void setup() {
  Serial.begin(BAUD);

  pca.Init(SERVO_MODE);
  pca.Sleep(false);

  delay(800);
  applyAll(); // park
}

void loop() {
  if (Serial.available() <= 0) return;

  char cmd = (char)Serial.read();

  // Base yaw
  if      (cmd == 'L') pos_base = clampi(pos_base - STEP_BASE, MIN_BASE, MAX_BASE);
  else if (cmd == 'R') pos_base = clampi(pos_base + STEP_BASE, MIN_BASE, MAX_BASE);

  // Shoulder pitch
  else if (cmd == 'C') pos_shoulder = clampi(pos_shoulder + STEP_SHOULDER, MIN_SHOULDER, MAX_SHOULDER);
  else if (cmd == 'c') pos_shoulder = clampi(pos_shoulder - STEP_SHOULDER, MIN_SHOULDER, MAX_SHOULDER);

  // Elbow pitch
  else if (cmd == 'P') pos_elbow = clampi(pos_elbow + STEP_ELBOW, MIN_ELBOW, MAX_ELBOW);
  else if (cmd == 'p') pos_elbow = clampi(pos_elbow - STEP_ELBOW, MIN_ELBOW, MAX_ELBOW);

  // Wrist pitch
  else if (cmd == 'U') pos_wrist_p = clampi(pos_wrist_p + STEP_WRIST_P, MIN_WRIST_P, MAX_WRIST_P);
  else if (cmd == 'G') pos_wrist_p = clampi(pos_wrist_p - STEP_WRIST_P, MIN_WRIST_P, MAX_WRIST_P);

  // Wrist roll
  else if (cmd == 'O') pos_wrist_r = clampi(pos_wrist_r + STEP_WRIST_R, MIN_WRIST_R, MAX_WRIST_R);
  else if (cmd == 'S') pos_wrist_r = clampi(pos_wrist_r - STEP_WRIST_R, MIN_WRIST_R, MAX_WRIST_R);

  // Gripper
  else if (cmd == 'F') pos_gripper = clampi(pos_gripper + STEP_GRIPPER, MIN_GRIPPER, MAX_GRIPPER);
  else if (cmd == 'f') pos_gripper = clampi(pos_gripper - STEP_GRIPPER, MIN_GRIPPER, MAX_GRIPPER);
  else return; // ignore unknown

  applyAll();
  delay(RESPONSE_DELAY_MS);
}
