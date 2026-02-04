/******************************************************************************
  Robotic Glove Controller (Arduino + 2x MPU6050 + 3x Flex + HC-05)
  - MPU6050 I2C: SDA=A4, SCL=A5
  - Addresses: MPU1=0x68 (AD0=GND), MPU2=0x69 (AD0=VCC)
  - Flex sensors: A1 (pinkie), A2 (finger), A3 (thumb)
  - Sends single-character commands over Serial (HC-05)
  - Baud: 4800

  Notes:
  - LED indicator on D3 (optional)
  - Uses simple threshold logic; tune thresholds as needed
******************************************************************************/

#include <Wire.h>

const int MPU1 = 0x68;
const int MPU2 = 0x69;

// Flex sensors
const int pinkie_Data = A1;
const int finger_Data = A2;
const int thumb_Data  = A3;

int pinkie = 0, finger = 0, thumb = 0;

// Auto-calibration bounds
int thumb_high=0, thumb_low=0;
int finger_high=0, finger_low=0;
int pinkie_high=0, pinkie_low=0;
bool calibrated = false;

// MPU values
int16_t AcX, AcY, AcZ;
int minVal = 265, maxVal = 402;
int minVal2 = 265, maxVal2 = 402;

double x=0, y=0, z=0;    // MPU1 angles
double x2=0, y2=0, z2=0; // MPU2 angles

const int LED_PIN = 3;
const int BAUD = 4800;
const int response_time = 100;

// Anti-spam
char lastCmd = 0;
unsigned long lastSentMs = 0;
const unsigned long MIN_CMD_INTERVAL_MS = 80;

static inline void sendCmd(char c) {
  if (c == 0) return;
  unsigned long now = millis();
  if (c == lastCmd && (now - lastSentMs) < MIN_CMD_INTERVAL_MS) return;
  Serial.print(c);
  lastCmd = c;
  lastSentMs = now;
}

void wakeMPU(int addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readMPUAngles(int addr, int which) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // temp bytes
  (void)(Wire.read() << 8 | Wire.read());
  // gyro bytes (unused but must be read)
  (void)(Wire.read() << 8 | Wire.read());
  (void)(Wire.read() << 8 | Wire.read());
  (void)(Wire.read() << 8 | Wire.read());

  int lo = (which == 1) ? minVal : minVal2;
  int hi = (which == 1) ? maxVal : maxVal2;

  int xAng = map(AcX, lo, hi, -90, 90);
  int yAng = map(AcY, lo, hi, -90, 90);
  int zAng = map(AcZ, lo, hi, -90, 90);

  double X = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI) + 4;
  double Y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  double Z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  if (which == 1) { x = X; y = Y; z = Z; }
  else           { x2 = X; y2 = Y; z2 = Z; }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);

  Wire.begin();
  wakeMPU(MPU1);
  wakeMPU(MPU2);

  Serial.begin(BAUD);
  delay(1000);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);

  readMPUAngles(MPU1, 1);
  delay(10);
  readMPUAngles(MPU2, 2);
  delay(10);

  // Read flex
  pinkie = analogRead(pinkie_Data);
  finger = analogRead(finger_Data);
  thumb  = analogRead(thumb_Data);

  // One-time calibration
  if (!calibrated) {
    delay(800);
    thumb_high  = (int)(thumb  * 1.15);  thumb_low  = (int)(thumb  * 0.90);
    finger_high = (int)(finger * 1.03);  finger_low = (int)(finger * 0.80);
    pinkie_high = (int)(pinkie * 1.06);  pinkie_low = (int)(pinkie * 0.80);
    calibrated = true;
  }

  // Command map:
  // L/R base, C/c shoulder, P/p elbow, U/G wrist pitch, O/S wrist roll, F/f gripper

  // MPU1 → base + wrist pitch
  if (x > 15 && x < 55 && y < 30)              sendCmd('L');
  else if (x < 310 && x > 270)                 sendCmd('R');
  else if (y > 60 && y < 80)                   sendCmd('G');
  else if (y < 310 && y > 270)                 sendCmd('U');

  // MPU2 → shoulder
  else if (y2 > 50 && y2 < 85)                 sendCmd('C');
  else if (y2 < 160 && y2 > 120)               sendCmd('c');

  // Flex → elbow + gripper + wrist roll
  else if (pinkie >= pinkie_high)              sendCmd('P');
  else if (pinkie <= pinkie_low)               sendCmd('p');
  else if (finger >= finger_high)              sendCmd('F');
  else if (finger <= finger_low)               sendCmd('f');
  else if (thumb >= thumb_high)                sendCmd('O');
  else if (thumb <= thumb_low)                 sendCmd('S');

  digitalWrite(LED_PIN, LOW);
  delay(response_time);
}
