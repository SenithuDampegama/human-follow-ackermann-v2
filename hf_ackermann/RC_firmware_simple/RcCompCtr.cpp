#include "RcCompCtr.h"

// ---------------- Constructor: set YOUR real pins ----------------
RcCompCtr::RcCompCtr() {
  // Rear motors (BTS7960) — PWM legs + EN and current-sense
  BL_PWM = 0;   BR_PWM = 1;    // <- your PWM pins
  BL_EN  = 2;   BR_EN  = 3;    // EN pins
  BL_IS  = A9;  BR_IS  = A8;   // analog current-sense

  // Front motors (BTS7960)
  FL_PWM = 5;   FR_PWM = 6;    // PWM legs
  FL_EN  = 7;   FR_EN  = 8;    // EN pins
  FL_IS  = A6;  FR_IS  = A5;   // analog current-sense

  // Steering motor (BTS7980 dual HB)
  TL_PWM = 28;  TR_PWM = 29;   // left/right legs
  TL_EN  = 30;  TR_EN  = 31;   // enables
  TL_IS  = A3;  TR_IS  = A2;   // analog current-sense

  // Alignment sensors (analog)
  sensorLeft  = A10;
  sensorMid   = A11;
  sensorRight = A12;
}

// ---------------- begin(): pin modes + PWM foundation --------------
void RcCompCtr::begin(uint32_t pwm_freq_hz, uint8_t pwm_resolution_bits) {
  _pwm_bits = pwm_resolution_bits;
  _pwm_max  = (1U << _pwm_bits) - 1;

  // Modes
  pinMode(BL_PWM, OUTPUT); pinMode(BR_PWM, OUTPUT);
  pinMode(BL_EN,  OUTPUT); pinMode(BR_EN,  OUTPUT);
  pinMode(FL_PWM, OUTPUT); pinMode(FR_PWM, OUTPUT);
  pinMode(FL_EN,  OUTPUT); pinMode(FR_EN,  OUTPUT);
  pinMode(TL_PWM, OUTPUT); pinMode(TR_PWM, OUTPUT);
  pinMode(TL_EN,  OUTPUT); pinMode(TR_EN,  OUTPUT);

  pinMode(BL_IS, INPUT); pinMode(BR_IS, INPUT);
  pinMode(FL_IS, INPUT); pinMode(FR_IS, INPUT);
  pinMode(TL_IS, INPUT); pinMode(TR_IS, INPUT);

  pinMode(sensorLeft,  INPUT);
  pinMode(sensorMid,   INPUT);
  pinMode(sensorRight, INPUT);

  // PWM base: explicit resolution + ~20 kHz on all PWM pins
  analogWriteResolution(_pwm_bits);
  analogWriteFrequency(BL_PWM, pwm_freq_hz);
  analogWriteFrequency(BR_PWM, pwm_freq_hz);
  analogWriteFrequency(FL_PWM, pwm_freq_hz);
  analogWriteFrequency(FR_PWM, pwm_freq_hz);
  analogWriteFrequency(TL_PWM, pwm_freq_hz);
  analogWriteFrequency(TR_PWM, pwm_freq_hz);

  // Default to COAST
  frontMotorsStop();
  rearMotorsStop();
  turnStop();
}

// ================== Front Drive (BTS7960) ==================
// IMPORTANT: Forward drives FL_PWM, reverse drives FR_PWM (one leg at a time)
void RcCompCtr::frontMotorsForward(int speed) {
  uint16_t s = clampPWM(speed);
  analogWrite(FL_PWM, s); analogWrite(FR_PWM, 0);
  digitalWrite(FL_EN, HIGH); digitalWrite(FR_EN, HIGH);
}

void RcCompCtr::frontMotorsBackward(int speed) {
  uint16_t s = clampPWM(speed);
  analogWrite(FL_PWM, 0); analogWrite(FR_PWM, s);
  digitalWrite(FL_EN, HIGH); digitalWrite(FR_EN, HIGH);
}

void RcCompCtr::frontMotorsStop() {
  analogWrite(FL_PWM, 0); analogWrite(FR_PWM, 0);
  digitalWrite(FL_EN, LOW); digitalWrite(FR_EN, LOW); // COAST
}

void RcCompCtr::frontMotorsBrakeShort(int pwm_strength) {
  uint16_t s = clampPWM(pwm_strength);
  analogWrite(FL_PWM, s); analogWrite(FR_PWM, s);     // short both legs
  digitalWrite(FL_EN, HIGH); digitalWrite(FR_EN, HIGH);
}

// ================== Rear Drive (BTS7960) ==================
void RcCompCtr::rearMotorsForward(int speed) {
  uint16_t s = clampPWM(speed);
  analogWrite(BL_PWM, s); analogWrite(BR_PWM, 0);
  digitalWrite(BL_EN, HIGH); digitalWrite(BR_EN, HIGH);
}

void RcCompCtr::rearMotorsBackward(int speed) {
  uint16_t s = clampPWM(speed);
  analogWrite(BL_PWM, 0); analogWrite(BR_PWM, s);
  digitalWrite(BL_EN, HIGH); digitalWrite(BR_EN, HIGH);
}

void RcCompCtr::rearMotorsStop() {
  analogWrite(BL_PWM, 0); analogWrite(BR_PWM, 0);
  digitalWrite(BL_EN, LOW); digitalWrite(BR_EN, LOW); // COAST
}

void RcCompCtr::rearMotorsBrakeShort(int pwm_strength) {
  uint16_t s = clampPWM(pwm_strength);
  analogWrite(BL_PWM, s); analogWrite(BR_PWM, s);     // short both legs
  digitalWrite(BL_EN, HIGH); digitalWrite(BR_EN, HIGH);
}

// ================== Steering (BTS7980) ==================
void RcCompCtr::turnLeft(int speed) {
  uint16_t s = clampPWM(speed);
  analogWrite(TL_PWM, s); analogWrite(TR_PWM, 0);
  digitalWrite(TL_EN, HIGH); digitalWrite(TR_EN, HIGH);
}

void RcCompCtr::turnRight(int speed) {
  uint16_t s = clampPWM(speed);
  analogWrite(TL_PWM, 0); analogWrite(TR_PWM, s);
  digitalWrite(TL_EN, HIGH); digitalWrite(TR_EN, HIGH);
}

void RcCompCtr::turnStop() {
  analogWrite(TL_PWM, 0); analogWrite(TR_PWM, 0);
  digitalWrite(TL_EN, HIGH); digitalWrite(TR_EN, HIGH); // neutral hold, fast re-engage
}

void RcCompCtr::turnBrakeShort(int pwm_strength) {
  uint16_t s = clampPWM(pwm_strength);
  analogWrite(TL_PWM, s); analogWrite(TR_PWM, s);       // short both legs
  digitalWrite(TL_EN, HIGH); digitalWrite(TR_EN, HIGH);
}

// ================== Current Reads ==================
int RcCompCtr::getCurrent(Motor m) {
  switch (m) {
    case Motor::BL: return analogRead(BL_IS);
    case Motor::BR: return analogRead(BR_IS);
    case Motor::FL: return analogRead(FL_IS);
    case Motor::FR: return analogRead(FR_IS);
    case Motor::TL: return analogRead(TL_IS);
    case Motor::TR: return analogRead(TR_IS);
    default:        return 0;
  }
}

bool RcCompCtr::labelToMotor_(const String& s, Motor& out) {
  if      (s == "BL") { out = Motor::BL; return true; }
  else if (s == "BR") { out = Motor::BR; return true; }
  else if (s == "FL") { out = Motor::FL; return true; }
  else if (s == "FR") { out = Motor::FR; return true; }
  else if (s == "TL") { out = Motor::TL; return true; }
  else if (s == "TR") { out = Motor::TR; return true; }
  return false;
}

int RcCompCtr::getCurrent(const String& motorLabel) {
  Motor m;
  if (labelToMotor_(motorLabel, m)) return getCurrent(m);
  return 0;
}

// ================== Alignment ==================
AlignmentReadings RcCompCtr::getWheelAlignment() {
  AlignmentReadings r;
  r.left   = analogRead(sensorLeft);
  r.middle = analogRead(sensorMid);
  r.right  = analogRead(sensorRight);
  return r;
}
