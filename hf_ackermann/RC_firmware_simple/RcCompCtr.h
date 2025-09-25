#ifndef RC_COMP_CTR_H
#define RC_COMP_CTR_H

#include <Arduino.h>

/**
 * RcCompCtr — PURE HAL (foundation layer)
 * - Only primitive actions: set PWM/EN for each half-bridge, read raw ADCs.
 * - No loops, no waits, no decisions, no filtering.
 * - Deterministic and idempotent.
 *
 * Notes:
 * - PWM resolution set once in begin() (default 8-bit). Frequencies set explicitly (~20 kHz) to avoid audible noise.
 * - All speed arguments are clamped internally to [0, PWM_MAX].
 * - "Stop" functions are COAST (EN LOW or both PWMs 0 as appropriate).
 * - New *_BrakeShort() functions perform DYNAMIC BRAKING (short the motor) with EN HIGH.
 * - Keep existing API for compatibility; add enums for zero-heap selectors.
 */

struct AlignmentReadings {
  int left;    // Raw ADC counts
  int middle;  // Raw ADC counts
  int right;   // Raw ADC counts
};

class RcCompCtr {
public:
  /** Motors addressed for current-sense, etc. */
  enum class Motor : uint8_t { BL, BR, FL, FR, TL, TR };

  /** Constructor wires fixed pin map (see .cpp). */
  RcCompCtr();

  /**
   * Initialize pins and PWM foundation (frequency + resolution).
   * @param pwm_freq_hz Target PWM frequency for ALL PWM pins (default 20 kHz).
   * @param pwm_resolution_bits PWM resolution (default 8 bits: 0..255).
   */
  void begin(uint32_t pwm_freq_hz = 20000, uint8_t pwm_resolution_bits = 8);

  // ===================== DRIVE: FRONT AXLE (BTS7960) =====================

  /** Drive front motors forward (coast if speed=0). */
  void frontMotorsForward(int speed);

  /** Drive front motors backward (coast if speed=0). */
  void frontMotorsBackward(int speed);

  /** COAST stop front motors (EN LOW / PWMs 0). */
  void frontMotorsStop();

  /** DYNAMIC BRAKE front motors (short-brake), EN HIGH, both half-bridges driven. */
  void frontMotorsBrakeShort(int pwm_strength);

  // ===================== DRIVE: REAR AXLE (BTS7960) ======================

  /** Drive rear motors forward (coast if speed=0). */
  void rearMotorsForward(int speed);

  /** Drive rear motors backward (coast if speed=0). */
  void rearMotorsBackward(int speed);

  /** COAST stop rear motors (EN LOW / PWMs 0). */
  void rearMotorsStop();

  /** DYNAMIC BRAKE rear motors (short-brake), EN HIGH, both half-bridges driven. */
  void rearMotorsBrakeShort(int pwm_strength);

  // ======================= STEERING (BTS7980 dual HB) ====================

  /** Turn steering LEFT at speed (full-lock linkage assumed above this layer). */
  void turnLeft(int speed);

  /** Turn steering RIGHT at speed. */
  void turnRight(int speed);

  /**
   * COAST stop steering: PWMs=0, EN HIGH (holding enable without drive).
   * Use this for quick neutral without waiting.
   */
  void turnStop();

  /** DYNAMIC BRAKE steering: short-brake to stop fast, EN HIGH, both legs driven. */
  void turnBrakeShort(int pwm_strength);

  // ========================= RAW SENSORS (no filtering) ==================

  /**
   * Read analog current-sense (raw ADC counts) for a motor (enum, zero-heap).
   */
  int getCurrent(Motor m);

  /**
   * Backward-compat selector using String label ("BL","BR","FL","FR","TL","TR").
   * Avoid in hot paths; provided for compatibility.
   */
  int getCurrent(const String& motorLabel);

  /** Read raw alignment sensors. */
  AlignmentReadings getWheelAlignment();

  // ========================= CONSTANTS / LIMITS ==========================

  /** Query the current PWM max value (depends on chosen resolution). */
  inline uint16_t pwmMax() const { return _pwm_max; }

private:
  // ---------- Rear Drive Motor Pins (BTS7960) ----------
  uint8_t BL_PWM, BR_PWM, BL_EN, BR_EN, BL_IS, BR_IS;

  // ---------- Front Drive Motor Pins (BTS7960) ----------
  uint8_t FL_PWM, FR_PWM, FL_EN, FR_EN, FL_IS, FR_IS;

  // ---------- Steering Motor Pins (BTS7980) ------------
  uint8_t TL_PWM, TR_PWM, TL_EN, TR_EN, TL_IS, TR_IS;

  // ---------- Alignment Sensors ------------------------
  uint8_t sensorLeft, sensorMid, sensorRight;

  // ---------- PWM config -------------------------------
  uint16_t _pwm_max{255};    // set by begin() from resolution
  uint8_t  _pwm_bits{8};     // cached resolution bits

  // ---------- helpers ----------------------------------
  inline uint16_t clampPWM(int v) const {
    if (v < 0) return 0;
    uint16_t m = _pwm_max;
    return (v > (int)m) ? m : (uint16_t)v;
  }

  // Low-level write helpers (keep .cpp clean & symmetric)
  void frontWrite_(uint16_t left_pwm, uint16_t right_pwm, bool enHigh);
  void rearWrite_ (uint16_t left_pwm, uint16_t right_pwm, bool enHigh);
  void steerWrite_(uint16_t left_pwm, uint16_t right_pwm, bool enHigh);

  // Map String → enum (compat only)
  static bool labelToMotor_(const String& s, Motor& out);
};

#endif // RC_COMP_CTR_H
