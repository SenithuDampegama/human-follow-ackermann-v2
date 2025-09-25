#ifndef RCBASICCTR_H
#define RCBASICCTR_H

#include <Arduino.h>
#include "RcCompCtr.h"

/**
 * RcBasicCtr (v2)
 * Layer-2 real-time controller on top of RcCompCtr.
 *
 * - Keeps existing blocking helpers (maintenance only).
 * - Adds a NON-BLOCKING command interface + tick() for real-time loops.
 * - Implements steering "max → safe hold" without grinding (no delays).
 * - Uses IIR filtering for alignment sensors (no micro-delays).
 */
class RcBasicCtr {
public:
  RcBasicCtr();

  // ---------- Boot / Params ----------
  void begin();

  struct Params {
    // Drive
    uint8_t  v_max = 200;         // default cap for drive bursts

    // Spot-stop (reverse burst) parameters
    uint8_t  brake_pwm        = 160; // reverse/forward burst strength
    uint16_t brake_ms         = 80;  // burst duration (blocking)
    uint16_t brake_is_thresh  = 60;  // IS >= this = axle is "active"


    // Steering "max→hold"
    uint8_t  steer_pwm_max   = 255;  // phase A power (your rig needs 255)
    uint16_t steer_drive_ms  = 280;  // phase A push to hard stop (ms)

    // HOLD strategy (choose nudges or preload)
    bool     hold_use_nudges = true;
    uint8_t  hold_preload_pwm= 60;   // if not using nudges
    uint8_t  nudge_pwm       = 180;  // nudge strength
    uint16_t nudge_ms        = 20;   // nudge duration
    uint16_t nudge_interval  = 300;  // interval between nudges
    bool     use_side_sensors = false; // optional: gate nudges with L/R sensors

    // Alignment (IIR + hysteresis on L/R only; middle ignored if noisy)
    float    align_alpha     = 0.35f; // IIR coefficient (0..1)
    uint16_t left_trig_hi    = 900;   // enter threshold
    uint16_t right_trig_hi   = 900;
    uint16_t left_trig_lo    = 880;   // exit threshold (hysteresis)
    uint16_t right_trig_lo   = 880;

    // Active centering
    uint8_t  center_pwm        = 180;  // steering power while centering
    uint16_t center_timeout_ms = 1500; // guard against hunting forever
    uint8_t  center_confirm_n  = 5;    // consecutive ticks both-high to confirm

    // ---- Drive stall protection (ADC counts + timings) ----
    // If either motor on an axle exceeds the threshold for detect_ms,
    // we stop that axle, cool down, then auto-retry (up to max_retries).
    uint16_t stall_adc_thresh_front = 500;
    uint16_t stall_adc_thresh_rear  = 500;
    uint16_t stall_detect_ms        = 60;
    uint16_t stall_cooldown_ms      = 800;
    uint8_t  stall_max_retries      = 3;

    // Timeouts / safety
    uint16_t max_cmd_ms      = 1500;  // generic guard for long actions
  };

  // Declaration only (defined in .cpp to avoid redefinition)
  void setParams(const Params& p);

  // ---------- REAL-TIME NON-BLOCKING API ----------
  // Call this frequently (100–500 Hz). Runs steering/drive state machines,
  // updates IIR sensor filters, and executes timed bursts.
  void tick();

  // Steering locks (persistent until Unlock). Non-blocking.
  void cmdSteerLockLeft();
  void cmdSteerLockRight();
  void cmdSteerUnlock();

  // NEW: Active center (non-blocking) — drives until both L & R are HIGH.
  void cmdSteerCenter();

  // Drive bursts (non-blocking): runs for dur_ms then stops/coasts.
  void cmdDriveForward(uint8_t pwm, uint16_t dur_ms);
  void cmdDriveBackward(uint8_t pwm, uint16_t dur_ms);
  void cmdDriveStop(); // immediate coast

  // Quick telemetry for your upper layer
  enum class SteerLock : uint8_t { None, Left, Right };
  enum class DriveMode : uint8_t { Idle, Forward, Backward };
  SteerLock steerState() const { return _steer_state; }
  DriveMode driveState() const { return _drive_state; }
  uint16_t  alignLeftCounts()  const { return (uint16_t)_L_iir; }
  uint16_t  alignRightCounts() const { return (uint16_t)_R_iir; }

  // ---------- EXISTING (BLOCKING) HELPERS — maintenance only ----------
  bool driveStallDetectionEach(const String &motor);
  bool driveStallDetectionAll();
  bool turnStallDetection();
  bool turnCheck(const String &side);
  bool stopAllMotors();
  bool stopDriveMotors();
  bool stopTurnMotors();     // blocking (waits up to ~100 ms)
  bool brake();              // blocking (calls steerStraight + delays)

  bool steerLeft(int speed); // blocking (loop+delay)
  bool steerRight(int speed);// blocking (loop+delay)
  bool steerStraight();      // blocking (~2 s worst case)
  void turnLeft(int speed);  // raw pass-through
  void turnRight(int speed); // raw pass-through

private:
  RcCompCtr bot;  // HAL

  Params _p{};

  // ---------- IIR alignment sensing (non-blocking) ----------
  float _L_iir{0}, _R_iir{0};
  bool  _L_at_hi{false}, _R_at_hi{false}; // hysteresis latches

  void   updateAlignmentIIR_(); // sample once per tick (no delays)
  bool   sideHi_(bool left) const; // using hysteresis & IIR
  bool   sideLo_(bool left) const;

  // ---------- Drive burst state (non-blocking) ----------
  DriveMode _drive_state{DriveMode::Idle};
  uint32_t  _drive_end_ms{0};
  uint8_t   _drive_pwm{0};

  void driveApply_();
  void driveStop_();

  // ---- Stall monitor per axle ----
  struct StallMon {
    bool     tripped{false};
    uint8_t  retries{0};
    uint32_t t_detect_start{0};  // when over-threshold began
    uint32_t t_release{0};       // when we may try again
  };
  StallMon _stall_front{};
  StallMon _stall_rear{};
  void updateDriveStall_();      // reads currents and updates state
  inline void resetStall_() { _stall_front = StallMon{}; _stall_rear = StallMon{}; }

  // ---------- Steering state (non-blocking) ----------
  SteerLock  _steer_state{SteerLock::None};
  enum class SteerPhase : uint8_t { Idle, DriveToStop, Hold, Center };
  SteerPhase _steer_phase{SteerPhase::Idle};
  uint32_t   _steer_phase_end_ms{0};

  // Nudge scheduler (for Hold)
  uint32_t   _next_nudge_ms{0};
  bool       _nudge_active{false};
  uint32_t   _nudge_end_ms{0};

  // Centering helpers/state
  bool       isCentered_() const; // both L & R HIGH?
  uint8_t    _center_hits{0};     // consecutive both-high counter
  uint32_t   _center_end_ms{0};   // timeout guard

  // helpers for steering phases
  void steerEnter_(SteerLock dir);
  void steerNeutral_();
  void steerTick_();
  void steerApplyDriveToStop_(SteerLock dir);
  void steerApplyHold_(SteerLock dir);
};

#endif // RCBASICCTR_H
