// ======== REAL-TIME EXTENSION ========
#define RCBASICCTR_DEBUG 1

#include "RcBasicCtr.h"

// ---------- ctor / begin ----------
RcBasicCtr::RcBasicCtr() {}

void RcBasicCtr::begin() {
  bot.begin(); // HAL init (pins, PWM freq/res)
  // seed IIR with first read so we don't start at 0
  AlignmentReadings r = bot.getWheelAlignment();
  _L_iir = r.left;
  _R_iir = r.right;
  _L_at_hi = (_L_iir >= _p.left_trig_hi);
  _R_at_hi = (_R_iir >= _p.right_trig_hi);
  _steer_state = SteerLock::None;
  _steer_phase = SteerPhase::Idle;
  _drive_state = DriveMode::Idle;
  _nudge_active = false;
}

// ---------- params ----------
void RcBasicCtr::setParams(const Params& p) { _p = p; }

// ---------- public RT API ----------
void RcBasicCtr::tick() {
  // 1) Update alignment IIR (non-blocking)
  updateAlignmentIIR_();

  // 2) Run steering state machine
  steerTick_();

  // 3) Run drive burst (+ stall monitor inside)
  driveApply_();
}

// Steering commands
void RcBasicCtr::cmdSteerLockLeft()  { steerEnter_(SteerLock::Left);  }
void RcBasicCtr::cmdSteerLockRight() { steerEnter_(SteerLock::Right); }
void RcBasicCtr::cmdSteerUnlock()    { steerNeutral_(); }
void RcBasicCtr::cmdSteerCenter() {
  _steer_state = SteerLock::None;          // dynamic centering, not a lock
  _steer_phase = SteerPhase::Center;
  _center_hits = 0;
  _center_end_ms = millis() + (uint32_t)_p.center_timeout_ms;
  bot.turnStop();
}

// Drive commands (bursts)
void RcBasicCtr::cmdDriveForward(uint8_t pwm, uint16_t dur_ms) {
  _drive_pwm   = (pwm > _p.v_max ? _p.v_max : pwm);
  _drive_state = DriveMode::Forward;
  const uint16_t guard = (_p.max_cmd_ms ? _p.max_cmd_ms : 60000);
  if (dur_ms > guard) dur_ms = guard;
  _drive_end_ms = millis() + (uint32_t)dur_ms;
  resetStall_();
}

void RcBasicCtr::cmdDriveBackward(uint8_t pwm, uint16_t dur_ms) {
  _drive_pwm   = (pwm > _p.v_max ? _p.v_max : pwm);
  _drive_state = DriveMode::Backward;
  const uint16_t guard = (_p.max_cmd_ms ? _p.max_cmd_ms : 60000);
  if (dur_ms > guard) dur_ms = guard;
  _drive_end_ms = millis() + (uint32_t)dur_ms;
  resetStall_();
}

void RcBasicCtr::cmdDriveStop() { driveStop_(); }

// ---------- alignment IIR / hysteresis ----------
void RcBasicCtr::updateAlignmentIIR_() {
  const AlignmentReadings r = bot.getWheelAlignment();
  const float a = _p.align_alpha;
  _L_iir = _L_iir + a * (float(r.left)  - _L_iir);
  _R_iir = _R_iir + a * (float(r.right) - _R_iir);

  // hysteresis latches for “at side”
  if (!_L_at_hi && _L_iir >= _p.left_trig_hi)   _L_at_hi = true;
  if (_L_at_hi  && _L_iir <= _p.left_trig_lo)   _L_at_hi = false;
  if (!_R_at_hi && _R_iir >= _p.right_trig_hi)  _R_at_hi = true;
  if (_R_at_hi  && _R_iir <= _p.right_trig_lo)  _R_at_hi = false;
}

bool RcBasicCtr::sideHi_(bool left) const { return left ? _L_at_hi : _R_at_hi; }
bool RcBasicCtr::sideLo_(bool left) const {
  const float v = left ? _L_iir : _R_iir;
  return left ? (v <= _p.left_trig_lo) : (v <= _p.right_trig_lo);
}

bool RcBasicCtr::isCentered_() const {
  // Your rule: centered when BOTH left and right sensors are HIGH.
  return _L_at_hi && _R_at_hi;
}

// ---------- drive stall monitor (per-axle, only when that axle is driven) ----------
void RcBasicCtr::updateDriveStall_() {
  const uint32_t now = millis();

  // Only “arm” detection when we are actually commanding that axle
  const bool driving = (_drive_state != DriveMode::Idle) && (_drive_pwm > 0);
  const bool frontActive = driving && !_stall_front.tripped;
  const bool rearActive  = driving && !_stall_rear.tripped;

  // ---------- FRONT axle ----------
  {
    const int cFL = bot.getCurrent(RcCompCtr::Motor::FL);
    const int cFR = bot.getCurrent(RcCompCtr::Motor::FR);
    const int cF  = (cFL > cFR ? cFL : cFR); // IS goes HIGH on stall → no inversion

    StallMon &S   = _stall_front;
    const uint16_t TH = _p.stall_adc_thresh_front;

    if (!frontActive) {
      // Not currently driving this axle → do not accumulate detection
      S.t_detect_start = 0;
    } else if (S.tripped) {
      // Cooldown complete? allow retry (up to max retries)
      if (now >= S.t_release && S.retries <= _p.stall_max_retries) {
        S.tripped = false;
        S.t_detect_start = 0;
        #if RCBASICCTR_DEBUG
          Serial.print("[stall] FRONT retry "); Serial.println(S.retries);
        #endif
      }
    } else {
      // Detect sustained high current while driving
      if (cF >= (int)TH) {
        if (S.t_detect_start == 0) S.t_detect_start = now;
        if ((now - S.t_detect_start) >= _p.stall_detect_ms) {
          bot.frontMotorsStop();          // cut only the front axle
          S.tripped   = true;
          S.retries  += 1;
          S.t_release = now + _p.stall_cooldown_ms;
          #if RCBASICCTR_DEBUG
            Serial.print("[stall] FRONT trip  use="); Serial.println(cF);
          #endif
        }
      } else {
        S.t_detect_start = 0; // below threshold resets timer
      }
    }
  }

  // ---------- REAR axle ----------
  {
    const int cBL = bot.getCurrent(RcCompCtr::Motor::BL);
    const int cBR = bot.getCurrent(RcCompCtr::Motor::BR);
    const int cR  = (cBL > cBR ? cBL : cBR); // IS goes HIGH on stall → no inversion

    StallMon &S   = _stall_rear;
    const uint16_t TH = _p.stall_adc_thresh_rear;

    if (!rearActive) {
      S.t_detect_start = 0;
    } else if (S.tripped) {
      if (now >= S.t_release && S.retries <= _p.stall_max_retries) {
        S.tripped = false;
        S.t_detect_start = 0;
        #if RCBASICCTR_DEBUG
          Serial.print("[stall] REAR retry "); Serial.println(S.retries);
        #endif
      }
    } else {
      if (cR >= (int)TH) {
        if (S.t_detect_start == 0) S.t_detect_start = now;
        if ((now - S.t_detect_start) >= _p.stall_detect_ms) {
          bot.rearMotorsStop();           // cut only the rear axle
          S.tripped   = true;
          S.retries  += 1;
          S.t_release = now + _p.stall_cooldown_ms;
          #if RCBASICCTR_DEBUG
            Serial.print("[stall] REAR trip  use="); Serial.println(cR);
          #endif
        }
      } else {
        S.t_detect_start = 0;
      }
    }
  }
}

// ---------- drive burst engine ----------
void RcBasicCtr::driveApply_() {
  const uint32_t now = millis();
  if (_drive_state == DriveMode::Idle) return;

  // time out?
  if (now >= _drive_end_ms) {
    driveStop_();
    return;
  }

  // Update stall monitors from current-sense
  updateDriveStall_();

  if (_drive_state == DriveMode::Forward) {
    if (!_stall_front.tripped) bot.frontMotorsForward(_drive_pwm); else bot.frontMotorsStop();
    if (!_stall_rear.tripped)  bot.rearMotorsForward(_drive_pwm);  else bot.rearMotorsStop();
  } else { // Backward
    if (!_stall_front.tripped) bot.frontMotorsBackward(_drive_pwm); else bot.frontMotorsStop();
    if (!_stall_rear.tripped)  bot.rearMotorsBackward(_drive_pwm);  else bot.rearMotorsStop();
  }
}

void RcBasicCtr::driveStop_() {
  bot.frontMotorsStop();
  bot.rearMotorsStop();
  _drive_state = DriveMode::Idle;
  resetStall_();
}

bool RcBasicCtr::brake() {
  const uint8_t  pwm = (_p.brake_pwm > _p.v_max) ? _p.v_max : _p.brake_pwm;
  const uint16_t ms  = _p.brake_ms;
  if (pwm == 0 || ms == 0) return false;

  // Which axles were actually being commanded when we hit brake?
  DriveMode prev = _drive_state;
  const bool frontWasCmd = (prev != DriveMode::Idle) && !_stall_front.tripped;
  const bool rearWasCmd  = (prev != DriveMode::Idle) && !_stall_rear.tripped;
  if (!frontWasCmd && !rearWasCmd) return false;

  // Pause the burst engine so it won't fight the brake pulse
  _drive_state  = DriveMode::Idle;
  _drive_end_ms = millis();

  // FRONT: coast only (to avoid nose-dive/topple)
  if (frontWasCmd) bot.frontMotorsStop();

  // REAR: hard active brake opposite to last commanded direction
  if (rearWasCmd) {
    if (prev == DriveMode::Forward) {
      bot.rearMotorsBackward(pwm);
    } else if (prev == DriveMode::Backward) {
      bot.rearMotorsForward(pwm);
    } else {
      // If direction unknown, fall back to dynamic short brake on rear
      bot.rearMotorsBrakeShort(pwm);
    }
  }

  // Short, blocking pulse
  delay(ms);

  // Coast rear after the pulse
  if (rearWasCmd) bot.rearMotorsStop();

  resetStall_();
  return true;
}






// ---------- steering state machine ----------
void RcBasicCtr::steerEnter_(SteerLock dir) {
  if (dir == SteerLock::None) { steerNeutral_(); return; }
  _steer_state = dir;
  _steer_phase = SteerPhase::DriveToStop;
  const uint16_t ms = (_p.steer_drive_ms ? _p.steer_drive_ms : 1);
  _steer_phase_end_ms = millis() + (uint32_t)ms;

  // kick Phase A immediately
  steerApplyDriveToStop_(dir);

  // schedule first nudge in HOLD later
  _next_nudge_ms = millis() + _p.nudge_interval;
  _nudge_active = false;
}

void RcBasicCtr::steerNeutral_() {
  bot.turnStop(); // EN high, PWM 0 — fast re-engage, no grinding
  _steer_state = SteerLock::None;
  _steer_phase = SteerPhase::Idle;
  _nudge_active = false;
}

void RcBasicCtr::steerTick_() {
  const uint32_t now = millis();

  switch (_steer_phase) {
    case SteerPhase::DriveToStop:
      if (now >= _steer_phase_end_ms) {
        _steer_phase = SteerPhase::Hold;
        bot.turnStop();
        _nudge_active = false;
        _next_nudge_ms = now + _p.nudge_interval;
      } else {
        steerApplyDriveToStop_(_steer_state);
      }
      break;

    case SteerPhase::Hold:
      // Only meaningful if _steer_state is Left/Right
      if (_steer_state != SteerLock::None) {
        steerApplyHold_(_steer_state);
      } else {
        bot.turnStop();
      }
      break;

    case SteerPhase::Center: {
      // Timeout guard
      if (now >= _center_end_ms) {
        bot.turnStop();
        _steer_phase = SteerPhase::Idle;
        break;
      }

      // Check center with confirmation window
      if (isCentered_()) {
        if (++_center_hits >= _p.center_confirm_n) {
          bot.turnStop();
          _steer_phase = SteerPhase::Idle;
        } else {
          bot.turnStop(); // neutral while confirming
        }
        break;
      } else {
        _center_hits = 0;
      }

      // Decide direction:
      // - only L high => biased left => drive RIGHT
      // - only R high => biased right => drive LEFT
      // - both low    => steer toward the weaker (lower IIR) side
      const uint8_t p = _p.center_pwm;

      if (_L_at_hi && !_R_at_hi) {
        bot.turnRight(p);
      } else if (_R_at_hi && !_L_at_hi) {
        bot.turnLeft(p);
      } else { // both low
        if (_L_iir > _R_iir) {
          bot.turnRight(p);
        } else {
          bot.turnLeft(p);
        }
      }
    } break;

    case SteerPhase::Idle:
    default:
      // nothing
      break;
  }
}

void RcBasicCtr::steerApplyDriveToStop_(SteerLock dir) {
  const uint8_t s = _p.steer_pwm_max; // you said 255 required, keep it here
  if (dir == SteerLock::Left)  bot.turnLeft(s);
  else                         bot.turnRight(s);
}

void RcBasicCtr::steerApplyHold_(SteerLock dir) {
  const uint32_t now = millis();

  if (_p.hold_use_nudges) {
    // Strategy B: periodic nudges (gear-friendly)
    if (_nudge_active) {
      if (now >= _nudge_end_ms) {
        // end pulse → neutral
        bot.turnStop();
        _nudge_active = false;
        _next_nudge_ms = now + _p.nudge_interval;
      } else {
        // keep pulsing in this tick
        const uint8_t p = _p.nudge_pwm;
        if (dir == SteerLock::Left)  bot.turnLeft(p);
        else                         bot.turnRight(p);
      }
    } else {
      // decide whether to emit a nudge now
      bool need_nudge = (now >= _next_nudge_ms);

      // Optional: if you ever enable side sensors, only nudge when we’ve drifted off the stop
      if (_p.use_side_sensors) {
        if (dir == SteerLock::Left) {
          if (sideHi_(true)) need_nudge = false;
          if (sideLo_(true)) need_nudge = true; // fell off left → re-nudge sooner
        } else { // Right
          if (sideHi_(false)) need_nudge = false;
          if (sideLo_(false)) need_nudge = true;
        }
      }

      if (need_nudge) {
        _nudge_active = true;
        _nudge_end_ms = now + _p.nudge_ms;
        const uint8_t p = _p.nudge_pwm;
        if (dir == SteerLock::Left)  bot.turnLeft(p);
        else                         bot.turnRight(p);
      } else {
        // neutral hold between nudges
        bot.turnStop();
      }
    }
  } else {
    // Strategy A: continuous low-PWM preload (simple)
    const uint8_t p = _p.hold_preload_pwm;
    if (p == 0) {
      bot.turnStop(); // no preload requested
    } else {
      if (dir == SteerLock::Left)  bot.turnLeft(p);
      else                         bot.turnRight(p);
    }
  }
}
