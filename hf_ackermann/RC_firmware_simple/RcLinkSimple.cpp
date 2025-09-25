#include "RcLinkSimple.h"
#include <ctype.h>   // toupper, isspace
#include <string.h>  // strncpy

// ------------------------ Public API ------------------------

void RcLinkSimple::begin(uint16_t telem_period_ms) {
  _telem_period_ms = telem_period_ms;
  _last_telem_ms   = 0;
  _line_len        = 0;
  _inflight.clear();
}

void RcLinkSimple::setTelemetryEnabled(bool en) {
  _telem_period_ms = en ? (_telem_period_ms ? _telem_period_ms : 50) : 0;
}

void RcLinkSimple::setTelemetryPeriod(uint16_t period_ms) {
  _telem_period_ms = period_ms;
}

void RcLinkSimple::service() {
  // 1) Run L2 state machines
  _rc.tick();

  // 2) Read serial and dispatch full lines
  while (_io.available()) {
    char c = _io.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (_line_len > 0) {
        _line[_line_len] = 0;
        handleLine_(_line);
        _line_len = 0;
      }
    } else if (_line_len < LINE_MAX - 1) {
      _line[_line_len++] = c;
    }
  }

  // 3) Supervise long-running actions (DRIVE/ARC)
  superviseInflight_();

  // 4) Telemetry pacing
  if (_telem_period_ms) {
    const uint32_t now = millis();
    if (now - _last_telem_ms >= _telem_period_ms) {
      sendTelemetry_();
      _last_telem_ms = now;
    }
  }
}

// ------------------------ Core handlers ------------------------

void RcLinkSimple::handleLine_(char* line) {
  strtoupper_(line);
  char* p = line;
  char tok[16];

  if (!nextToken_(p, tok, sizeof(tok))) return;

  // STOP
  if (strcmp(tok, "STOP") == 0) {
    stopAll_();
    sendOK_("STOP");
    return;
  }

  // BRAKE
  if (strcmp(tok, "BRAKE") == 0) {
    brakeAll_();
    sendOK_("BRAKE");
    return;
  }

  // CENTER
  if (strcmp(tok, "CENTER") == 0) {
    // Non-blocking active centering
    cancelInflight_();
    _rc.cmdSteerCenter();
    sendOK_("CENTER");
    return;
  }

  // STEER
  if (strcmp(tok, "STEER") == 0) {
    char dir[8];
    if (!nextToken_(p, dir, sizeof(dir))) { sendERR_("STEER missing arg"); return; }
    handleSteer_(dir);
    return;
  }

  // DRIVE
  if (strcmp(tok, "DRIVE") == 0) {
    char dir[8], pwmS[12], durS[12];
    if (!nextToken_(p, dir,  sizeof(dir)))  { sendERR_("DRIVE missing dir"); return; }
    if (!nextToken_(p, pwmS, sizeof(pwmS))) { sendERR_("DRIVE missing pwm"); return; }
    if (!nextToken_(p, durS, sizeof(durS))) { sendERR_("DRIVE missing dur"); return; }
    handleDrive_(dir, pwmS, durS);
    return;
  }

  // ARC
  if (strcmp(tok, "ARC") == 0) {
    char dir[8], pwmS[12], durS[12], endS[8];
    bool haveEnd = true;
    if (!nextToken_(p, dir,  sizeof(dir)))  { sendERR_("ARC missing dir"); return; }
    if (!nextToken_(p, pwmS, sizeof(pwmS))) { sendERR_("ARC missing pwm"); return; }
    if (!nextToken_(p, durS, sizeof(durS))) { sendERR_("ARC missing dur"); return; }
    if (!nextToken_(p, endS, sizeof(endS))) haveEnd = false;
    handleArc_(dir, pwmS, durS, haveEnd ? endS : nullptr);
    return;
  }

  // PARAM
  if (strcmp(tok, "PARAM") == 0) {
    char key[16], val[16];
    if (!nextToken_(p, key, sizeof(key)) || !nextToken_(p, val, sizeof(val))) {
      sendERR_("PARAM needs key val");
      return;
    }
    handleParam_(key, val);
    return;
  }

  // Unknown
  sendERR_("Unknown cmd");
}

void RcLinkSimple::handleDrive_(const char* dir, const char* pwmS, const char* durS) {
  const int pwm = atoi_clamped_(pwmS, 0, 255);
  const int dur = atoi_clamped_(durS, 0, 65535);

  cancelInflight_();
  _rc.cmdDriveStop(); // coast before new run

  if (strcmp(dir, "F") == 0) {
    _rc.cmdDriveForward((uint8_t)pwm, (uint16_t)dur);
  } else if (strcmp(dir, "B") == 0) {
    _rc.cmdDriveBackward((uint8_t)pwm, (uint16_t)dur);
  } else {
    sendERR_("DRIVE dir must be F or B");
    return;
  }

  _inflight.kind     = InflightKind::Drive;
  _inflight.t_end_ms = millis() + (uint32_t)dur;
  strncpy(_inflight.tag, "DRIVE", sizeof(_inflight.tag)-1);
  _inflight.tag[sizeof(_inflight.tag)-1] = 0;

  sendOK_("DRIVE");
}

void RcLinkSimple::handleSteer_(const char* dir) {
  // Steering overrides do not create an inflight timer by themselves
  if (strcmp(dir, "L") == 0) {
    _rc.cmdSteerLockLeft();
  } else if (strcmp(dir, "R") == 0) {
    _rc.cmdSteerLockRight();
  } else if (strcmp(dir, "U") == 0) {
    _rc.cmdSteerUnlock();
  } else {
    sendERR_("STEER arg must be L|R|U");
    return;
  }
  sendOK_("STEER");
}

void RcLinkSimple::handleArc_(const char* dir, const char* pwmS, const char* durS, const char* endSOpt) {
  const int pwm = atoi_clamped_(pwmS, 0, 255);
  const int dur = atoi_clamped_(durS, 0, 65535);

  bool unlock = true; // default U
  if (endSOpt) {
    if (strcmp(endSOpt, "U") == 0) unlock = true;
    else if (strcmp(endSOpt, "H") == 0) unlock = false;
    else { sendERR_("ARC end must be U or H"); return; }
  }

  cancelInflight_();
  _rc.cmdDriveStop(); // coast before new run

  if (strcmp(dir, "L") == 0) {
    _rc.cmdSteerLockLeft();
  } else if (strcmp(dir, "R") == 0) {
    _rc.cmdSteerLockRight();
  } else {
    sendERR_("ARC dir must be L or R");
    return;
  }

  _rc.cmdDriveForward((uint8_t)pwm, (uint16_t)dur);

  _inflight.kind       = InflightKind::Arc;
  _inflight.t_end_ms   = millis() + (uint32_t)dur;
  _inflight.arc_unlock = unlock;
  strncpy(_inflight.tag, "ARC", sizeof(_inflight.tag)-1);
  _inflight.tag[sizeof(_inflight.tag)-1] = 0;

  sendOK_("ARC");
}

void RcLinkSimple::handleParam_(const char* key, const char* val) {
  // Placeholder for live tuning hook — for now just echo.
  _io.print(F("PARAM "));
  _io.print(key);
  _io.print('=');
  _io.println(val);
  sendOK_("PARAM");
}

// ------------------------ Supervision & helpers ------------------------

void RcLinkSimple::stopAll_() {
  cancelInflight_();
  _rc.cmdDriveStop();  // immediate coast
}

void RcLinkSimple::brakeAll_() {
  cancelInflight_();
  _rc.brake();         // blocking quick stop (L2 policy)
}

void RcLinkSimple::cancelInflight_() {
  _inflight.clear();
}

void RcLinkSimple::superviseInflight_() {
  if (!_inflight.active()) return;

  const uint32_t now = millis();
  const bool time_done  = (now >= _inflight.t_end_ms);
  const bool drive_idle = (_rc.driveState() == RcBasicCtr::DriveMode::Idle);

  if (time_done || drive_idle) {
    if (_inflight.kind == InflightKind::Arc && _inflight.arc_unlock) {
      _rc.cmdSteerUnlock();
    }
    sendDONE_(_inflight.tag);
    _inflight.clear();
  }
}

// ------------------------ Telemetry & I/O ------------------------

void RcLinkSimple::sendTelemetry_() {
  const uint8_t drive = static_cast<uint8_t>(_rc.driveState()); // 0 Idle, 1 Fwd, 2 Bwd
  const uint8_t steer = static_cast<uint8_t>(_rc.steerState()); // 0 None, 1 Left, 2 Right
  const uint16_t L = _rc.alignLeftCounts();
  const uint16_t R = _rc.alignRightCounts();

  _io.print(F("TELEM "));
  _io.print(F("drive=")); _io.print(drive);
  _io.print(F(" steer=")); _io.print(steer);
  _io.print(F(" L="));     _io.print(L);
  _io.print(F(" R="));     _io.print(R);
  _io.print(F(" t="));     _io.print(millis());
  _io.println();
}

void RcLinkSimple::sendOK_(const char* tag) {
  _io.print(F("OK "));
  _io.println(tag);
}

void RcLinkSimple::sendDONE_(const char* tag) {
  _io.print(F("DONE "));
  _io.println(tag);
}

void RcLinkSimple::sendERR_(const char* msg) {
  _io.print(F("ERR "));
  _io.println(msg);
}

// ------------------------ util ------------------------

void RcLinkSimple::strtoupper_(char* s) {
  while (*s) { *s = toupper(*s); ++s; }
}

int RcLinkSimple::nextToken_(char*& p, char* out, int maxlen) {
  while (*p && isspace(*p)) ++p;
  if (!*p) return 0;
  int i = 0;
  while (*p && !isspace(*p) && i < maxlen - 1) {
    out[i++] = *p++;
  }
  out[i] = 0;
  return 1;
}

int RcLinkSimple::atoi_clamped_(const char* s, int lo, int hi) {
  long v = atol(s);
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (int)v;
}
