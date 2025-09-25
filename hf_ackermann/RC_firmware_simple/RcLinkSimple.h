#ifndef RCLINK_SIMPLE_H
#define RCLINK_SIMPLE_H

#include <Arduino.h>
#include "RcBasicCtr.h"

/**
 * RcLinkSimple — Minimal Layer 3 over Serial (no heartbeat, no modes).
 *
 * Protocol (case-insensitive tokens, space-separated):
 *   STOP
 *   BRAKE
 *   CENTER
 *   STEER L | STEER R | STEER U
 *   DRIVE F <pwm:0..255> <dur_ms:0..65535>
 *   DRIVE B <pwm:0..255> <dur_ms:0..65535>
 *   ARC   L|R <pwm:0..255> <dur_ms:0..65535> [U|H]   // U=unlock after (default), H=hold
 *   PARAM <key> <value>     // Optional; echoes back (hook for live tuning)
 *
 * Replies:
 *   OK <TAG>                // accepted/start; TAG is one of: STOP|BRAKE|CENTER|STEER|DRIVE|ARC|PARAM
 *   DONE <TAG>              // long-running action finished (DRIVE/ARC)
 *   ERR <message>           // parse/arg errors
 *
 * Telemetry (optional; no modes/heartbeats here):
 *   TELEM drive=<0|1|2> steer=<0|1|2> L=<n> R=<n> t=<ms>
 *     - drive: 0 Idle, 1 Forward, 2 Backward
 *     - steer: 0 None, 1 Left, 2 Right
 *
 * Runtime rules:
 *   - Exactly one long-running action inflight (DRIVE or ARC). A new command
 *     interrupts the previous one: STOP immediately, BRAKE stops fast, others replace.
 *   - DONE is emitted when the action’s time elapses OR earlier if L2 reports Idle.
 *   - No SAFE/DIRECT, no heartbeats. It just does what you say.
 */

class RcLinkSimple {
public:
  RcLinkSimple(RcBasicCtr& rc, Stream& io = Serial)
  : _rc(rc), _io(io) {}

  /**
   * @param telem_period_ms 0 to disable telemetry; otherwise TELEM interval.
   */
  void begin(uint16_t telem_period_ms = 50);

  /** Call this very frequently from loop(). */
  void service();

  // Telemetry controls
  void setTelemetryEnabled(bool en);
  void setTelemetryPeriod(uint16_t period_ms);

  // Status (for optional diagnostics)
  inline uint16_t telemPeriodMs() const { return _telem_period_ms; }
  inline bool telemetryEnabled() const { return _telem_period_ms != 0; }

private:
  RcBasicCtr& _rc;
  Stream&     _io;

  // ---------- line parser ----------
  static constexpr size_t LINE_MAX = 96;
  char     _line[LINE_MAX];
  size_t   _line_len{0};

  // ---------- telemetry pacing ----------
  uint16_t _telem_period_ms{50};
  uint32_t _last_telem_ms{0};

  // ---------- in-flight long action supervisor ----------
  enum class InflightKind : uint8_t { None=0, Drive=1, Arc=2 };
  struct Inflight {
    InflightKind kind{InflightKind::None};
    uint32_t     t_end_ms{0};
    bool         arc_unlock{true};     // ARC: unlock at end
    char         tag[8] = {0};         // "DRIVE" or "ARC" for DONE
    void clear() { kind = InflightKind::None; t_end_ms = 0; arc_unlock = true; tag[0] = 0; }
    bool active() const { return kind != InflightKind::None; }
  } _inflight;

  // ---------- core steps ----------
  void handleLine_(char* line);
  void handleDrive_(const char* dir, const char* pwmS, const char* durS);
  void handleSteer_(const char* dir);
  void handleArc_(const char* dir, const char* pwmS, const char* durS, const char* endSOpt);
  void handleParam_(const char* key, const char* val);

  void stopAll_();       // immediate coast stop (cancels inflight)
  void brakeAll_();      // fast stop (cancels inflight)
  void cancelInflight_();// clear supervisor (no output)

  void superviseInflight_(); // emits DONE <tag> when finished

  // ---------- helpers ----------
  static void strtoupper_(char* s);
  static int  nextToken_(char*& p, char* out, int maxlen);
  static int  atoi_clamped_(const char* s, int lo, int hi);

  void sendTelemetry_();
  void sendOK_(const char* tag);
  void sendDONE_(const char* tag);
  void sendERR_(const char* msg);
};

#endif // RCLINK_SIMPLE_H
