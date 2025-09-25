#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Layer 5 — Decision Maker for Human-Follow (Ackermann crawler) with directional loss recovery
and optional target lock + depth-aware distance.

State machine: INIT → SEARCH → ACQUIRE → FOLLOW → FAULT

Vision interface (VisionBus.publish):
  - offset_x: float in [-1..+1]
  - area: float in [0..1]
  - quality: float in [0..1]
  - track_id: Optional[int]
  - z_m: Optional[float]  (meters; if provided and params.target_z_m is set, used for distance)

L4 contract: unchanged; we only call RcLayer4.send() with uppercase ASCII commands.
Cooldown gating is local to L5 and is NOT forwarded to L4.
"""

from __future__ import annotations
import time
import threading
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Dict, Any, Literal

# Local import within package
from .rc_layer4 import RcLayer4, L4Error, L4Disconnected


# ---------------------------- Parameters ----------------------------

@dataclass
class HFParams:
    # Steering (ARC)
    deadband_x: float = 0.16          # tuned defaults are provided by app
    k_arc_pwm:   int  = 700
    min_arc_pwm: int  = 120
    max_arc_pwm: int  = 220
    arc_impulse_ms: int = 180

    # Distance (DRIVE) — area based unless target_z_m is set and vision provides z_m
    target_area: float = 0.18
    area_tol:    float = 0.04
    k_drive_pwm: int   = 2200
    min_drive_pwm: int = 120
    max_drive_pwm: int = 190
    drive_impulse_ms: int = 240

    # Depth-based distance control (optional)
    target_z_m: Optional[float] = None    # if set, use z_m to regulate distance
    z_tol_m: float = 0.15                 # +/- tolerance window around target_z_m

    # Back-off when too close
    close_back_pwm: int = 140
    close_back_ms:  int = 150

    # Freshness & loop
    stale_ms:   int   = 300
    lost_ms:    int   = 700
    loop_hz:    float = 15.0
    cooldown_ms:int   = 90

    # Filters (1-pole IIR)
    tau_x_ms:   int = 180
    tau_area_ms:int = 220
    tau_z_ms:   int = 220

    # Logging
    verbose: bool = True

    # Directional recovery on loss
    recover_enable:      bool = True
    recover_max_ms:      int  = 3000
    recover_arc_pwm:     int  = 250
    recover_impulse_ms:  int  = 180
    recover_cooldown_ms: int  = 120
    recover_min_abs_x:   float = 0.10
    quality_gate:        float = 0.60

    # Turning shape & minimum
    arc_gamma: float = 1.6          # non-linearity exponent for |offset_x|
    pwm_turn_min: int = 140         # minimum PWM when we decide to turn

    # Asymmetric left/right gains and caps
    k_arc_pwm_left: int = 700
    k_arc_pwm_right: int = 820      # boost right
    arc_pwm_cap_left: int = 250
    arc_pwm_cap_right: int = 300    # allow a bit more on right
    arc_impulse_ms_left: int = 180
    arc_impulse_ms_right: int = 200 # slightly longer on right

    # Optional static bias (normalized units; + biases to right)
    steer_bias: float = 0.00

    # Search behavior: 'idle' (stationary) or 'peek' (legacy peeks). Default stationary.
    search_mode: Literal['idle', 'peek'] = 'idle'


# ---------------------------- Vision plumbing ----------------------------

@dataclass
class VisionSample:
    offset_x: float
    area:    float
    quality: float
    stamp_ms:int
    track_id: Optional[int] = None
    z_m: Optional[float] = None


class VisionBus:
    """
    Minimal thread-safe mailbox for the latest vision measurement.
    Camera adapter calls publish(); Layer 5 polls latest().
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._latest: Optional[VisionSample] = None

    def publish(
        self,
        offset_x: float,
        area: float,
        quality: float,
        track_id: Optional[int] = None,
        z_m: Optional[float] = None,
    ):
        now_ms = int(time.monotonic() * 1000)
        sample = VisionSample(offset_x, area, quality, now_ms, track_id, z_m)
        with self._lock:
            self._latest = sample

    def latest(self) -> Optional[VisionSample]:
        with self._lock:
            return self._latest


# ---------------------------- Synthetic Vision (unchanged) ----------------------------

class SyntheticVision(threading.Thread):
    def __init__(self, bus: VisionBus, period_s: float = 0.05):
        super().__init__(daemon=True)
        self.bus = bus
        self.period_s = period_s
        self._stop_evt = threading.Event()

    def stop(self): self._stop_evt.set()

    def run(self):
        t0 = time.monotonic()
        while not self._stop_evt.is_set():
            t = time.monotonic() - t0
            offset_x = 0.25 * (1 if int(t) % 8 < 4 else -1) + 0.12 * (((t * 0.8) % 1.0) - 0.5) * 2
            area     = 0.06 + 0.02 * ((t * 0.3) % 1.0)
            quality  = 0.9
            self.bus.publish(offset_x, area, quality, track_id=None, z_m=None)
            time.sleep(self.period_s)


# ---------------------------- Layer 5 Controller ----------------------------

class Mode(Enum):
    INIT = auto()
    SEARCH = auto()
    ACQUIRE = auto()
    FOLLOW = auto()
    FAULT = auto()


def _clamp(v, lo, hi):
    return hi if v > hi else lo if v < lo else v


class Layer5(threading.Thread):
    """
    Decision layer. One command at a time via L4. Run as its own thread.
    """
    def __init__(self, l4: RcLayer4, vision: VisionBus, params: Optional[HFParams] = None):
        super().__init__(daemon=True)
        self.l4 = l4
        self.vision = vision
        self.p = params or HFParams()

        # internal state
        self.mode = Mode.INIT
        self._stop_evt = threading.Event()
        self._cooldown_until_ms = 0
        self._last_seen_ms = 0
        self._centered_once = False
        self._search_stopped = False

        # filters
        self._x_f = 0.0
        self._area_f = 0.0
        self._z_f: Optional[float] = None
        self._have_filter_seed = False

        # directional recovery memory
        self._last_good_x: float = 0.0         # sign tells last-seen side
        self._recover_active: bool = False
        self._recover_until_ms: int = 0        # wall-clock deadline for recovery peeks

        # target lock
        self._locked_id: Optional[int] = None

        # state entry timestamp (ms) for edge-trigger actions
        self._state_entry_ms: int = 0

        # FAULT/reconnect management
        self._reconnect_backoff_s: float = 1.0
        self._reconnect_next_ms: int = 0

    # --------- public API ---------
    def stop(self): self._stop_evt.set()

    def lock_target(self, track_id: Optional[int]):
        self._locked_id = int(track_id) if track_id is not None else None
        if self.p.verbose:
            print(f"[L5] lock_target → {self._locked_id}")

    def release_lock(self):
        if self.p.verbose and self._locked_id is not None:
            print(f"[L5] release_lock (was {self._locked_id})")
        self._locked_id = None

    def locked_id(self) -> Optional[int]:
        return self._locked_id

    # --------- helpers ---------

    def _now_ms(self) -> int:
        return int(time.monotonic() * 1000)

    def _iir(self, prev: float, new: float, tau_ms: int, dt_ms: int) -> float:
        if tau_ms <= 0 or dt_ms <= 0: return new
        a = dt_ms / (tau_ms + dt_ms)
        return (1 - a) * prev + a * new

    def _arc_pwm(self, abs_x: float) -> int:
        return int(_clamp(abs_x * self.p.k_arc_pwm, self.p.min_arc_pwm, self.p.max_arc_pwm))

    def _drive_pwm(self, err: float) -> int:
        return int(_clamp(err * self.p.k_drive_pwm, self.p.min_drive_pwm, self.p.max_drive_pwm))

    def _log(self, *a):
        if self.p.verbose: print(*a)

    # --- fixed: local cooldown override, never forwarded into L4 ---
    def _issue(self, cmd: str, *args, cooldown_override_ms: Optional[int] = None, **kw):
        """Single actuation to L4 with OK/DONE wait + cooldown (optionally overridden)."""
        try:
            res = self.l4.send(cmd, *args, **kw)     # do NOT pass the override into L4
            cool = self.p.cooldown_ms if (cooldown_override_ms is None) else int(cooldown_override_ms)
            self._cooldown_until_ms = self._now_ms() + max(0, cool)
            return res
        except L4Disconnected as e:
            self._log(f"[FAULT] L4 disconnected during {cmd}: {e}")
            self.mode = Mode.FAULT
            # schedule immediate reconnect attempt
            self._reconnect_backoff_s = 1.0
            self._reconnect_next_ms = self._now_ms()
            return None
        except L4Error as e:
            self._log(f"[FAULT] L4 error on {cmd}: {e}")
            self.mode = Mode.FAULT
            try:
                self.l4.send("stop")
            except Exception:
                pass
            return None

    # --------- main loop ---------

    def run(self):
        period_s = 1.0 / max(self.p.loop_hz, 1.0)
        last_loop_ms = self._now_ms()

        while not self._stop_evt.is_set():
            t_ms = self._now_ms()

            # Fetch latest vision
            m = self.vision.latest()
            fresh = False
            good = False
            if m:
                age = t_ms - m.stamp_ms
                fresh = (age <= self.p.stale_ms)
                good = fresh and (m.quality >= self.p.quality_gate)
                if self._locked_id is not None:
                    # Only accept measurements from the locked track
                    good = good and (m.track_id == self._locked_id)
                if good:
                    self._last_seen_ms = t_ms

                    # seed filters on first sample
                    if not self._have_filter_seed:
                        self._x_f, self._area_f = m.offset_x, m.area
                        self._z_f = m.z_m if m.z_m is not None else None
                        self._have_filter_seed = True

                    # update filters
                    dt_ms = max(t_ms - last_loop_ms, 1)
                    self._x_f    = self._iir(self._x_f,    m.offset_x, self.p.tau_x_ms,    dt_ms)
                    self._area_f = self._iir(self._area_f, m.area,     self.p.tau_area_ms, dt_ms)
                    if m.z_m is not None:
                        self._z_f = self._iir(self._z_f if self._z_f is not None else m.z_m,
                                              m.z_m, self.p.tau_z_ms, dt_ms)

                    # remember last-good side
                    self._last_good_x = self._x_f

            # Loss detection (FOLLOW/ACQUIRE → SEARCH + center + arm recovery)
            if self.mode in (Mode.FOLLOW, Mode.ACQUIRE):
                if t_ms - self._last_seen_ms > self.p.lost_ms:
                    self._log("[SEARCH] Lost target → entering SEARCH (stationary)")
                    self.mode = Mode.SEARCH
                    self._state_entry_ms = 0  # force edge-trigger actions
                    self._centered_once = False
                    self._search_stopped = False
                    # ignore recovery when in stationary mode
                    self._recover_active = False

            # Cooldown gate
            if t_ms < self._cooldown_until_ms:
                time.sleep(0.002)
                last_loop_ms = t_ms
                continue

            # ---------------- State machine ----------------
            if self.mode == Mode.INIT:
                self._log("[INIT] Centering, then SEARCH")
                self._issue("stop")
                self._issue("center")
                self.mode = Mode.SEARCH
                self._state_entry_ms = 0
                self._search_stopped = False
                self._recover_active = False

            elif self.mode == Mode.SEARCH:
                # Edge-triggered stop+center on entering SEARCH
                if self._state_entry_ms == 0:
                    self._log("[SEARCH] stationary (stop+center)")
                    self._issue("stop")
                    self._issue("center")
                    self._state_entry_ms = t_ms
                    self._centered_once = True
                    self._search_stopped = True

                # Stationary SEARCH: no motion. Transition only when good target seen.
                if good:
                    self._log("[ACQUIRE] target seen")
                    self._search_stopped = False
                    self._recover_active = False
                    self.mode = Mode.ACQUIRE

            elif self.mode == Mode.ACQUIRE:
                if not good:
                    self._log("[SEARCH] Lost during acquire → SEARCH (stationary)")
                    self.mode = Mode.SEARCH
                    self._state_entry_ms = 0
                    self._search_stopped = False
                    self._recover_active = False
                else:
                    # Nudge heading toward target
                    pwm = self._arc_pwm(abs(self._x_f))
                    dir_cmd = "arcleft" if self._x_f < 0 else "arcright"
                    self._log(f"[ACQUIRE] {dir_cmd} pwm={pwm}")
                    self._issue(dir_cmd, pwm, self.p.arc_impulse_ms, end="U")

                    # Distance control (prefer depth if enabled)
                    if self.p.target_z_m is not None and self._z_f is not None:
                        err = self.p.target_z_m - float(self._z_f)
                        if err > self.p.z_tol_m:
                            fwd = self._drive_pwm(err)
                            self._log(f"[ACQUIRE] forward (z) pwm={fwd}")
                            self._issue("driveforward", fwd, self.p.drive_impulse_ms)
                    else:
                        err = self.p.target_area - self._area_f
                        if err > self.p.area_tol:
                            fwd = self._drive_pwm(err)
                            self._log(f"[ACQUIRE] forward (area) pwm={fwd}")
                            self._issue("driveforward", fwd, self.p.drive_impulse_ms)

                    self.mode = Mode.FOLLOW

            elif self.mode == Mode.FOLLOW:
                if not good:
                    # stale → stop once, maybe center once
                    if not self._search_stopped:
                        self._issue("stop")
                        self._search_stopped = True
                    if not self._centered_once:
                        self._log("[FOLLOW] stale → center once")
                        self._issue("center")
                        self._centered_once = True
                    # Note: recovery arming happens in the loss block above
                else:
                    self._centered_once = False
                    self._search_stopped = False

                    # Priority 1: steering
                    side, pwm, impulse_ms = self._turn_cmd_from_offset(self._x_f)
                    if side == "R":
                        self._log(f"[FOLLOW] steer arcright pwm={pwm}")
                        self._issue("arcright", pwm, impulse_ms, end="U")
                    elif side == "L":
                        self._log(f"[FOLLOW] steer arcleft pwm={pwm}")
                        self._issue("arcleft", pwm, impulse_ms, end="U")
                    else:
                        # Priority 2: distance
                        if self.p.target_z_m is not None and self._z_f is not None:
                            err = self.p.target_z_m - float(self._z_f)
                            if err > self.p.z_tol_m:
                                pwm = self._drive_pwm(err)
                                self._log(f"[FOLLOW] forward (z) pwm={pwm}")
                                self._issue("driveforward", pwm, self.p.drive_impulse_ms)
                            elif err < -self.p.z_tol_m:
                                self._log(f"[FOLLOW] back (z) pwm={self.p.close_back_pwm}")
                                self._issue("driveback", self.p.close_back_pwm, self.p.close_back_ms)
                            else:
                                if not self._centered_once:
                                    self._log("[FOLLOW] in band → center")
                                    self._issue("center")
                                    self._centered_once = True
                        else:
                            err = self.p.target_area - self._area_f
                            if err > self.p.area_tol:
                                pwm = self._drive_pwm(err)
                                self._log(f"[FOLLOW] forward (area) pwm={pwm}")
                                self._issue("driveforward", pwm, self.p.drive_impulse_ms)
                            elif err < -self.p.area_tol:
                                self._log(f"[FOLLOW] back (area) pwm={self.p.close_back_pwm}")
                                self._issue("driveback", self.p.close_back_pwm, self.p.close_back_ms)
                            else:
                                if not self._centered_once:
                                    self._log("[FOLLOW] in band → center")
                                    self._issue("center")
                                    self._centered_once = True

            elif self.mode == Mode.FAULT:
                # Periodically attempt to reopen the port with exponential backoff
                if self._reconnect_next_ms == 0:
                    self._reconnect_backoff_s = 1.0
                    self._reconnect_next_ms = t_ms
                if t_ms >= self._reconnect_next_ms:
                    self._log("[FAULT] Attempting L4 reconnect…")
                    port = None
                    try:
                        port = self.l4._reopen(prefer_last=True)
                    except Exception:
                        port = None
                    if port:
                        self._log(f"[FAULT] Reconnected on {port}; resyncing")
                        # Best-effort resync
                        try:
                            self.l4.send("stop")
                            self.l4.send("center")
                        except Exception:
                            pass
                        # Clear lock, transition to SEARCH (stationary)
                        self.release_lock()
                        self.mode = Mode.SEARCH
                        self._state_entry_ms = 0
                        self._search_stopped = False
                        self._recover_active = False
                        # reset backoff for future
                        self._reconnect_backoff_s = 1.0
                        self._reconnect_next_ms = 0
                    else:
                        # schedule next attempt
                        self._reconnect_backoff_s = min(10.0, max(1.0, self._reconnect_backoff_s * 2.0))
                        self._reconnect_next_ms = t_ms + int(self._reconnect_backoff_s * 1000)
                # Small sleep to avoid busy loop
                time.sleep(0.05)

            last_loop_ms = t_ms
            time.sleep(period_s)

    # --------- steering helper (FOLLOW only) ---------
    def _turn_cmd_from_offset(self, offx: float) -> tuple[str, int, int]:
        """
        Map normalized horizontal offset [-1..+1] to (side, pwm, impulse_ms).
        Uses deadband, bias, non-linear gamma, min PWM, and asymmetric left/right caps.
        Returns: ("L" or "R", pwm, impulse_ms) or ("", 0, 0) if within deadband.
        """
        # Apply optional bias
        try:
            offx = float(offx) + float(self.p.steer_bias)
        except Exception:
            offx = float(offx)

        # Deadband
        if abs(offx) <= self.p.deadband_x:
            return ("", 0, 0)

        # Normalize magnitude beyond deadband to [0..1]
        mag = (abs(offx) - self.p.deadband_x) / max(1e-6, 1.0 - self.p.deadband_x)
        # Non-linear shaping
        mag = max(0.0, min(1.0, mag)) ** float(self.p.arc_gamma)

        if offx > 0:
            # RIGHT turn
            base = int(self.p.k_arc_pwm_right * mag)
            pwm = max(int(self.p.pwm_turn_min), min(int(self.p.arc_pwm_cap_right), base))
            return ("R", pwm, int(self.p.arc_impulse_ms_right))
        else:
            # LEFT turn
            base = int(self.p.k_arc_pwm_left * mag)
            pwm = max(int(self.p.pwm_turn_min), min(int(self.p.arc_pwm_cap_left), base))
            return ("L", pwm, int(self.p.arc_impulse_ms_left))


# ---------------------------- Inline demo (legacy) ----------------------------
if __name__ == "__main__":
    # Legacy in-module demo kept for convenience; new app CLI is hf_ackermann.app
    l4 = RcLayer4()
    try:
        l4.open()
        bus = VisionBus()
        synth = SyntheticVision(bus, period_s=0.05)
        synth.start()
        params = HFParams(verbose=True)
        l5 = Layer5(l4, bus, params)
        l5.start()
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            l5.stop(); l5.join(timeout=1.0)
        except Exception:
            pass
        try:
            synth.stop(); synth.join(timeout=1.0)
        except Exception:
            pass
        try:
            l4.close()
        except Exception:
            pass
