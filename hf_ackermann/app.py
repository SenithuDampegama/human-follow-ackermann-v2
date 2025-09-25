#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import time
import threading
from typing import Optional, Dict, Any

from .control.rc_layer5 import Layer5, VisionBus, SyntheticVision, HFParams
from .control.rc_layer4 import RcLayer4, L4Error


# ----------------------- STUB L4 (no hardware motion) -----------------------
class FakeRcLayer4:
    """
    Emulates RcLayer4.send(): prints the command and blocks briefly
    to simulate the DONE wait for timed actions (ARC/DRIVE).
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._open = False

    def open(self):
        self._open = True
        print("[STUB L4] open()")

    def close(self):
        print("[STUB L4] close()")
        self._open = False

    def send(self, name: str, *args, **kwargs) -> Dict[str, Any]:
        t0 = time.time()
        name_norm = name.strip().lower()
        done_tag = None
        sleep_s = 0.0
        if name_norm in ("arcleft", "arcright"):
            done_tag = "ARC"
            dur_ms = int(args[1]) if len(args) >= 2 else 0
            sleep_s = min(dur_ms / 1000.0, 0.35)
        elif name_norm in ("driveforward", "driveback"):
            done_tag = "DRIVE"
            dur_ms = int(args[1]) if len(args) >= 2 else 0
            sleep_s = min(dur_ms / 1000.0, 0.35)
        print(f"[STUB L4] send: {name}  args={args}  kw={kwargs}")
        if sleep_s > 0:
            time.sleep(sleep_s)
        return {"ok": done_tag or "IMMEDIATE", "done": done_tag, "elapsed_s": time.time() - t0}


_last_telem = 0.0
def _telem_cb(d: Dict[str, Any]):
    global _last_telem
    now = time.time()
    if now - _last_telem >= 0.2:  # ~5 Hz throttle
        print(f"TELEM drive={d.get('drive')} steer={d.get('steer')} L={d.get('L')} R={d.get('R')} t={d.get('t')}")
        _last_telem = now


def make_params(args: argparse.Namespace) -> HFParams:
    p = HFParams(
        verbose=bool(args.verbose),
        deadband_x=0.16,
        k_arc_pwm=700,
        arc_impulse_ms=180,
        cooldown_ms=90,
        target_area=float(args.target_area),
        area_tol=0.04,
        k_drive_pwm=2200,
        max_drive_pwm=250,
        drive_impulse_ms=240,
        close_back_pwm=140,
        close_back_ms=150,
        recover_arc_pwm=250,
        recover_impulse_ms=180,
        recover_cooldown_ms=120,
        recover_max_ms=3000,
        loop_hz=float(args.loop_hz),
        target_z_m=(float(args.target_z) if args.target_z is not None else None),
        # steering tuning overrides
        arc_gamma=float(args.arc_gamma),
        pwm_turn_min=int(args.pwm_turn_min),
        k_arc_pwm_left=int(args.k_arc_left),
        k_arc_pwm_right=int(args.k_arc_right),
        arc_pwm_cap_left=int(args.cap_left),
        arc_pwm_cap_right=int(args.cap_right),
        arc_impulse_ms_left=int(args.impulse_left),
        arc_impulse_ms_right=int(args.impulse_right),
        steer_bias=float(args.steer_bias),
    )
    return p


def _parse_depth_range(s: str):
    try:
        parts = [p.strip() for p in str(s).split(',')]
        if len(parts) != 2:
            raise ValueError
        return (float(parts[0]), float(parts[1]))
    except Exception:
        return (0.3, 4.0)


def build_vision(args: argparse.Namespace, bus: VisionBus, l5: Layer5):
    vision_name = args.vision.upper()
    if vision_name == "OAKD":
        from .vision.vision_oakd_onboard import OAKDSpatial
        vis = OAKDSpatial(
            bus,
            blob_path=args.blob,
            conf_threshold=float(args.conf),
            fps=int(args.fps),
            show_preview=bool(args.show),
            use_tracker=args.use_tracker,
            lock_cb=l5.lock_target,
            release_cb=l5.release_lock,
            get_locked_id=l5.locked_id,
            verbose=bool(args.verbose),
            depth_range_m=_parse_depth_range(getattr(args, 'depth_range', '0.3,4.0')),
            preview_scale=float(getattr(args, 'show_scale', 0.6)),
        )
        try:
            vis.probe()
        except Exception as e:
            print(f"[VISION] OAK-D unavailable ({e}). Falling back to FAKE.")
            vis = None
        if vis is None:
            synth = SyntheticVision(bus, period_s=max(1/float(args.fps), 0.02))
            synth.start()
            return synth
        vis.start()
        return vis
    elif vision_name == "PINSIGHT":
        from .vision.vision_pinsight_legacy import PiNSightDepthAI
        vis = PiNSightDepthAI(
            bus,
            blob_path=args.blob,
            conf_threshold=float(args.conf),
            fps=int(args.fps),
            show_preview=bool(args.show),
        )
        try:
            vis.probe()
            vis.start()
            return vis
        except Exception as e:
            print(f"[VISION] PiNSIGHT unavailable ({e}). Falling back to FAKE.")
            synth = SyntheticVision(bus, period_s=max(1/float(args.fps), 0.02))
            synth.start()
            return synth
    else:
        synth = SyntheticVision(bus, period_s=max(1/float(args.fps), 0.02))
        synth.start()
        return synth


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(description="Human-follow Ackermann controller")
    ap.add_argument("--mode", choices=["STUB","LIVE"], default="STUB")
    ap.add_argument("--vision", choices=["OAKD","FAKE","PINSIGHT"], default="OAKD")
    ap.add_argument("--port", default=None)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--loop-hz", type=float, default=15.0)
    ap.add_argument("--show", type=int, default=1)
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--blob", default=None)
    ap.add_argument("--use-tracker", choices=["iou","heavy"], default="iou")
    ap.add_argument("--target-area", type=float, default=0.18)
    ap.add_argument("--target-z", type=float, default=None)
    ap.add_argument("--verbose", type=int, default=1)
    ap.add_argument("--depth-range", default="0.3,4.0", help="near,far in meters for depth visualization")
    ap.add_argument("--show-scale", type=float, default=0.6, help="preview scale factor for display window")
    ap.add_argument("--max-seconds", type=float, default=None, help="bounded run time; exit cleanly after N seconds")
    # Steering tuning flags
    ap.add_argument("--arc-gamma", type=float, default=1.6, help="non-linearity exponent for |offset_x|")
    ap.add_argument("--pwm-turn-min", type=int, default=140, help="minimum PWM when turning")
    ap.add_argument("--k-arc-left", type=int, default=700, help="left turn gain")
    ap.add_argument("--k-arc-right", type=int, default=820, help="right turn gain")
    ap.add_argument("--cap-left", type=int, default=250, help="left PWM cap")
    ap.add_argument("--cap-right", type=int, default=300, help="right PWM cap")
    ap.add_argument("--impulse-left", type=int, default=180, help="left impulse duration (ms)")
    ap.add_argument("--impulse-right", type=int, default=200, help="right impulse duration (ms)")
    ap.add_argument("--steer-bias", type=float, default=0.0, help="static bias (+ to the right)")

    args = ap.parse_args(argv)

    # Vision feed
    bus = VisionBus()

    # Choose L4 implementation
    if args.mode.upper() == "LIVE":
        l4 = RcLayer4(port=args.port, telemetry_cb=_telem_cb)
    else:
        l4 = FakeRcLayer4()

    # Start L4 + L5
    l4.open()
    params = make_params(args)
    l5 = Layer5(l4, bus, params)
    l5.start()

    # Vision last so we can pass lock callbacks
    vision_thread = build_vision(args, bus, l5)

    t0 = time.time()
    try:
        while True:
            if args.max_seconds is not None and (time.time() - t0) >= float(args.max_seconds):
                break
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        # Graceful shutdown
        try:
            l5.stop(); l5.join(timeout=1.0)
        except Exception:
            pass
        try:
            vision_thread.stop(); vision_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            l4.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
