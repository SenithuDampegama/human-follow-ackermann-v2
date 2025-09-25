# Human-Following Ackermann v2 — OAK-D Pro W Refactor

Author: Senithu (University of Hertfordshire)

Date: 2025-09-23

## Table of Contents
- [Executive Summary](#executive-summary)
- [System Overview](#system-overview)
- [Hardware Summary](#hardware-summary)
- [Software Architecture](#software-architecture)
  - [Vision (OAK-D Onboard)](#vision-oak-d-onboard)
  - [Tracking](#tracking)
  - [L5 FSM (Decision Layer)](#l5-fsm-decision-layer)
  - [L4 Serial Client](#l4-serial-client)
- [Control Parameters](#control-parameters)
  - [Steering and Turn Shaping](#steering-and-turn-shaping)
  - [Distance and Motion](#distance-and-motion)
  - [Safety and Robustness](#safety-and-robustness)
- [Setup & Runbook](#setup--runbook)
- [Validation & Tests](#validation--tests)
- [Tuning Guide](#tuning-guide)
- [Known Limitations & Risks](#known-limitations--risks)
- [Future Work](#future-work)
- [Appendices](#appendices)
  - [A. CLI Flags](#a-cli-flags)
  - [B. Serial Protocol](#b-serial-protocol)
  - [C. Safety Checklist](#c-safety-checklist)

---

## Executive Summary
- Goal: Refactor into a production-ready human-following stack using OAK-D Pro W with on-camera MobileNetSpatialDetectionNetwork, an efficient IOU tracker, and a robust L5 controller targeting a Teensy-based drive system.
- Approach: On-camera spatial detections (RGB-aligned depth), lightweight IOU tracker for sticky IDs, L5 FSM with stationary SEARCH and decisive FOLLOW steering, and L4 serial client with robust auto-reconnect.
- Results: Stable target lock among multiple people, safe stationary SEARCH, decisive right turns via asymmetric tuning, and bounded runtime option for CI/smoke.
- Safety: Default STUB mode, capped drive PWM, stationary SEARCH/RECOVER, robust serial fault handling, and explicit wheels-up first-run guidance.
- Constraints: Layer-4 ASCII protocol unchanged; firmware unmodified; FSM maintained with enhancements (lock, z-based distance, stationary SEARCH).
- Extensibility: Optional heavy DeepSORT wrapper, configurable preview scaling, CLI-tunable steering shaping.

## System Overview
A modular pipeline runs OAK-D spatial detections, tracking, decision-making, and serial control.

```mermaid
flowchart LR
  A[OAK-D Pro W\nRGB + StereoDepth\nSpatial NN 300x300] --> B[Detections\n(bbox, conf, z_m)]
  B --> C[IOU Tracker\nStable Track IDs]
  C --> D[VisionBus\n(offset_x, area, quality,\ntrack_id, z_m)]
  D --> E[L5 FSM\nINIT→SEARCH→ACQUIRE→FOLLOW→FAULT]
  E --> F[L4 Serial Client]
  F --> G[Teensy L3 Firmware]
```

ASCII block (compatibility):

```
OAK-D (RGB, StereoDepth, Spatial NN)
       | detections (bbox, conf, z_m)
       v
  IOU Tracker  --> Track IDs
       v
  VisionBus (offset_x, area, quality, track_id, z_m)
       v
  L5 FSM (INIT→SEARCH→ACQUIRE→FOLLOW→FAULT)
       v
  L4 (ASCII serial)  ==USB==>  Teensy (L3)
```

Repository layout:

```
./hf_ackermann/
  app.py                      # CLI entry point
  control/
    rc_layer4.py              # L4 serial client (ASCII, OK/DONE, TELEM, reconnect)
    rc_layer5.py              # L5 FSM (stationary SEARCH, lock, depth distance, steering shaping)
  tracking/
    deepsort_iou.py           # lightweight IOU-based tracker
    deepsort_heavy.py         # optional wrapper (deep-sort-realtime)
  vision/
    vision_oakd_onboard.py    # OAK-D spatial NN + RGB/Depth composite preview
    vision_pinsight_legacy.py # legacy MobileNetSSD fallback (non-spatial)
  tests/
    test_l4_smoke.py
    test_l4_fault.py
    test_turn_gains.py
scripts/
  run_stub_oakd.sh
  run_live_oakd.sh
requirements.txt
README.md
```

## Hardware Summary
- Compute: NVIDIA Jetson (generic) or equivalent Linux SBC with USB3.
- Vision: Luxonis OAK-D Pro W (RGB + stereo IR + active illumination; wide FOV).
- MCU: Teensy 4.1 running Layer-3 firmware (ASCII serial protocol).
- Drivers/Actuation: e.g., BTS7960 H-bridge motor drivers; Ackermann steering.
- Power: Provide adequate isolation, brownout protection, and EMI mitigation (ferrite beads, star-grounding). Ensure powered USB hub if needed.
- Safety: First runs on a stand (wheels up). Confirm emergency stops. Avoid tether snags.

## Software Architecture

### Vision (OAK-D Onboard)
- Pipeline: ColorCamera (THE_1080_P) with ISP scaling → ImageManip (keepAspect) → MobileNetSpatialDetectionNetwork (300×300) with depth aligned to RGB via StereoDepth.
- Output per detection: bbox, confidence, label, spatialCoordinates → z_m = max(0, z_mm/1000).
- Preview: Composite RGB (annotated) | Depth (false-color TURBO), optional scaling `--show-scale` (default 0.6). Hotkeys: `l` lock nearest, `r` release, `q`/ESC quit.
- Robustness: Stereo preset resolved across DepthAI versions; logs DepthAI version and device MXID.

### Tracking
- IOU-only tracker (deepsort_iou.py): greedy IOU assignment, `max_age`, `min_hits`, and `iou_threshold`. Monotonic, sticky IDs. Low-CPU.
- Optional heavy wrapper (deepsort_heavy.py) if `deep-sort-realtime` is installed; same `.update()` API.

### L5 FSM (Decision Layer)
- States: INIT → SEARCH → ACQUIRE → FOLLOW → FAULT.
- SEARCH/RECOVER: Stationary. On entry, STOP + CENTER once; no arcs or nudges until a confident target appears.
- FOLLOW Steering: Non-linear mapping from offset_x to PWM with tunable exponent `arc_gamma`, minimum PWM, and asymmetric gains/caps/impulses (right-biased by default). Cooldown gated locally (not passed into L4).
- Distance Control: Prefer depth `z_m` toward `target_z_m` (with `z_tol_m` band). If depth unavailable, hold via area setpoint (`target_area`, `area_tol`).
- Target Lock: If a `locked_id` is set, only measurements with that track_id are considered valid; public `lock_target(track_id)`/`release_lock()`.
- Fault Handling: Catch `L4Disconnected`, enter FAULT, stop issuing motion, and attempt reconnect with exponential backoff. On success: STOP + CENTER (resync), clear lock, transition to SEARCH.

### L4 Serial Client
- Protocol (unchanged): Uppercase ASCII commands
  - `STOP`, `CENTER`, `STEER L/R/U`, `DRIVE F/B <pwm> <ms>`, `ARC L/R <pwm> <ms> <U|H>`
  - Replies: `OK <TAG>`, `DONE <ARC|DRIVE>`, `ERR ...`; TELEM lines (e.g., `TELEM drive=.. steer=.. L=.. R=.. t=..`).
- IO: 115200 baud, `timeout≈0.05s`, `ACK timeout≈2s`, DONE wait ≥ max(duration+2.5s, 6s). Auto-detect port preferring “Teensy”.
- Robustness: Catches serial exceptions, marks disconnected, and exposes `_reopen()` to re-establish link and resync.

## Control Parameters

### Steering and Turn Shaping
Defaults (HFParams in `rc_layer5.py`) and CLI overrides (see Appendix):

| Parameter | Default | Description |
|---|---:|---|
| deadband_x | 0.16 | No-steer window on |offset_x| |
| arc_gamma | 1.6 | Exponent shaping for turn intensity |
| pwm_turn_min | 140 | Minimum PWM when issuing a turn |
| k_arc_pwm_left | 700 | Left turn gain |
| k_arc_pwm_right | 820 | Right turn gain (bias to the right) |
| arc_pwm_cap_left | 250 | Left PWM cap |
| arc_pwm_cap_right | 300 | Right PWM cap |
| arc_impulse_ms_left | 180 ms | Left arc impulse duration |
| arc_impulse_ms_right | 200 ms | Right arc impulse duration |
| steer_bias | 0.00 | Static bias (+ shifts to right) |
| cooldown_ms | 90 | Local cooldown after actuation |

### Distance and Motion

| Parameter | Default | Description |
|---|---:|---|
| target_z_m | None | Desired meter range (if set, prefer depth) |
| z_tol_m | 0.15 m | Tolerance band around `target_z_m` |
| target_area | 0.18 | Desired bbox area fraction (fallback) |
| area_tol | 0.04 | Tolerance band for area control |
| k_drive_pwm | 2200 | Drive aggression toward setpoint |
| min_drive_pwm | 120 | Minimum forward PWM |
| max_drive_pwm | 190 (HFParams) / 250 (app default) | Forward PWM cap |
| drive_impulse_ms | 240 ms | Forward impulse duration |
| close_back_pwm | 140 | Reverse PWM when too close |
| close_back_ms | 150 ms | Reverse impulse duration |

### Safety and Robustness

| Parameter | Default | Description |
|---|---:|---|
| loop_hz | 15.0 | L5 control loop rate |
| stale_ms | 300 ms | Stale vision age threshold |
| lost_ms | 700 ms | Time without fresh target to declare loss |
| reconnect backoff | 1→2→4→8→10 s | Exponential backoff in FAULT |
| bounded runtime | `--max-seconds` | Clean exit for CI/smoke runs |

## Setup & Runbook

Environment:

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

OAK-D probe (bounded):

```python
# Quick import check (≤5 s)
from hf_ackermann.vision.vision_oakd_onboard import OAKDSpatial
print("OAKD module import OK")
```

Run commands (examples):

```bash
# STUB (safe; no motion)
python -m hf_ackermann.app --mode STUB --vision OAKD --show 1 --max-seconds 10

# LIVE (Teensy connected; wheels-up)
python -m hf_ackermann.app --mode LIVE --vision OAKD --show 1 --max-seconds 10

# Smaller preview window
python -m hf_ackermann.app --mode STUB --vision OAKD --show 1 --show-scale 0.6 --max-seconds 10
```

## Validation & Tests
- STUB + OAK-D: Observe INIT → SEARCH (stationary STOP+CENTER), then ACQUIRE/FOLLOW when a person enters. Confirm target lock stability among multiple people.
- LIVE (wheels-up): TELEM ~5 Hz; steering/drive impulses within caps; no movement in SEARCH.
- Disconnect/Reconnect: Unplug Teensy during operation → FAULT; logs periodic reconnect attempts, resyncs on success (STOP+CENTER), resumes SEARCH.
- Depth vs Area: With `--target-z` set, verify distance holds using depth; otherwise area-based setpoint.

## Tuning Guide
- Boost right turns:

```bash
--k-arc-right 900 --cap-right 320 --impulse-right 220 --arc-gamma 1.7 --pwm-turn-min 150
```

- Reduce oscillation:
  - Increase `deadband_x` slightly (e.g., 0.18–0.20).
  - Lower `arc_gamma` toward linear (e.g., 1.2–1.4) or reduce gains/caps proportionally.
- Depth jitter:
  - Increase `tau_z_ms` or widen `z_tol_m`.
  - Ensure good lighting/texture; avoid reflective/transparent surfaces.

## Known Limitations & Risks
- Open-loop motion (no encoders): true speed/heading not closed-loop; drift possible.
- Lighting/Depth edge cases: specular/low-texture surfaces can degrade depth.
- Tracker ID swaps: rare with IOU-only; embeddings (heavy) can help when available.
- EMI/USB reliability: mitigate with ferrites and quality cabling; auto-reconnect in software.

## Future Work
- Appearance embeddings with heavy DeepSORT; adaptive association.
- Systemd unit for unattended startup; watchdog monitoring.
- Web UI/telemetry (stream logs, preview, tuning sliders).
- Stable `/dev/teensy` symlink via udev for consistent port naming.

## Appendices

### A. CLI Flags
Extracted from `hf_ackermann/app.py` (defaults shown):

| Flag | Type | Default | Description |
|---|---|---|---|
| --mode | {STUB, LIVE} | STUB | Operating mode |
| --vision | {OAKD, FAKE, PINSIGHT} | OAKD | Vision source |
| --port | str | None | Serial port (auto-detect if None) |
| --fps | int | 30 | Camera FPS |
| --loop-hz | float | 15.0 | L5 loop rate |
| --show | int | 1 | Show preview (1/0) |
| --conf | float | 0.5 | Detection confidence threshold |
| --blob | str | None | MobileNet-SSD blob path (optional) |
| --use-tracker | {iou, heavy} | iou | Tracker implementation |
| --target-area | float | 0.18 | Area setpoint (fallback) |
| --target-z | float | None | Depth setpoint (meters) |
| --verbose | int | 1 | Verbose logging |
| --depth-range | str | "0.3,4.0" | Depth viz near,far (meters) |
| --show-scale | float | 0.6 | Preview display scale |
| --max-seconds | float | None | Bounded runtime for CI/smoke |
| --arc-gamma | float | 1.6 | Non-linearity exponent for \|offset_x\| |
| --pwm-turn-min | int | 140 | Minimum PWM when turning |
| --k-arc-left | int | 700 | Left turn gain |
| --k-arc-right | int | 820 | Right turn gain |
| --cap-left | int | 250 | Left PWM cap |
| --cap-right | int | 300 | Right PWM cap |
| --impulse-left | int | 180 | Left impulse duration (ms) |
| --impulse-right | int | 200 | Right impulse duration (ms) |
| --steer-bias | float | 0.0 | Static bias (+ right) |

### B. Serial Protocol
Examples (ASCII over 115200 baud):

```
Host → Teensy:  DRIVE F 180 1200\n
Teensy → Host:  OK DRIVE
...
Teensy → Host:  DONE DRIVE

Host → Teensy:  ARC R 200 700 U\n
Teensy → Host:  OK ARC
...
Teensy → Host:  DONE ARC

Teensy → Host (async): TELEM drive=0 steer=1 L=123 R=456 t=789012
```

### C. Safety Checklist
- [ ] First power-on with wheels up on a stand.
- [ ] Serial port auto-detected; verify `TELEM` at ~5 Hz in LIVE.
- [ ] Confirm `--show 1` preview overlays and depth composite.
- [ ] Verify stationary SEARCH (no motion until target appears).
- [ ] Confirm forward PWM caps are reasonable for environment.
- [ ] Keep emergency stop within immediate reach.

