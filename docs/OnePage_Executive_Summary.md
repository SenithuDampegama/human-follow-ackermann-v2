# One-Page Executive Summary — Human-Following Ackermann v2 (OAK-D Pro W)

Author: Senithu (University of Hertfordshire)  
Date: 2025-09-23

## Overview
This project refactors a human-following Ackermann robot into a clean, production-ready stack centered on the Luxonis OAK-D Pro W. It uses on-camera MobileNetSpatialDetectionNetwork for person detection with depth, a lightweight IOU tracker for stable IDs, and a robust decision layer (L5) that commands a Teensy-based drive via an ASCII serial protocol (L4). The system prioritizes safety (stationary SEARCH mode, capped motion) and reliability (serial auto-reconnect), with runtime-tunable steering and a compact RGB+Depth preview.

## Architecture
- OAK-D Pro W: RGB + StereoDepth aligned to RGB, MobileNet spatial NN (300×300).  
- Tracking: IOU-only tracker with greedy IOU matching; stable, monotonic IDs.  
- L5 FSM: INIT → SEARCH → ACQUIRE → FOLLOW → FAULT. SEARCH is stationary (STOP + CENTER once), FOLLOW uses a non-linear steering model with asymmetric left/right gains and caps; distance uses depth `z_m` if available, else bbox area.  
- L4 Serial: Uppercase ASCII commands (`STOP`, `CENTER`, `DRIVE`, `ARC`, `STEER`) with OK/DONE waits and TELEM parsing. Robust auto-reconnect after transient drops.

## Key Features
- Stationary SEARCH/RECOVER: No motion until a confident target reappears.  
- Decisive steering: Tunable non-linear mapping; asymmetric right-turn boost.  
- Safety: STUB default mode, capped motion, wheels-up first run.  
- Robustness: Serial disconnect handling with exponential backoff and resync.  
- Preview: Side-by-side RGB (annotated) + false-color Depth; `--show-scale` for display size.

## How to Run (Examples)
- STUB (safe; 10s bounded):
```
python -m hf_ackermann.app --mode STUB --vision OAKD --show 1 --max-seconds 10
```
- LIVE (wheels-up; bounded):
```
python -m hf_ackermann.app --mode LIVE --vision OAKD --show 1 --max-seconds 10
```
- Smaller preview window:
```
python -m hf_ackermann.app --mode STUB --vision OAKD --show 1 --show-scale 0.6 --max-seconds 10
```

## Steering Tuning (Runtime Flags)
- Non-linearity and minimum: `--arc-gamma 1.6 --pwm-turn-min 140`
- Asymmetry (right-biased defaults):
```
--k-arc-right 820 --cap-right 300 --impulse-right 200 \
--k-arc-left 700  --cap-left 250  --impulse-left 180
```
- Example right-turn boost:
```
--k-arc-right 900 --cap-right 320 --impulse-right 220 --arc-gamma 1.7 --pwm-turn-min 150
```

## Validation
- STUB: INIT → stationary SEARCH; ACQUIRE/FOLLOW engage when a person enters; lock persists across multi-person scenes.  
- LIVE: TELEM ~5 Hz; capped impulses; no motion in SEARCH.  
- Faults: USB drop → FAULT; periodic reconnect; resync (STOP+CENTER); resume SEARCH.

## Risks & Next Steps
- Risks: Open-loop motion without encoders; depth edge cases (reflective/low-texture surfaces); occasional ID swaps with IOU-only tracking.  
- Next: Optional heavy DeepSORT (embeddings), systemd service for autostart, udev rule for stable `/dev/teensy`, telemetry UI.

