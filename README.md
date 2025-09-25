# Human‑Following Ackermann v2 — OAK‑D Pro W

> **v2 of the human‑following Ackermann robot.** On‑camera perception with **OAK‑D Pro W** (Spatial MobileNet‑SSD), lightweight **IOU DeepSORT** tracking, and a robust layered control stack (L5 decision FSM on Jetson, L4 ASCII serial to Teensy 4.1). Safe **stationary SEARCH/RECOVER**, preview **RGB + Depth**, and runtime‑tunable steering.

---

## ✨ Highlights
- **On‑device vision** (OAK‑D Pro W): RGB, StereoDepth (aligned), MobileNet **Spatial** detection (x,y,z in m)
- **Multi‑person lock:** IOU‑only DeepSORT tracker delivers stable `track_id`
- **Layered control:** L5 FSM (INIT→SEARCH→ACQUIRE→FOLLOW→FAULT) → L4 serial → Teensy drivers
- **Safety:** stationary SEARCH/RECOVER (no peeking), bounded runtime (`--max-seconds`), PWM caps
- **Robustness:** serial **auto‑reconnect** on USB drops; optional stable `/dev/teensy` symlink
- **Operator view:** dual preview **RGB + Depth** (scalable window), lock hotkeys `l/r`

---

## 🧱 Repository Layout
```
hf_ackermann/
  app.py                         # CLI entry (STUB/LIVE, OAKD/PINSIGHT/FAKE, tuning)
  control/
    rc_layer4.py                 # L4 serial client (ASCII, OK/DONE/TELEM, reconnect)
    rc_layer5.py                 # L5 decision FSM (stationary SEARCH), steering & distance
  vision/
    vision_oakd_onboard.py       # OAK‑D spatial NN + depth + composite preview
    vision_pinsight_legacy.py    # Legacy PiNSight adapter (fallback)
  tracking/
    deepsort_iou.py              # Lightweight IOU tracker (default)
    deepsort_heavy.py            # Optional wrapper for deep‑sort‑realtime
  tests/
    test_l4_smoke.py             # OK/DONE/TELEM smoke (auto‑skip if no Teensy)
    test_l4_fault.py             # L4 disconnect → auto‑reconnect (mocked)
scripts/
  run_stub_oakd.sh               # safe preview (no motion)
  run_live_oakd.sh               # hardware‑in‑loop (wheels‑up first!)
requirements.txt
README.md  (this)
LICENSE    (MIT)
```

---

## 🔧 Hardware
- **Compute:** NVIDIA Jetson (Orin Nano or better)
- **Camera:** OAK‑D Pro W (wide FOV, Myriad X)
- **MCU:** Teensy 4.1 (actuation & safety)
- **Drivers:** BTS7960 (drive), BTS7980 or equivalent (steering)
- **Power:** LiPo + protection; common ground; short, shielded USB to Teensy

> Tip: Keep OAK‑D and motor returns cleanly separated; ferrites on USB help.

---

## 🚀 Setup
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# (optional) verify OAK‑D
python - <<'PY'
import depthai as dai
with dai.Device() as d:
    print('OAK-D connected:', d.getDeviceName(), d.getMxId())
PY

# serial permissions (Linux)
sudo usermod -aG dialout $USER && newgrp dialout
```

---

## ▶️ Quick Start
### Safe (STUB: no motion)
```bash
./scripts/run_stub_oakd.sh
# or
python -m hf_ackermann.app --mode STUB --vision OAKD --show 1 --max-seconds 8
```

### LIVE (hardware‑in‑loop)
> **Safety:** first runs **wheels‑up** on a stand.
```bash
./scripts/run_live_oakd.sh
# or, explicit port
python -m hf_ackermann.app --mode LIVE --vision OAKD --port /dev/ttyACM0 --show 1 --max-seconds 10
```

**Preview hotkeys:** `l` lock nearest, `r` release, `q/ESC` quit  
*(optional if enabled)* `s` STOP, `c` CENTER

---

## 🧭 Control & Protocols
**L5 FSM:** INIT → **SEARCH** → ACQUIRE → FOLLOW → FAULT  
- **SEARCH/RECOVER:** stationary (STOP + CENTER once), no motion until a valid target appears
- **FOLLOW:** steer by horizontal offset (non‑linear map); hold distance by **depth z** if available else **area**

**L4 (ASCII serial, unchanged):**
```
STOP
CENTER
DRIVE F <pwm> <ms>   | DRIVE B <pwm> <ms>
ARC   L <pwm> <ms> U | ARC R <pwm> <ms> U
```
Replies: `OK`, `DONE <ARC|DRIVE>`, `TELEM drive=… steer=… L=… R=… t=…` (≈5 Hz)

---

## ⚙️ CLI (selected flags)
```
--mode {STUB,LIVE}                --vision {OAKD,FAKE,PINSIGHT}
--port /dev/ttyACM0               --max-seconds 10
--show {0,1}                      --show-scale 0.5

# Steering (turn shaping)
--deadband-x 0.16                 --arc-gamma 1.6
--pwm-turn-min 140                --steer-bias 0.00
--k-arc-left 700  --k-arc-right 820
--cap-left 250    --cap-right 300
--impulse-left 180 --impulse-right 200

# Distance
--target-z 1.2                    # prefer depth (m)
--target-area 0.18 --area-tol 0.04 # fallback via bbox area
```

---

## 🧪 Tests
```bash
pytest -q hf_ackermann/tests   # lightweight unit tests (no hardware needed)
```

---

## 🛟 Safety & Troubleshooting
- Start in **STUB**, then LIVE **wheels‑up**.
- If serial drops mid‑run: system enters **FAULT**, auto‑reconnects, then resumes in SEARCH.
- Permission denied on `/dev/ttyACM*` → add user to `dialout` (see Setup).
- Choppy preview on Jetson → `--show 0` or lower `--show-scale`.

**Stable serial path (optional):** add udev rule for `/dev/teensy` symlink:
```
# /etc/udev/rules.d/99-teensy.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="teensy"
# then
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Use with `--port /dev/teensy`.

---

## 🎛️ Tuning Cheatsheet
- **Right turns too soft?** Increase `--k-arc-right`, `--cap-right`, `--impulse-right`; raise `--pwm-turn-min`.
- **Oscillation?** Increase `--deadband-x` (+0.02) or lower `--arc-gamma` (→1.4–1.6).
- **Far off‑center response slow?** Raise `--arc-gamma` (e.g., 1.8).
- **Prefer depth‑first distance:** set `--target-z` (e.g., 1.2 m).

---

## 📄 License
MIT — see `LICENSE`.

## 🙏 Acknowledgements
DepthAI/OAK‑D (Luxonis), Jetson community, Teensy core devs.

