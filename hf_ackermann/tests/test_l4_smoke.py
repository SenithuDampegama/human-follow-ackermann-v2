#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Basic smoke test for Layer 4 OK/DONE/TELEM.
Skips automatically if no Teensy is detected.
"""

import time
from typing import Dict, Any, Optional

import sys

try:
    import pytest  # type: ignore
except Exception:
    pytest = None

from hf_ackermann.control.rc_layer4 import RcLayer4, L4Error


PORT: Optional[str] = None     # auto-detect if None
PRINT_TELEM: bool = True       # True = show TELEM (throttled), False = silent
TELEM_HZ_LIMIT: float = 5.0    # max TELEM prints per second


_last_telem_t = 0.0
def telem_cb(d: Dict[str, Any]):
    global _last_telem_t
    now = time.time()
    if not PRINT_TELEM:
        return
    # throttle to TELEM_HZ_LIMIT
    if now - _last_telem_t >= 1.0 / max(TELEM_HZ_LIMIT, 0.1):
        print(f"TELEM drive={d.get('drive')} steer={d.get('steer')} L={d.get('L')} R={d.get('R')} t={d.get('t')}")
        _last_telem_t = now


def do(l4: RcLayer4, cmd: str, *args, **kw):
    print(f">>> {cmd} {args} {kw if kw else ''}".strip())
    res = l4.send(cmd, *args, **kw)
    print(f"    OK={res['ok']}  DONE={res['done']}  elapsed={res['elapsed_s']:.2f}s")
    return res


def test_l4_smoke():
    l4 = RcLayer4(port=PORT, telemetry_cb=telem_cb if PRINT_TELEM else None)
    try:
        l4.open()
    except L4Error as e:
        if pytest is not None:
            pytest.skip(f"No Teensy detected: {e}")
        else:
            print(f"[SKIP] No Teensy detected: {e}")
            return

    try:
        time.sleep(0.2)
        do(l4, "steerleft")
        do(l4, "arcright", 160, 400)
        do(l4, "driveforward", 150, 600)
        do(l4, "center")
        do(l4, "stop")
        do(l4, "arcleft", 150, 300, end="H")
        time.sleep(0.2)
        do(l4, "steerunlock")
    finally:
        l4.close()


if __name__ == "__main__":
    # Allow running directly without pytest
    try:
        test_l4_smoke()
    except SystemExit:
        pass

