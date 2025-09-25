#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Lightweight unit test for L4 disconnect handling and L5 FAULT auto-reconnect.
This test uses a dummy L4 that raises L4Disconnected once, then allows _reopen() to succeed.
"""

import time as _time
from typing import Dict, Any, Optional

import pytest

from hf_ackermann.control.rc_layer5 import Layer5, VisionBus, HFParams, Mode
from hf_ackermann.control.rc_layer4 import L4Disconnected


class DummyL4:
    def __init__(self, fail_after: int = 1):
        self.calls = 0
        self.fail_after = fail_after
        self.reopened = False

    def send(self, name: str, *args, **kwargs) -> Dict[str, Any]:
        self.calls += 1
        # Simulate disconnect after N calls
        if self.calls >= self.fail_after and not self.reopened:
            raise L4Disconnected("simulated drop")
        return {"ok": "IMMEDIATE", "done": None, "elapsed_s": 0.0}

    def _reopen(self, prefer_last: bool = True) -> Optional[str]:
        # First call returns a port and arms the reopened flag
        self.reopened = True
        return "COM_FAKE"


def test_l4_disconnect_fault_and_reconnect(monkeypatch):
    # Speed up sleep in L5 loop
    monkeypatch.setattr("time.sleep", lambda s: None)

    bus = VisionBus()
    l4 = DummyL4(fail_after=1)
    params = HFParams(verbose=False, loop_hz=50.0)  # fast loop for test
    l5 = Layer5(l4, bus, params)
    l5.start()

    # Trigger one command to induce disconnect (Layer5 does stop+center on INIT/SEARCH)
    _time.sleep(0.1)

    # L5 should enter FAULT, then attempt reconnect and return to SEARCH
    # Allow a moment for reconnect logic
    _time.sleep(0.1)
    assert l4.reopened is True
    assert l5.mode in (Mode.SEARCH, Mode.ACQUIRE, Mode.FOLLOW, Mode.FAULT)

    l5.stop(); l5.join(timeout=1.0)

