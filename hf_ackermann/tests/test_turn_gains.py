#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

from hf_ackermann.control.rc_layer5 import HFParams, Layer5, VisionBus


class _NoopL4:
    def send(self, *a, **k):
        return {"ok": "IMMEDIATE", "done": None, "elapsed_s": 0.0}


def _mk_l5(**over):
    p = HFParams(**over)
    return Layer5(_NoopL4(), VisionBus(), p)


def test_deadband_returns_no_turn():
    l5 = _mk_l5(deadband_x=0.2)
    side, pwm, ms = l5._turn_cmd_from_offset(0.1)
    assert (side, pwm, ms) == ("", 0, 0)


def test_right_ge_left_with_asymmetry():
    l5 = _mk_l5(
        deadband_x=0.0,
        k_arc_pwm_left=700,
        k_arc_pwm_right=820,
        arc_pwm_cap_left=250,
        arc_pwm_cap_right=300,
        pwm_turn_min=100,
    )
    sideL, pwmL, _ = l5._turn_cmd_from_offset(-0.5)
    sideR, pwmR, _ = l5._turn_cmd_from_offset(+0.5)
    assert sideL == "L" and sideR == "R"
    assert pwmR >= pwmL


def test_gamma_affects_large_offsets_more():
    # Set high caps to avoid clamping
    l5a = _mk_l5(deadband_x=0.0, arc_gamma=1.0, arc_pwm_cap_left=10000, arc_pwm_cap_right=10000)
    l5b = _mk_l5(deadband_x=0.0, arc_gamma=2.0, arc_pwm_cap_left=10000, arc_pwm_cap_right=10000)
    # Small vs large offset
    _, p_small_a, _ = l5a._turn_cmd_from_offset(0.2)
    _, p_large_a, _ = l5a._turn_cmd_from_offset(0.8)
    _, p_small_b, _ = l5b._turn_cmd_from_offset(0.2)
    _, p_large_b, _ = l5b._turn_cmd_from_offset(0.8)
    # Gamma 2.0 should increase the ratio large/small compared to gamma 1.0
    ratio_a = p_large_a / max(1, p_small_a)
    ratio_b = p_large_b / max(1, p_small_b)
    assert ratio_b >= ratio_a

