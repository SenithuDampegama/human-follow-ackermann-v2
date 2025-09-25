#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Layer 4 (Jetson-side) control for your Layer 3 serial protocol.

WHAT THIS PROVIDES
- Single-call API: l4.send("arcright", 255, 500)  → sends "ARC R 255 500 U"
  and blocks until the firmware replies OK/DONE (with robust timeouts).
- Command aliases & argument validation.
- Built-in ACK/DONE handling; ERR raises an exception.
- Optional telemetry callback (receives parsed TELEM dicts).
- Auto port detection for Teensy; or set PORT manually.

BASIC USAGE
-----------
from rc_layer4 import RcLayer4

l4 = RcLayer4()           # auto-detects Teensy port, 115200 baud
l4.open()

# examples:
l4.send("steerleft")                 # OK STEER
l4.send("driveforward", 200, 1500)   # OK DRIVE ... DONE DRIVE
l4.send("arcright", 255, 500)        # OK ARC   ... DONE ARC (auto-unlock)
l4.send("center")                    # OK CENTER
l4.send("stop")                      # OK STOP

l4.close()

NOTES
-----
- For drive/arc you pass (pwm, duration_ms). For arc you can also pass end="H" to hold lock.
  Example: l4.send("arcleft", 180, 2000, end="H")
- Default timeout for DONE is max( duration_ms + 2500ms, 6s ). Adjust per call via timeout=.
- Telemetry lines from Layer 3 are ignored unless you pass telemetry_cb= in constructor.
- You can also call convenience helpers: l4.arcright(255,500), l4.driveforward(180,2000), etc.
"""

import time
from typing import Optional, Callable, Dict, Any, Tuple, List

try:
    import serial
    from serial.tools import list_ports
except ImportError as e:
    raise SystemExit("pyserial missing. Install with: pip install pyserial") from e


# ---------------------------- configuration defaults ----------------------------

_DEFAULT_BAUD = 115200
_DEFAULT_READ_TIMEOUT = 0.05   # seconds; used for non-blocking-ish line reads
_DEFAULT_ACK_TIMEOUT = 2.0     # seconds; waiting for OK/ERR
_MIN_DONE_TIMEOUT = 6.0        # seconds; minimum DONE wait
_DONE_FUDGE_MS = 2500          # ms extra on top of commanded duration


# ---------------------------- utilities ----------------------------

def _find_teensy_port() -> Optional[str]:
    """
    Try to find a Teensy-like port. Preference for 'Teensy' in description,
    otherwise first USB-serial-ish candidate.
    """
    candidates: List[Tuple[str, str]] = []
    for p in list_ports.comports():
        desc = f"{p.manufacturer or ''} {p.product or ''} {p.description or ''}".strip()
        if "Teensy" in desc:
            return p.device
        if any(k in (desc or "").lower() for k in ["usb", "serial", "tty", "acm", "usbmodem"]):
            candidates.append((p.device, desc))
    return candidates[0][0] if candidates else None


class L4Error(RuntimeError):
    """Raised for device ERR replies or protocol timeouts."""


class L4PortError(Exception):
    """Base for Layer-4 port-level errors (disconnects, I/O)."""


class L4Disconnected(L4PortError):
    """Raised when the serial link drops (USB disconnect / re-enumeration)."""


# ---------------------------- the Layer 4 client ----------------------------

class RcLayer4:
    """
    High-level Layer 4 client: one-call `send()` that maps your friendly command names
    to the Layer 3 wire protocol, waits for OK/DONE, validates args, and raises on errors.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = _DEFAULT_BAUD,
        telemetry_cb: Optional[Callable[[Dict[str, Any]], None]] = None
    ):
        """
        :param port: Serial port (e.g., 'COM7' or '/dev/ttyACM0'). If None, auto-detect.
        :param baud: Baud rate (Layer 3 uses 115200 by default).
        :param telemetry_cb: Optional callback receiving parsed TELEM dicts.
        """
        self._cfg_port = port
        self._baud = baud
        self._ser: Optional[serial.Serial] = None
        self._telemetry_cb = telemetry_cb
        self._connected: bool = False
        self._port_path: Optional[str] = None

    # ---------- lifecycle ----------

    def open(self):
        """Open the serial port (auto-detect if needed) and flush input."""
        if self._ser and self._ser.is_open:
            return
        port = self._cfg_port or _find_teensy_port()
        if not port:
            raise L4Error("Could not auto-detect Teensy port. Pass port= to RcLayer4().")
        try:
            self._ser = serial.Serial(port=port, baudrate=self._baud, timeout=_DEFAULT_READ_TIMEOUT)
        except Exception as e:
            self._connected = False
            raise
        # give MCU a moment in case it resets-on-open
        time.sleep(0.5)
        self._flush_input()
        self._connected = True
        self._port_path = port

    def close(self):
        """Close the serial port."""
        try:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
        finally:
            self._ser = None
            self._connected = False

    # ---------- connection helpers ----------

    def is_connected(self) -> bool:
        return bool(self._connected and self._ser and self._ser.is_open)

    def _safe_disconnect(self):
        try:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
        finally:
            self._ser = None
            self._connected = False

    def _reopen(self, prefer_last: bool = True) -> Optional[str]:
        """Attempt to reopen the serial port. Returns the port path on success; None on failure."""
        candidates: List[str] = []
        if prefer_last and self._port_path:
            candidates.append(self._port_path)
        # Prefer Teensy-identified port
        p = _find_teensy_port()
        if p and p not in candidates:
            candidates.append(p)
        # Also include any device paths that look serial-ish (best effort)
        try:
            from serial.tools import list_ports
            for c in list_ports.comports():
                desc = f"{c.device} {c.description or ''} {c.manufacturer or ''}"
                if "teensy" in desc.lower() and c.device not in candidates:
                    candidates.append(c.device)
        except Exception:
            pass

        for devpath in candidates:
            try:
                self._ser = serial.Serial(port=devpath, baudrate=self._baud, timeout=_DEFAULT_READ_TIMEOUT)
                time.sleep(0.3)
                self._flush_input()
                self._connected = True
                self._port_path = devpath
                # Best-effort resync; ignore any error
                try:
                    self.send("stop")
                    self.send("center")
                except Exception:
                    pass
                return devpath
            except Exception:
                self._safe_disconnect()
                continue
        return None

    # ---------- public: single-call API ----------

    def send(self, cmd: str, *args, timeout: Optional[float] = None, end: Optional[str] = None) -> Dict[str, Any]:
        """
        Send a high-level command and wait for OK / DONE appropriately.

        :param cmd: Friendly command name, e.g. 'arcright', 'driveforward', 'center', 'stop', 'steerleft', etc.
        :param args: Positional numeric args (PWM, duration_ms) for drive/arc. Others take no args.
        :param timeout: DONE timeout (seconds). If None, auto: max(duration_ms + 2.5s, 6s).
        :param end: For ARC only: 'U' to unlock (default) or 'H' to hold steering lock after.
        :return: dict with keys: {'ok': 'TAG', 'done': 'TAG' or None, 'elapsed_s': float}
        :raises L4Error: on ERR reply or timeouts.
        """
        line, expect_done, duration_ms = self._encode(cmd, *args, end=end)
        t0 = time.time()
        self._write_line(line)
        self._wait_ok_or_err(ack_timeout=_DEFAULT_ACK_TIMEOUT)

        done_tag = None
        if expect_done:
            # Determine DONE timeout
            if timeout is None:
                # convert ms & add fudge
                candidate = (duration_ms + _DONE_FUDGE_MS) / 1000.0
                timeout = candidate if candidate > _MIN_DONE_TIMEOUT else _MIN_DONE_TIMEOUT
            done_tag = self._wait_done(expect_tag=expect_done, timeout_s=timeout)

        return {"ok": expect_done or "IMMEDIATE", "done": done_tag, "elapsed_s": time.time() - t0}

    # Convenience helpers so Layer 5 can call either `.send("arcright"... )` or `.arcright(...)`
    def stop(self):             return self.send("stop")
    def brake(self):            return self.send("brake")
    def center(self):           return self.send("center")
    def steerleft(self):        return self.send("steerleft")
    def steerright(self):       return self.send("steerright")
    def steerunlock(self):      return self.send("steerunlock")
    def driveforward(self, pwm:int, dur_ms:int, timeout:Optional[float]=None):
        return self.send("driveforward", pwm, dur_ms, timeout=timeout)
    def driveback(self, pwm:int, dur_ms:int, timeout:Optional[float]=None):
        return self.send("driveback", pwm, dur_ms, timeout=timeout)
    def arcleft(self, pwm:int, dur_ms:int, end:Optional[str]=None, timeout:Optional[float]=None):
        return self.send("arcleft", pwm, dur_ms, timeout=timeout, end=end)
    def arcright(self, pwm:int, dur_ms:int, end:Optional[str]=None, timeout:Optional[float]=None):
        return self.send("arcright", pwm, dur_ms, timeout=timeout, end=end)

    # ---------- encoding / validation ----------

    def _encode(self, cmd: str, *args, end: Optional[str]) -> Tuple[str, Optional[str], int]:
        """
        Map friendly command to Layer 3 wire string.
        Returns (line, expect_done_tag or None, duration_ms_for_timeout)
        """
        if not cmd:
            raise L4Error("Empty command")
        key = self._normalize_key(cmd)

        # STOP / BRAKE / CENTER
        if key in ("stop",):
            return "STOP", None, 0
        if key in ("brake",):
            return "BRAKE", None, 0
        if key in ("center",):
            return "CENTER", None, 0

        # STEER
        if key in ("steerleft", "steerl", "steer_l", "steer-left"):
            return "STEER L", None, 0
        if key in ("steerright", "steerr", "steer_r", "steer-right"):
            return "STEER R", None, 0
        if key in ("steerunlock", "steeru", "steer_u", "steer-unlock", "unlock"):
            return "STEER U", None, 0

        # DRIVE
        if key in ("driveforward", "drivef", "drive_f", "drive-f", "fwd", "forward"):
            pwm, dur = self._parse_pwm_dur(args)
            return f"DRIVE F {pwm} {dur}", "DRIVE", dur
        if key in ("driveback", "driveb", "drive_b", "drive-b", "back", "reverse", "rev"):
            pwm, dur = self._parse_pwm_dur(args)
            return f"DRIVE B {pwm} {dur}", "DRIVE", dur

        # ARC
        if key in ("arcleft", "arc_left", "arc-left", "arcl"):
            pwm, dur = self._parse_pwm_dur(args)
            e = self._normalize_end(end)
            return f"ARC L {pwm} {dur} {e}", "ARC", dur
        if key in ("arcright", "arc_right", "arc-right", "arcr"):
            pwm, dur = self._parse_pwm_dur(args)
            e = self._normalize_end(end)
            return f"ARC R {pwm} {dur} {e}", "ARC", dur

        raise L4Error(f"Unknown command '{cmd}'")

    @staticmethod
    def _normalize_key(cmd: str) -> str:
        return cmd.strip().lower().replace(" ", "").replace("_", "").replace("-", "")

    @staticmethod
    def _parse_pwm_dur(args: Tuple[Any, ...]) -> Tuple[int, int]:
        if len(args) < 2:
            raise L4Error("Command requires (pwm, duration_ms)")
        try:
            pwm = int(args[0])
            dur = int(args[1])
        except Exception:
            raise L4Error("pwm and duration_ms must be integers")
        if not (0 <= pwm <= 255):
            raise L4Error("pwm out of range (0..255)")
        if not (0 <= dur <= 65535):
            raise L4Error("duration_ms out of range (0..65535)")
        return pwm, dur

    @staticmethod
    def _normalize_end(end: Optional[str]) -> str:
        if end is None:
            return "U"  # default = unlock after ARC
        e = str(end).strip().upper()
        if e not in ("U", "H"):
            raise L4Error("end must be 'U' (unlock) or 'H' (hold)")
        return e

    # ---------- I/O & protocol waits ----------

    def _write_line(self, line: str):
        if not self._ser or not self._ser.is_open:
            raise L4Error("Serial port is not open")
        if not line.endswith("\n"):
            line += "\n"
        try:
            self._ser.write(line.encode("ascii", errors="ignore"))
            self._ser.flush()
        except (serial.SerialException, OSError) as e:
            # Mark disconnected and raise specific error for L5 to handle
            self._safe_disconnect()
            raise L4Disconnected(str(e))

    def _read_line(self) -> Optional[str]:
        """Read one line; returns None on timeout. Also dispatches TELEM to callback if set."""
        if not self._ser:
            return None
        try:
            raw = self._ser.readline()
        except (serial.SerialException, OSError) as e:
            self._safe_disconnect()
            raise L4Disconnected(str(e))
        if not raw:
            return None
        try:
            s = raw.decode("utf-8", errors="ignore").strip("\r\n")
        except Exception:
            return None

        if s.startswith("TELEM"):
            if self._telemetry_cb:
                parsed = self._parse_telem(s)
                if parsed is not None:
                    try:
                        self._telemetry_cb(parsed)
                    except Exception:
                        pass
            # Either way, TELEM is not returned to command waits
            return None
        return s

    def _wait_ok_or_err(self, ack_timeout: float):
        t0 = time.time()
        while True:
            ln = self._read_line()
            if ln is None:
                if time.time() - t0 > ack_timeout:
                    raise L4Error("Timeout waiting for OK/ERR")
                continue
            if ln.startswith("ERR"):
                raise L4Error(f"Device error: {ln}")
            if ln.startswith("OK "):
                return  # success; proceed to DONE if needed
            # any other line types are ignored here

    def _wait_done(self, expect_tag: str, timeout_s: float) -> str:
        t0 = time.time()
        while True:
            ln = self._read_line()
            if ln is None:
                if time.time() - t0 > timeout_s:
                    raise L4Error(f"Timeout waiting for DONE {expect_tag}")
                continue
            if ln.startswith("ERR"):
                raise L4Error(f"Device error: {ln}")
            if ln.strip() == f"DONE {expect_tag}":
                return expect_tag
            # ignore other lines

    def _flush_input(self):
        """Clear any buffered data and briefly drain boot-time TELEM."""
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        # Drain a little (some boards spam TELEM on boot)
        t0 = time.time()
        while time.time() - t0 < 0.2:
            _ = self._read_line()

    # ---------- TELEM parsing ----------

    @staticmethod
    def _parse_telem(line: str) -> Optional[Dict[str, Any]]:
        """
        Parse: TELEM drive=0 steer=1 L=123 R=456 t=789012
        Returns dict or None if malformed.
        """
        try:
            parts = line.split()
            out: Dict[str, Any] = {}
            for p in parts[1:]:
                if '=' not in p:
                    continue
                k, v = p.split('=', 1)
                # ints please
                out[k] = int(v)
            return out
        except Exception:
            return None


# ---------------------------- optional self-test ----------------------------
if __name__ == "__main__":
    """
    Quick self-test. Edit 'PORT' to force a specific port, or leave None to auto-detect.
    """
    PORT = None  # e.g., "COM7" on Windows or "/dev/ttyACM0" on Linux/Mac

    def on_telem(d: Dict[str, Any]):
        # print small, non-spammy telemetry (comment out to silence)
        print(f"TELEM drive={d.get('drive')} steer={d.get('steer')} L={d.get('L')} R={d.get('R')} t={d.get('t')}")

    l4 = RcLayer4(port=PORT, telemetry_cb=on_telem)
    try:
        l4.open()
        # Example sequence; comment/uncomment as needed:
        l4.send("steerleft")
        l4.send("driveforward", 180, 2000)     # waits for DONE DRIVE
        l4.send("center")
        l4.send("arcright", 200, 1500)         # waits for DONE ARC (auto unlock)
        l4.send("stop")
        print("Self-test complete.")
    except L4Error as e:
        print(f"[L4 ERROR] {e}")
    finally:
        l4.close()
