#!/usr/bin/env python3
"""Guarded OT-One Z calibration and tip-pickup console.

This is for the original USB-connected Opentrons OT-One/Smoothieboard, not an
OT-2 or Flex. It deliberately never commands X, Y, A, B, or C. In particular,
Y is left untouched because the connected robot's Y home has already stalled.

The press cycle is preserved from the archived OT-One API implementation:
https://github.com/tvr/opentrons-api/blob/master/opentrons/instruments/pipette.py
"""

from __future__ import annotations

import argparse
import json
import re
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import serial
from serial.tools import list_ports


DEFAULT_BAUD = 115200
DEFAULT_MAX_Z = 95.0  # Archived OT-One Z envelope is 100 mm; retain 5 mm reserve.
MAX_SINGLE_JOG = 5.0
CALIBRATION_PATH = Path(__file__).with_name("ot_one_tip_calibration.json")


class ControllerError(RuntimeError):
    pass


@dataclass(frozen=True)
class ZPosition:
    target: float
    current: float


def parse_position(text: str) -> ZPosition:
    """Parse target Z and real-time z without collapsing their case."""
    upper = re.search(r"(?:^|\s)Z:\s*(-?\d+(?:\.\d+)?)", text)
    lower = re.search(r"(?:^|\s)z:\s*(-?\d+(?:\.\d+)?)", text)
    if not upper or not lower:
        raise ControllerError(f"could not parse Z position from: {text!r}")
    return ZPosition(float(upper.group(1)), float(lower.group(1)))


def parse_config_number(text: str, key: str) -> Optional[float]:
    match = re.search(
        rf"(?:^|\s){re.escape(key)}\s*:?\s*(-?\d+(?:\.\d+)?)", text
    )
    return float(match.group(1)) if match else None


def pickup_targets(seated_z: float) -> tuple[float, float, float, float]:
    """Translate the legacy API's +6/-7/+7/-6 sequence to board Z."""
    # The old API flips its logical Z before emitting Smoothie G-code. In board
    # coordinates, increasing Z goes physically down from the top-home datum.
    return (seated_z - 6.0, seated_z + 1.0, seated_z - 6.0, seated_z)


def discover_port() -> str:
    matches = []
    for item in list_ports.comports():
        if (
            item.vid == 0x1D50
            and item.pid == 0x6015
            and "Smoothieboard" in (item.description or "")
        ):
            matches.append(item.device)
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise ControllerError("no OT-One Smoothieboard USB serial port found")
    raise ControllerError("multiple Smoothieboards found; pass --port explicitly")


class SmoothieZ:
    def __init__(self, port: str, baud: int, max_z: float) -> None:
        self.port = port
        self.baud = baud
        self.max_z = max_z
        self.serial: Optional[serial.Serial] = None
        self.homed = False
        self.version = "unknown"

    def open(self) -> None:
        try:
            self.serial = serial.Serial(
                self.port,
                self.baud,
                timeout=0.15,
                write_timeout=0.5,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        except Exception as exc:
            raise ControllerError(f"cannot open {self.port}: {exc}") from exc

        # Opening this generation of Smoothieboard may reset it.
        time.sleep(2.5)
        self._drain(0.4)
        response = self.query("version", timeout=2.5, require_ok=False)
        if not response.strip():
            raise ControllerError(
                "controller is enumerated but silent; unplug USB and main power, "
                "wait 10 seconds, reconnect USB, then switch main power on"
            )
        self.version = response.strip()
        self.query("G90")

    def close(self) -> None:
        if self.serial is not None:
            try:
                self.serial.close()
            finally:
                self.serial = None

    def _write(self, payload: bytes) -> None:
        if self.serial is None:
            raise ControllerError("serial port is not open")
        try:
            self.serial.write(payload)
        except Exception as exc:
            raise ControllerError(f"USB write failed: {exc}") from exc

    def _drain(self, duration: float) -> bytes:
        if self.serial is None:
            return b""
        deadline = time.monotonic() + duration
        parts = []
        while time.monotonic() < deadline:
            chunk = self.serial.read(4096)
            if chunk:
                parts.append(chunk)
        return b"".join(parts)

    def query(self, command: str, timeout: float = 2.0, require_ok: bool = True) -> str:
        if self.serial is None:
            raise ControllerError("serial port is not open")
        self.serial.reset_input_buffer()
        self._write((command + "\r\n").encode())

        deadline = time.monotonic() + timeout
        last_data = None
        parts = []
        while time.monotonic() < deadline:
            chunk = self.serial.read(4096)
            if chunk:
                parts.append(chunk)
                last_data = time.monotonic()
                low = b"".join(parts).lower()
                if b"error" in low or b"alarm" in low or b"halt" in low:
                    raise ControllerError(
                        f"controller rejected {command!r}: "
                        f"{b''.join(parts).decode(errors='replace').strip()}"
                    )
            elif parts and last_data is not None and time.monotonic() - last_data >= 0.20:
                break

        text = b"".join(parts).decode(errors="replace")
        if require_ok and "ok" not in text.lower():
            raise ControllerError(f"no acknowledgement for {command!r}: {text!r}")
        return text

    def emergency_stop(self) -> bool:
        self.homed = False
        if self.serial is None:
            return False
        wrote = False
        for payload in (b"\x18", b"M112\r\n", b"M18\r\n"):
            try:
                self.serial.write(payload)
                wrote = True
                time.sleep(0.15)
            except Exception:
                pass
        return wrote

    def position(self) -> ZPosition:
        return parse_position(self.query("M114.2"))

    def endstops(self) -> str:
        return " ".join(self.query("M119").split())

    def read_z_config(self) -> dict[str, object]:
        result: dict[str, object] = {}
        for key in (
            "gamma_max_travel",
            "gamma_homing_direction",
            "gamma_limit_enable",
            "gamma_min",
            "gamma_max",
        ):
            try:
                raw = self.query(f"config-get sd {key}")
                result[key] = parse_config_number(raw, key) or raw.strip()
            except ControllerError as exc:
                result[key] = f"unavailable ({exc})"
        return result

    def home_z(self) -> ZPosition:
        # Z is the shared vertical head axis on OT-One. This robot has already
        # homed Z successfully. No other axis is named in these commands.
        try:
            self.query("G28.2 Z", timeout=15.0)
            self.query("G92 Z0")
            self.query("G90")
            pos = self.position()
        except BaseException:
            self.emergency_stop()
            raise
        if abs(pos.current) > 0.15:
            self.emergency_stop()
            raise ControllerError(f"Z home did not establish zero: {pos}")
        self.homed = True
        return pos

    def move_z(self, target: float, feedrate: float = 600.0) -> ZPosition:
        if not self.homed:
            raise ControllerError("Z is not homed; run 'home' first")
        if not 0.0 <= target <= self.max_z:
            raise ControllerError(
                f"refusing Z={target:.2f}; guarded range is 0.00..{self.max_z:.2f} mm"
            )

        start = self.position().current
        distance = abs(target - start)
        budget = min(15.0, max(3.0, distance / max(feedrate / 60.0, 0.1) + 2.0))
        try:
            self.query(f"G0 Z{target:.3f} F{feedrate:.0f}", timeout=2.5)
            deadline = time.monotonic() + budget
            latest = self.position()
            while time.monotonic() < deadline:
                latest = self.position()
                if abs(latest.current - target) <= 0.10:
                    return latest
                time.sleep(0.08)
        except BaseException:
            self.emergency_stop()
            raise
        self.emergency_stop()
        raise ControllerError(
            f"Z failed to arrive at {target:.2f} within {budget:.1f}s; emergency stop sent"
        )

    def jog(self, physical_down_mm: float) -> ZPosition:
        if abs(physical_down_mm) > MAX_SINGLE_JOG:
            raise ControllerError(
                f"one jog is limited to {MAX_SINGLE_JOG:.1f} mm; use repeated small jogs"
            )
        here = self.position().current
        return self.move_z(here + physical_down_mm)

    def pickup_press_cycle(self) -> ZPosition:
        if not self.homed:
            raise ControllerError("Z is not homed; run 'home' first")
        seated = self.position().current
        targets = pickup_targets(seated)
        if min(targets) < 0 or max(targets) > self.max_z:
            raise ControllerError(
                "not enough guarded Z clearance for the legacy pickup press cycle"
            )
        latest = self.position()
        for target in targets:
            latest = self.move_z(target, feedrate=420.0)
        return latest


HELP = """Commands:
  status              read Z position and endstops; no motion
  home                home shared Z only and establish top as Z=0
  down [0.1..5]       lower the head by mm (default 1)
  up [0.1..5]         raise the head by mm (default 1)
  pickup              run the original OT-One two-press pickup cycle here
  lift [0.1..5]       alias for up (default 5) to check that the tip attached
  save                save the current Z as the calibrated pickup position
  estop               controller reset + halt + disable steppers
  help                show these commands
  quit                close without moving
"""


def save_calibration(robot: SmoothieZ) -> None:
    pos = robot.position()
    payload = {
        "device": "OT-One / Smoothieboard",
        "port": robot.port,
        "firmware": robot.version,
        "tip_pickup_z_mm": pos.current,
        "z_home_mm": 0.0,
        "guarded_max_z_mm": robot.max_z,
        "saved_at": datetime.now(timezone.utc).isoformat(),
        "note": "X/Y were intentionally not moved or calibrated; Y home is faulty.",
    }
    CALIBRATION_PATH.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"saved {CALIBRATION_PATH}")


def run_console(robot: SmoothieZ) -> int:
    print(f"connected: {robot.port} @ {robot.baud}")
    print(f"firmware:  {robot.version}")
    print(f"guarded Z: 0.00..{robot.max_z:.2f} mm (down increases Z)")
    print(f"endstops:  {robot.endstops()}")
    print(f"Z config:  {json.dumps(robot.read_z_config(), sort_keys=True)}")
    print("\nNo axis has moved. Keep hands clear, then type 'home'.")
    print(HELP)

    while True:
        try:
            raw = input("ot-one> ").strip().lower()
            if not raw:
                continue
            bits = raw.split()
            command = bits[0]

            if command in {"quit", "exit"}:
                return 0
            if command == "help":
                print(HELP)
                continue
            if command == "status":
                print(f"Z={robot.position()} | {robot.endstops()}")
                continue
            if command == "estop":
                print("emergency stop written" if robot.emergency_stop() else "NOTHING WRITTEN")
                continue
            if command == "home":
                print(f"homed Z only: {robot.home_z()}")
                continue
            if command in {"down", "up", "lift"}:
                default = 5.0 if command == "lift" else 1.0
                amount = float(bits[1]) if len(bits) > 1 else default
                if not 0.1 <= amount <= MAX_SINGLE_JOG:
                    raise ControllerError("jog amount must be between 0.1 and 5.0 mm")
                down = amount if command == "down" else -amount
                print(robot.jog(down))
                continue
            if command == "pickup":
                print(f"pickup press cycle complete: {robot.pickup_press_cycle()}")
                continue
            if command == "save":
                save_calibration(robot)
                continue
            print("unknown command; type 'help'")
        except (ControllerError, ValueError) as exc:
            print(f"REFUSED: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--max-z", type=float, default=DEFAULT_MAX_Z)
    args = parser.parse_args()

    if not 5.0 <= args.max_z <= 100.0:
        parser.error("--max-z must be between 5 and 100 mm")

    try:
        port = args.port or discover_port()
        robot = SmoothieZ(port, args.baud, args.max_z)
        robot.open()
    except ControllerError as exc:
        print(f"CONNECT FAILED: {exc}", file=sys.stderr)
        return 1

    def panic(signum, _frame):
        wrote = robot.emergency_stop()
        print(
            f"\nsignal {signum}: "
            + ("emergency stop written" if wrote else "CUT MAIN POWER NOW"),
            file=sys.stderr,
        )
        raise SystemExit(130)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, panic)

    try:
        return run_console(robot)
    finally:
        robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
