#!/usr/bin/env python3
"""ot_driver: a small, safe control driver for Opentrons hardware.

One uniform API over three transports, so the same demo code runs against
whatever you actually have in front of you:

  serial : GCode over USB serial. The "old school" path (OT-One / Smoothieboard).
           Also the fallback if a robot's onboard robot-server is dead.
  http   : robot-server HTTP API on port 31950 (OT-2 and Flex).
  sim    : no hardware. Tracks position in memory so the control logic,
           the soft limits, and your demo script can all be proven offline.

Nothing moves on import, and nothing moves unless you construct with
allow_motion=True. Every commanded move is bounds-checked first.

CLI:
    python3 ot_driver.py detect
    python3 ot_driver.py demo --transport sim
    python3 ot_driver.py demo --transport serial --port /dev/cu.usbmodem1234 --go
    python3 ot_driver.py demo --transport http   --host 192.168.1.50 --go

VERIFY BEFORE HARDWARE USE: the GCode words and the soft limits below are the
documented defaults for their platforms, but they have NOT been checked against
your specific machine or firmware. Confirm both, on your robot, before --go.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

ROBOT_SERVER_PORT = 31950

# --- Smoothieware GCode used by the legacy Opentrons smoothie driver. --------
# TODO(verify on hardware): confirm against your firmware before trusting.
GCODE_HOME = "G28.2"      # Smoothie homing (NOT plain G28)
GCODE_MOVE = "G0"
GCODE_ABSOLUTE = "G90"
GCODE_POSITION = "M114.2"
GCODE_ESTOP = "M112"              # emergency stop -> latches HALT state
GCODE_CLEAR_HALT = "M999"         # clear HALT so commands are accepted again
GCODE_DISABLE_STEPPERS = "M18"    # de-energize steppers
GCODE_ENDSTOPS = "M119"           # report endstop states
SMOOTHIE_RESET = b"\x18"          # Ctrl-X: handled at the serial layer, so it
                                  # interrupts an in-flight move. M112 alone can
                                  # sit behind a queued long move.
SERIAL_BAUD = 115200

# Smoothieware names its axes by Greek letter. Opentrons wires them up as below,
# confirmed against this board's own six-axis M114.2 reply.
SMOOTHIE_AXIS_MAP = {
    "X": "alpha", "Y": "beta", "Z": "gamma",
    "A": "delta", "B": "epsilon", "C": "zeta",
}
# Config keys worth reading before trusting any coordinate. max_travel is how far
# homing will drive before giving up, and homing_direction says which end it
# drives toward; together they explain a home that grinds instead of arriving.
CONFIG_KEYS = (
    "max_travel", "homing_direction", "min", "max",
    "fast_homing_rate_mm_s", "slow_homing_rate_mm_s", "limit_enable",
)

# Axes proven to home cleanly on this machine on 2026-07-25: Z, A, X each
# acknowledged G28.2. Y did NOT ack and stalled audibly, so it is excluded by
# default until the cause is found. Pass --axes to override deliberately.
DEFAULT_HOME_AXES = ("Z", "A", "X")
STALLED_AXES = ("Y",)

# --- Soft limits, in mm. Conservative; deliberately smaller than the true ----
# envelope so a bad demo coordinate stops here instead of at a hard stop.
# TODO(verify on hardware): measure your own usable envelope.
SOFT_LIMITS: Dict[str, Dict[str, Tuple[float, float]]] = {
    "ot-one": {"x": (0.0, 350.0), "y": (0.0, 240.0), "z": (0.0, 100.0)},
    "ot-2":   {"x": (0.0, 400.0), "y": (0.0, 340.0), "z": (0.0, 200.0)},
    "sim":    {"x": (0.0, 400.0), "y": (0.0, 340.0), "z": (0.0, 200.0)},
}


class MotionBlocked(RuntimeError):
    """Raised when motion is attempted without allow_motion=True."""


class OutOfBounds(ValueError):
    """Raised when a commanded coordinate falls outside the soft limits."""


class TransportError(RuntimeError):
    """Raised when the underlying link fails."""


@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def __str__(self) -> str:
        return f"(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"


# ----------------------------------------------------------------------------
# Transports
# ----------------------------------------------------------------------------
class Transport:
    """Interface every transport implements."""

    model = "sim"

    def open(self) -> None: ...
    def close(self) -> None: ...
    def describe(self) -> str: ...
    def home(self, axes: Optional[Tuple[str, ...]] = None) -> None: ...
    def move_to(self, x: float, y: float, z: float, feedrate: float,
                axes: Optional[Tuple[str, ...]] = None) -> None: ...
    def position(self) -> Position: ...

    def estop(self) -> None:
        """Cut motion. No-op where the transport has no notion of it."""
        return None

    def endstops(self) -> Dict[str, int]:
        raise TransportError("endstop read is not supported by this transport")

    def axis_config(self) -> Dict[str, Dict[str, str]]:
        raise TransportError("config read is not supported by this transport")

    def learn_limits(self) -> Dict[str, Tuple[float, float]]:
        raise TransportError("limit learning is not supported by this transport")


@dataclass
class SimTransport(Transport):
    """No hardware. Records every command so tests can assert on them."""

    model: str = "sim"
    pos: Position = field(default_factory=Position)
    log: List[str] = field(default_factory=list)
    opened: bool = False

    def open(self) -> None:
        self.opened = True
        self.log.append("open")

    def close(self) -> None:
        self.opened = False
        self.log.append("close")

    def describe(self) -> str:
        return "sim (no hardware; in-memory position)"

    def home(self, axes: Optional[Tuple[str, ...]] = None) -> None:
        self.pos = Position(0.0, 0.0, 0.0)
        self.log.append(f"home {','.join(axes) if axes else 'default'}")

    def estop(self) -> None:
        self.log.append("estop")

    def endstops(self) -> Dict[str, int]:
        return {"min_x": 0, "min_y": 0, "min_z": 0, "min_a": 0, "min_b": 0}

    def move_to(self, x: float, y: float, z: float, feedrate: float,
                axes: Optional[Tuple[str, ...]] = None) -> None:
        allowed = axes if axes is not None else ("X", "Y", "Z")
        self.pos = Position(
            x if "X" in allowed else self.pos.x,
            y if "Y" in allowed else self.pos.y,
            z if "Z" in allowed else self.pos.z,
        )
        self.log.append(
            f"move {'/'.join(allowed)} -> {self.pos.x:.2f},{self.pos.y:.2f},"
            f"{self.pos.z:.2f} f={feedrate:.0f}"
        )

    def position(self) -> Position:
        return Position(self.pos.x, self.pos.y, self.pos.z)


@dataclass
class SerialGCodeTransport(Transport):
    """GCode over USB serial. The old-school OT-One / Smoothieboard path."""

    port: str = ""
    baud: int = SERIAL_BAUD
    model: str = "ot-one"
    # Non-motion commands ack almost instantly. Motion gets its own, still
    # BOUNDED, budget. A generous single timeout is what let a stalling axis
    # grind for 20s on 2026-07-25.
    ack_timeout: float = 2.5
    motion_timeout: float = 12.0
    # Short blocking read so the wait loop can poll and enforce its own
    # deadline. With pyserial's timeout set to the full budget, a single
    # readline() blocks for the whole budget and the deadline never gets to run.
    read_poll: float = 0.25
    _ser: Optional[object] = None

    def open(self) -> None:
        try:
            import serial  # pyserial
        except ImportError as e:
            raise TransportError("pyserial is required: pip install pyserial") from e
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=self.read_poll)
        except Exception as e:
            raise TransportError(f"cannot open {self.port}: {e}") from e
        # Smoothieboard resets on DTR; give the firmware time to come up.
        time.sleep(2.0)
        self._ser.reset_input_buffer()

        if not self._responsive():
            # A previous M112 latches the board in HALT, where it ignores
            # everything until the halt is cleared. Try that before giving up.
            self._raw(GCODE_CLEAR_HALT)
            time.sleep(0.6)
            self._ser.reset_input_buffer()
            if not self._responsive():
                raise TransportError(
                    "board is enumerated but not answering, even after M999. "
                    "Power-cycle the robot before attempting motion."
                )
        self._send(GCODE_ABSOLUTE)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def describe(self) -> str:
        return f"serial GCode on {self.port} @ {self.baud} (model={self.model})"

    # -- low level ----------------------------------------------------------
    def _raw(self, cmd: str) -> None:
        """Write a command without waiting for an acknowledgement."""
        if self._ser is None:
            raise TransportError("serial port is not open")
        self._ser.write((cmd + "\r\n").encode())
        self._ser.flush()

    def _responsive(self) -> bool:
        """True if the board answers a harmless query. Commands no motion."""
        try:
            self._raw("version")
        except TransportError:
            return False
        deadline = time.time() + self.ack_timeout
        while time.time() < deadline:
            if self._ser.read(256):
                return True
        return False

    def estop(self) -> None:
        """Cut motion immediately. Best effort, never raises.

        Ctrl-X goes first because it is handled at the serial layer and so
        interrupts a move already executing; M112 can otherwise sit in the
        queue behind that very move. Closing the port does neither, which is
        why the original stall kept grinding after the script exited.
        """
        if self._ser is None:
            return
        for payload in (SMOOTHIE_RESET, (GCODE_ESTOP + "\r\n").encode(),
                        (GCODE_DISABLE_STEPPERS + "\r\n").encode()):
            try:
                self._ser.write(payload)
                self._ser.flush()
                time.sleep(0.25)
            except Exception:
                pass

    def _send(self, cmd: str, motion: bool = False,
              timeout: Optional[float] = None) -> str:
        if self._ser is None:
            raise TransportError("serial port is not open")
        budget = timeout if timeout is not None else (
            self.motion_timeout if motion else self.ack_timeout)
        self._raw(cmd)
        deadline = time.time() + budget
        buf = ""
        while time.time() < deadline:
            chunk = self._ser.readline().decode(errors="replace")
            if not chunk:
                continue
            buf += chunk
            low = chunk.lower()
            if "error" in low or "alarm" in low or "halt" in low:
                if motion:
                    self.estop()
                raise TransportError(f"firmware rejected {cmd!r}: {chunk.strip()}")
            if "ok" in low:
                return buf
        # No ack inside the budget. For a motion command that means the axis is
        # very likely driving against a hard stop, so cut power to it BEFORE
        # raising rather than unwinding the stack with the motor still turning.
        if motion:
            self.estop()
            raise TransportError(
                f"no ack for {cmd!r} within {budget:.1f}s -> emergency stop sent. "
                f"The axis did not reach its endstop (blocked, wrong homing "
                f"direction, or a dead endstop switch)."
            )
        raise TransportError(f"timed out waiting for ack to {cmd!r}")

    # -- queries ------------------------------------------------------------
    def endstops(self) -> Dict[str, int]:
        """Read endstop states. Commands no motion."""
        raw = self._send(GCODE_ENDSTOPS)
        states: Dict[str, int] = {}
        for tok in raw.replace(",", " ").split():
            if ":" in tok and tok.lower().startswith("min"):
                k, _, v = tok.partition(":")
                try:
                    states[k.strip().lower()] = int(v)
                except ValueError:
                    pass
        return states

    # -- configuration, read from the board itself -------------------------
    def config_get(self, key: str) -> str:
        """Read one Smoothieware config value. Commands no motion."""
        raw = self._send(f"config-get sd {key}")
        # Replies look like "alpha_max_travel: 400.0000" followed by "ok".
        text = " ".join(raw.split())
        for marker in (key + ":", key):
            if marker in text:
                text = text.split(marker, 1)[1]
                break
        return text.replace("ok", "").strip()

    def axis_config(self) -> Dict[str, Dict[str, str]]:
        """Dump the configured travel and homing direction for every axis."""
        out: Dict[str, Dict[str, str]] = {}
        for ot_axis, smoothie in SMOOTHIE_AXIS_MAP.items():
            vals: Dict[str, str] = {}
            for key in CONFIG_KEYS:
                try:
                    vals[key] = self.config_get(f"{smoothie}_{key}")
                except TransportError:
                    vals[key] = ""
            out[ot_axis] = vals
        return out

    def learn_limits(self) -> Dict[str, Tuple[float, float]]:
        """Derive real soft limits from the board's own config.

        This exists because the hard-coded SOFT_LIMITS in this file are platform
        defaults, not measurements, and commanding a coordinate past the real
        envelope is what overreached on this machine. Prefer these numbers.
        """
        cfg = self.axis_config()
        limits: Dict[str, Tuple[float, float]] = {}
        for axis in ("X", "Y", "Z"):
            vals = cfg.get(axis, {})
            lo_s, hi_s, travel_s = vals.get("min", ""), vals.get("max", ""), vals.get("max_travel", "")
            lo = hi = None
            try:
                lo = float(lo_s)
            except (TypeError, ValueError):
                pass
            try:
                hi = float(hi_s)
            except (TypeError, ValueError):
                pass
            if hi is None:
                # Some configs express the envelope only as max_travel.
                try:
                    hi = float(travel_s)
                except (TypeError, ValueError):
                    hi = None
            if lo is None:
                lo = 0.0
            if hi is not None and hi > lo:
                limits[axis.lower()] = (lo, hi)
        if not limits:
            raise TransportError(
                "the board reported no usable min/max/max_travel values, so the "
                "envelope cannot be learned. Do not guess it."
            )
        return limits

    def home(self, axes: Optional[Tuple[str, ...]] = None) -> None:
        # Opentrons axis map, confirmed from this board's own M114.2 reply:
        # X, Y = gantry; Z = left mount; A = right mount; B, C = plungers.
        # Home the MOUNT axes first so both pipettes lift clear of the deck
        # before the gantry sweeps in X/Y. That is the order the Opentrons
        # software uses; reversing it drags a low pipette across the deck.
        for axis in (axes if axes is not None else DEFAULT_HOME_AXES):
            self._send(f"{GCODE_HOME} {axis}", motion=True)

    def move_to(self, x: float, y: float, z: float, feedrate: float,
                axes: Optional[Tuple[str, ...]] = None) -> None:
        # Emit ONLY the requested axes. Naming an axis that has not been homed
        # would move it from an unknown position toward an absolute coordinate,
        # which is how an unhomed axis gets driven into a hard stop.
        allowed = axes if axes is not None else ("X", "Y", "Z")
        vals = {"X": x, "Y": y, "Z": z}
        words = " ".join(f"{a}{vals[a]:.2f}" for a in ("X", "Y", "Z") if a in allowed)
        if not words:
            raise TransportError("move_to called with no movable axes")
        self._send(f"{GCODE_MOVE} {words} F{feedrate:.0f}", motion=True)

    def position(self) -> Position:
        # This board replies e.g.:
        #   ok C: X:0.000 Y:0.000 Z:0.000 x:0.000 y:0.000 z:0.000
        #   C: A:0.000 B:0.000 C:0.000 a:0.000 b:0.000 c:0.000
        # Uppercase is the commanded/current value and lowercase is the realtime
        # readout, so prefer uppercase and only fall back to lowercase. Matching
        # case-insensitively would let the second pair silently overwrite the
        # first once the two diverge mid-move.
        raw = self._send(GCODE_POSITION)
        upper: Dict[str, float] = {}
        lower: Dict[str, float] = {}
        for tok in raw.replace(",", " ").split():
            if ":" not in tok:
                continue
            k, _, v = tok.partition(":")
            k = k.strip()
            try:
                val = float(v)
            except ValueError:
                continue
            if k in ("X", "Y", "Z"):
                upper.setdefault(k, val)
            elif k in ("x", "y", "z"):
                lower.setdefault(k, val)
        pick = lambda u, l: upper.get(u, lower.get(l, 0.0))
        return Position(pick("X", "x"), pick("Y", "y"), pick("Z", "z"))


@dataclass
class HTTPTransport(Transport):
    """robot-server HTTP API (OT-2 and Flex)."""

    host: str = ""
    port: int = ROBOT_SERVER_PORT
    mount: str = "right"
    model: str = "ot-2"
    timeout: float = 120.0

    @property
    def _headers(self) -> Dict[str, str]:
        # The robot-server rejects requests that omit this header.
        return {"Opentrons-Version": "*", "Content-Type": "application/json"}

    def _call(self, path: str, method: str = "GET", body=None):
        url = f"http://{self.host}:{self.port}{path}"
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(url, data=data, method=method, headers=self._headers)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                raw = r.read().decode()
                try:
                    return r.status, json.loads(raw)
                except json.JSONDecodeError:
                    return r.status, raw
        except urllib.error.HTTPError as e:
            raise TransportError(f"{method} {path} -> HTTP {e.code}: {e.read().decode()[:300]}") from e
        except (urllib.error.URLError, OSError) as e:
            raise TransportError(f"{method} {path} -> unreachable: {e}") from e

    def open(self) -> None:
        status, health = self._call("/health")
        if status != 200:
            raise TransportError(f"/health returned {status}")
        advertised = str(health.get("robot_model", "")).lower()
        # "OT-3 Standard" is the Flex; anything else on this API is an OT-2.
        self.model = "ot-2" if "ot-3" not in advertised else "flex"
        self._health = health

    def close(self) -> None:
        return None

    def describe(self) -> str:
        h = getattr(self, "_health", {})
        return (
            f"http robot-server at {self.host}:{self.port} "
            f"(name={h.get('name')}, model={h.get('robot_model')}, api={h.get('api_version')})"
        )

    def home(self, axes: Optional[Tuple[str, ...]] = None) -> None:
        if self.model == "flex":
            raise TransportError(
                "This is a Flex (OT-3). The /robot/home endpoint is OT-2 only; "
                "the Flex homes through the /runs + /commands API. Tell me and I will wire it."
            )
        # The HTTP API homes the whole robot; it has no per-axis form, so a
        # partial axis list cannot be honoured here.
        if axes is not None:
            raise TransportError(
                "per-axis homing is serial-only; the HTTP API homes everything at once"
            )
        self._call("/robot/home", "POST", {"target": "robot"})

    def move_to(self, x: float, y: float, z: float, feedrate: float,
                axes: Optional[Tuple[str, ...]] = None) -> None:
        if self.model == "flex":
            raise TransportError("Flex move goes through /runs + /commands, not /robot/move.")
        if axes is not None and set(axes) != {"X", "Y", "Z"}:
            raise TransportError(
                "the HTTP API takes a whole XYZ point; per-axis moves are serial-only"
            )
        self._call(
            "/robot/move", "POST",
            {"target": "pipette", "mount": self.mount, "point": [x, y, z]},
        )

    def position(self) -> Position:
        # The OT-2 HTTP API exposes no clean absolute-position read, so the
        # driver's cached target is authoritative here. Honest limitation.
        raise TransportError("position() is not available over the HTTP transport")


# ----------------------------------------------------------------------------
# Driver
# ----------------------------------------------------------------------------
class OTDriver:
    """Uniform, bounds-checked control surface over any transport."""

    def __init__(self, transport: Transport, allow_motion: bool = False,
                 default_feedrate: float = 4000.0):
        self.t = transport
        self.allow_motion = allow_motion
        self.default_feedrate = default_feedrate
        self._last = Position()
        self._homed = False
        self._homed_axes: Tuple[str, ...] = ()
        self._learned_limits: Optional[Dict[str, Tuple[float, float]]] = None
        self._open = False

    # -- lifecycle ----------------------------------------------------------
    def __enter__(self) -> "OTDriver":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.disconnect()

    def connect(self) -> "OTDriver":
        self.t.open()
        self._open = True
        return self

    def disconnect(self) -> None:
        if self._open:
            self.t.close()
            self._open = False

    def describe(self) -> str:
        return self.t.describe()

    # -- safety -------------------------------------------------------------
    @property
    def limits(self) -> Dict[str, Tuple[float, float]]:
        """Limits learned from the board win over the hard-coded guesses."""
        base = dict(SOFT_LIMITS.get(self.t.model, SOFT_LIMITS["sim"]))
        if self._learned_limits:
            base.update(self._learned_limits)
        return base

    @property
    def limits_are_learned(self) -> bool:
        return bool(self._learned_limits)

    def learn_limits(self) -> Dict[str, Tuple[float, float]]:
        """Replace the guessed envelope with the board's own configuration."""
        self._learned_limits = self.t.learn_limits()
        return self._learned_limits

    def _guard_motion(self) -> None:
        if not self._open:
            raise TransportError("not connected; call connect() first")
        if not self.allow_motion:
            raise MotionBlocked(
                "motion is disabled. Construct OTDriver(..., allow_motion=True) "
                "and clear the deck first."
            )

    def check_bounds(self, x: float, y: float, z: float) -> None:
        for axis, val in (("x", x), ("y", y), ("z", z)):
            lo, hi = self.limits[axis]
            if not (lo <= val <= hi):
                raise OutOfBounds(
                    f"{axis}={val:.2f} outside soft limit [{lo:.1f}, {hi:.1f}] "
                    f"for model {self.t.model}"
                )

    # -- motion -------------------------------------------------------------
    def estop(self) -> None:
        """Cut motion now. Safe to call unconditionally."""
        self.t.estop()

    def endstops(self) -> Dict[str, int]:
        return self.t.endstops()

    def home(self, axes: Optional[Tuple[str, ...]] = None) -> None:
        self._guard_motion()
        self.t.home(axes)
        self._homed = True
        self._homed_axes = tuple(axes) if axes is not None else DEFAULT_HOME_AXES
        # Read back rather than assuming home is the origin. On this hardware
        # the mount axes home to the TOP of their travel, so asserting (0,0,0)
        # would make every subsequent relative move compute from a false base.
        try:
            self._last = self.t.position()
        except TransportError:
            self._last = Position(0.0, 0.0, 0.0)

    @property
    def homed_axes(self) -> Tuple[str, ...]:
        return self._homed_axes

    @property
    def movable_axes(self) -> Tuple[str, ...]:
        """Cartesian axes that have been homed, so are safe to command."""
        return tuple(a for a in ("X", "Y", "Z") if a in self._homed_axes)

    def move_to(self, x: float, y: float, z: float,
                feedrate: Optional[float] = None,
                axes: Optional[Tuple[str, ...]] = None) -> Position:
        self._guard_motion()
        if not self._homed:
            raise TransportError("home() before moving; position is unknown until homed")
        target = axes if axes is not None else self.movable_axes
        unhomed = [a for a in target if a not in self._homed_axes]
        if unhomed:
            raise TransportError(
                f"refusing to move unhomed axis/axes {','.join(unhomed)}: their "
                f"position is unknown, so an absolute move could drive them into "
                f"a hard stop. Homed axes are {','.join(self._homed_axes) or 'none'}."
            )
        if not target:
            raise TransportError("no homed Cartesian axes available to move")
        # Bounds-check only what is actually being commanded.
        vals = {"X": x, "Y": y, "Z": z}
        for axis in target:
            lo, hi = self.limits[axis.lower()]
            v = vals[axis]
            if not (lo <= v <= hi):
                raise OutOfBounds(
                    f"{axis}={v:.2f} outside soft limit [{lo:.1f}, {hi:.1f}] "
                    f"for model {self.t.model}"
                )
        try:
            self.t.move_to(x, y, z, feedrate or self.default_feedrate, target)
        except Exception:
            self.estop()
            raise
        self._last = Position(
            x if "X" in target else self._last.x,
            y if "Y" in target else self._last.y,
            z if "Z" in target else self._last.z,
        )
        return self._last

    def jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
            feedrate: Optional[float] = None) -> Position:
        base = self._last
        return self.move_to(base.x + dx, base.y + dy, base.z + dz, feedrate)

    def position(self) -> Position:
        """Live read where the transport supports it, else the cached target."""
        try:
            return self.t.position()
        except TransportError:
            return self._last


# ----------------------------------------------------------------------------
# Discovery
# ----------------------------------------------------------------------------
# A real USB-serial board enumerates under one of these names. Matching on an
# allowlist (not a denylist) keeps Bluetooth audio profiles and the Mac's own
# debug pseudo-ports from being reported as robot candidates.
USB_SERIAL_HINTS = ("usbmodem", "usbserial", "wchusbserial", "ttyacm", "ttyusb", "slab_usb")


def find_serial_ports() -> List[Tuple[str, str]]:
    """USB serial ports that could plausibly be a robot."""
    try:
        import serial.tools.list_ports as lp
    except ImportError:
        return []
    out = []
    for p in lp.comports():
        if any(h in p.device.lower() for h in USB_SERIAL_HINTS):
            out.append((p.device, p.description or "n/a"))
    return out


def find_other_serial_ports() -> List[Tuple[str, str]]:
    """Every remaining port, so nothing is hidden from the user."""
    try:
        import serial.tools.list_ports as lp
    except ImportError:
        return []
    return [
        (p.device, p.description or "n/a")
        for p in lp.comports()
        if not any(h in p.device.lower() for h in USB_SERIAL_HINTS)
    ]


def _alive(host: str, timeout: float = 1.5) -> bool:
    try:
        with socket.create_connection((host, ROBOT_SERVER_PORT), timeout=timeout):
            return True
    except OSError:
        return False


def find_http_robots() -> List[str]:
    """Scan every /24 this machine is on for a robot-server."""
    import subprocess

    nets: List[str] = []
    out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("inet ") and "127.0.0.1" not in line:
            nets.append(".".join(line.split()[1].split(".")[:3]))
    if "192.168.0" not in nets:
        nets.append("192.168.0")  # subnet the Opentrons app last saw a robot on

    found: List[str] = []
    for net in nets:
        hosts = [f"{net}.{i}" for i in range(1, 255)]
        with ThreadPoolExecutor(max_workers=128) as ex:
            for host, ok in zip(hosts, ex.map(_alive, hosts)):
                if ok:
                    found.append(host)
    return found


def detect() -> dict:
    print("Scanning for Opentrons hardware ...")
    ports = find_serial_ports()
    print(f"\n  USB serial candidates: {len(ports)}")
    for dev, desc in ports:
        print(f"    {dev}  ({desc})")
    if not ports:
        print("    none. An OT-One / Smoothieboard would appear as /dev/cu.usbmodem*")
    other = find_other_serial_ports()
    if other:
        print(f"  (ignored {len(other)} non-USB port(s): "
              f"{', '.join(d.split('/')[-1] for d, _ in other)})")

    robots = find_http_robots()
    print(f"\n  robot-server (HTTP :{ROBOT_SERVER_PORT}) candidates: {len(robots)}")
    for h in robots:
        print(f"    {h}")
    if not robots:
        print("    none on any local subnet.")

    if not ports and not robots:
        print(
            "\nNo Opentrons hardware is reachable.\n"
            "  - If on USB: confirm the robot is powered on and the cable is a DATA\n"
            "    cable. A charge-only cable leaves the USB bus completely empty.\n"
            "  - If on Wi-Fi: put the robot on this machine's subnet.\n"
            "Meanwhile: --transport sim runs the whole control path with no hardware."
        )
    return {"serial": ports, "http": robots}


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------
def build_transport(args) -> Transport:
    if args.transport == "sim":
        return SimTransport()
    if args.transport == "serial":
        port = args.port
        if not port:
            cands = find_serial_ports()
            if not cands:
                raise SystemExit("no USB serial port found; pass --port explicitly")
            port = cands[0][0]
            print(f"auto-selected serial port {port}")
        return SerialGCodeTransport(port=port, baud=args.baud, model=args.model)
    host = args.host
    if not host:
        cands = find_http_robots()
        if not cands:
            raise SystemExit("no robot-server found; pass --host explicitly")
        host = cands[0]
        print(f"auto-selected host {host}")
    return HTTPTransport(host=host, mount=args.mount)


def parse_axes(spec: Optional[str]) -> Optional[Tuple[str, ...]]:
    if not spec:
        return None
    axes = tuple(a.strip().upper() for a in spec.split(",") if a.strip())
    bad = [a for a in axes if a not in ("X", "Y", "Z", "A", "B", "C")]
    if bad:
        raise SystemExit(f"unknown axes: {','.join(bad)}")
    risky = [a for a in axes if a in STALLED_AXES]
    if risky:
        print(f"WARNING: {','.join(risky)} stalled previously on this machine. "
              f"Confirm the endstop and free travel before homing it.")
    return axes


def cmd_estop(args) -> int:
    """Panic button. Cuts motion; commands none."""
    transport = build_transport(args)
    try:
        transport.open()
    except TransportError as e:
        # Still worth slamming the brakes: open() sets up the port before it
        # runs its handshake, so the estop write can land even if that failed.
        print(f"connect reported: {e}")
    print("sending emergency stop ...")
    transport.estop()
    transport.close()
    print("sent: Ctrl-X reset, M112 halt, M18 steppers off.")
    print("If the noise continues, cut power at the switch. Do not rely on software here.")
    return 0


def cmd_endstops(args) -> int:
    """Read endstop states. Commands no motion. This is the Y diagnostic."""
    transport = build_transport(args)
    try:
        transport.open()
    except TransportError as e:
        print(f"connect failed: {e}")
        return 1
    try:
        states = transport.endstops()
    except TransportError as e:
        print(f"endstop read failed: {e}")
        transport.close()
        return 1
    print("\nendstop states (1 = triggered):")
    for k in sorted(states):
        print(f"  {k}: {states[k]}")
    print(
        "\nY DIAGNOSTIC: press the Y endstop switch by hand and re-run this.\n"
        "  min_y changes  -> switch is good, so Y was blocked mechanically or\n"
        "                    homed in the wrong direction.\n"
        "  min_y stuck    -> the switch or its wiring is the fault, which fully\n"
        "                    explains Y never acking its home and grinding."
    )
    transport.close()
    return 0


def cmd_config(args) -> int:
    """Dump the board's own axis configuration. Commands no motion.

    This is the "learn the homing positions first" step. Everything here comes
    from the board, so it replaces guesswork about the envelope.
    """
    transport = build_transport(args)
    try:
        transport.open()
    except TransportError as e:
        print(f"connect failed: {e}")
        return 1
    try:
        cfg = transport.axis_config()
    except TransportError as e:
        print(f"config read failed: {e}")
        transport.close()
        return 1

    print("\n=== axis configuration, straight from the board ===")
    print("(Smoothieware: alpha=X beta=Y gamma=Z delta=A epsilon=B zeta=C)")
    for axis in ("X", "Y", "Z", "A", "B", "C"):
        vals = cfg.get(axis) or {}
        if not any(vals.values()):
            continue
        print(f"\n-- {SMOOTHIE_AXIS_MAP[axis]} ({axis}) --")
        for k in CONFIG_KEYS:
            print(f"   {k:24} -> {vals.get(k) or '(no value)'}")

    try:
        learned = transport.learn_limits()
        print("\n=== derived soft limits (use these, not the hard-coded guesses) ===")
        for a in sorted(learned):
            lo, hi = learned[a]
            print(f"   {a}: ({lo:.1f}, {hi:.1f})")
    except TransportError as e:
        print(f"\ncould not derive limits: {e}")

    ydir = (cfg.get("Y") or {}).get("homing_direction", "")
    if ydir:
        print(f"\nY homing_direction = {ydir!r}")
        print("  If that points away from where the Y endstop physically sits, the")
        print("  axis drives to the far stop and grinds. That would be the fault.")

    transport.close()
    return 0


def cmd_home(args) -> int:
    """Home only, then report the true home coordinates.

    Homing is bounded by the endstops, so it cannot be sent to a bad
    coordinate the way an absolute move can. That makes it both the safest
    first motion and the way to learn the real envelope before trusting the
    soft limits in this file.
    """
    transport = build_transport(args)
    driver = OTDriver(transport, allow_motion=args.go)
    try:
        driver.connect()
    except TransportError as e:
        print(f"connect failed: {e}")
        return 1

    print(f"\nconnected: {driver.describe()}")
    try:
        print(f"position BEFORE home: {driver.position()}")
        print("  (all zeros with untriggered endstops means 'unknown', not 'at origin')")
    except TransportError as e:
        print(f"position read unavailable: {e}")

    if not args.go:
        print("\nDry run. Motion BLOCKED (no --go). Re-run with --go to home.")
        driver.disconnect()
        return 0

    axes = parse_axes(args.axes) or DEFAULT_HOME_AXES
    print(f"\n*** MOTION: homing {','.join(axes)} in that order. "
          f"Mount axes lift before the gantry sweeps. ***")
    if "Y" not in axes:
        print("    (Y is excluded by default because it stalled on this machine.)")
    try:
        driver.home(axes)
    except (TransportError, MotionBlocked) as e:
        print(f"HOME FAILED: {e}")
        driver.disconnect()
        return 1

    pos = driver.position()
    print(f"\nhomed. TRUE home position: {pos}")
    print("Record these numbers; they define the real envelope for SOFT_LIMITS.")
    driver.disconnect()
    return 0


def cmd_demo(args) -> int:
    transport = build_transport(args)
    driver = OTDriver(transport, allow_motion=args.go)

    try:
        driver.connect()
    except TransportError as e:
        print(f"connect failed: {e}")
        return 1

    print(f"\nconnected: {driver.describe()}")

    if getattr(args, "learn_limits", False):
        try:
            learned = driver.learn_limits()
            print(f"learned envelope from the board: {learned}")
        except TransportError as e:
            # Asked to learn and could not: refuse to fall back to guesses,
            # because moving on a guessed envelope is what overreached before.
            print(f"could not learn the envelope from the board: {e}")
            print("refusing to move on guessed limits.")
            driver.disconnect()
            return 1

    src = "LEARNED from board" if driver.limits_are_learned else "hard-coded GUESS"
    print(f"soft limits ({transport.model}, {src}): {driver.limits}")

    if not args.go:
        print(
            "\nDry run. Motion is BLOCKED (no --go).\n"
            "Re-run with --go to actually move. Clear the deck first."
        )
        driver.disconnect()
        return 0

    print("\n*** MOTION ENABLED. The gantry will move. ***")
    try:
        _axes = parse_axes(args.axes) or DEFAULT_HOME_AXES
        print(f"homing {','.join(_axes)} ...")
        driver.home(_axes)
        print(f"  homed. position {driver.position()}")

        # Only command axes that actually homed. Y is excluded by default
        # because it stalled on this machine, and Z is deliberately left
        # uncommanded: its sign convention is unverified here and guessing
        # drives the pipette down into the deck.
        print(f"  homed axes:   {','.join(driver.homed_axes) or 'none'}")
        print(f"  movable (XYZ): {','.join(driver.movable_axes) or 'none'}")
        if "X" not in driver.movable_axes:
            print("\nX is not homed, so there is no axis safe to demo. Stopping.")
            driver.disconnect()
            return 1

        lo, hi = driver.limits["x"]
        mid = (lo + hi) / 2.0
        cur = driver.position()
        print(f"\nsweeping X only (Y and Z are not commanded at all):")
        for tx in (mid, max(lo, mid - 60.0), mid):
            p = driver.move_to(tx, cur.y, cur.z,
                               feedrate=args.feedrate, axes=("X",))
            print(f"  move X -> {p}")
            time.sleep(0.4)

        print("returning home ...")
        driver.home(_axes)
        print(f"  home. position {driver.position()}")
    except (TransportError, OutOfBounds, MotionBlocked) as e:
        print(f"MOTION ABORTED: {e}")
        driver.disconnect()
        return 1

    driver.disconnect()
    print("\nPOC complete: the robot moved under driver control.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Opentrons control driver")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("detect", help="find attached/reachable hardware, no motion")

    def add_conn_args(p):
        p.add_argument("--transport", choices=["sim", "serial", "http"], default="sim")
        p.add_argument("--port", help="serial device, e.g. /dev/cu.usbmodem1234")
        p.add_argument("--baud", type=int, default=SERIAL_BAUD)
        p.add_argument("--host", help="robot IP for the http transport")
        p.add_argument("--mount", default="right", choices=["left", "right"])
        p.add_argument("--model", default="ot-one", choices=sorted(SOFT_LIMITS))
        p.add_argument("--feedrate", type=float, default=2000.0,
                       help="mm/min; deliberately slow so you can watch it")
        p.add_argument("--axes", default=None,
                       help=f"comma-separated axes to home; default "
                            f"{','.join(DEFAULT_HOME_AXES)} (Y excluded: it stalled)")
        p.add_argument("--learn-limits", action="store_true", dest="learn_limits",
                       help="read the real envelope from the board instead of "
                            "trusting the hard-coded guesses")
        p.add_argument("--go", action="store_true", help="REQUIRED to allow motion")

    add_conn_args(sub.add_parser("home", help="home only, then report true home coords"))
    add_conn_args(sub.add_parser("demo", help="home, then run a short move sequence"))
    add_conn_args(sub.add_parser("estop", help="panic button: cut motion now"))
    add_conn_args(sub.add_parser("endstops", help="read endstop states, no motion"))
    add_conn_args(sub.add_parser("config", help="dump the board's axis config, no motion"))

    args = ap.parse_args()
    if args.cmd == "detect":
        detect()
        return 0
    if args.cmd == "estop":
        return cmd_estop(args)
    if args.cmd == "endstops":
        return cmd_endstops(args)
    if args.cmd == "config":
        return cmd_config(args)
    if args.cmd == "home":
        return cmd_home(args)
    return cmd_demo(args)


if __name__ == "__main__":
    sys.exit(main())
