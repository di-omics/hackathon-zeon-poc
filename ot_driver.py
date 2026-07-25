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
GCODE_WAIT_MOVES = "M400"         # blocks until the planner queue drains, so its
                                  # ack is the real "movement finished" signal.
                                  # G0/G28.2 ack when QUEUED, not when arrived.
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
# Axes whose direction convention is not verified on this machine. They are
# homed (which is endstop-bounded and therefore safe) but never included in a
# default move, because guessing the sign drives the pipette into the deck.
# Pass them explicitly to move them on purpose.
UNVERIFIED_AXES = ("Z",)

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
    # Set by estop(). Ctrl-X resets the board and discards its homing reference,
    # and _send can fire estop from deep inside the transport where OTDriver
    # cannot see it. This flag is how that fact travels back up.
    reference_lost = False

    def open(self) -> None: ...
    def close(self) -> None: ...
    def describe(self) -> str: ...
    def home(self, axes: Optional[Tuple[str, ...]] = None) -> None: ...
    def move_to(self, x: float, y: float, z: float, feedrate: float,
                axes: Optional[Tuple[str, ...]] = None) -> None: ...
    def position(self) -> Position: ...

    def estop(self) -> bool:
        """Cut motion. Returns False where the transport cannot actually do it."""
        return False

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

    def estop(self) -> bool:
        self.log.append("estop")
        self.reference_lost = True
        return False  # a simulator stops nothing physical; never claim it did

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
    reference_lost: bool = False
    _ser: Optional[object] = None

    def open(self) -> None:
        self.reference_lost = False   # fresh connection, no stale loss carried in
        try:
            import serial  # pyserial
        except ImportError as e:
            raise TransportError("pyserial is required: pip install pyserial") from e
        try:
            # write_timeout matters: without it, a write to a board that is
            # enumerated but wedged blocks forever, which would hang the very
            # estop meant to rescue that situation.
            self._ser = serial.Serial(self.port, self.baud,
                                      timeout=self.read_poll, write_timeout=0.5)
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

    def estop(self) -> bool:
        """Cut motion immediately. Best effort, never raises.

        Ctrl-X goes first because it is handled at the serial layer and so
        interrupts a move already executing; M112 can otherwise sit in the
        queue behind that very move. Closing the port does neither, which is
        why the original stall kept grinding after the script exited.

        Returns True only if at least one payload was actually written to the
        port. Callers MUST report that honestly: an estop that printed success
        while writing nothing would be worse than no estop at all.
        """
        # Mark the reference lost even if no byte lands: if we are trying to
        # stop, the machine's position can no longer be trusted either way.
        self.reference_lost = True
        if self._ser is None:
            return False
        wrote = False
        for payload in (SMOOTHIE_RESET, (GCODE_ESTOP + "\r\n").encode(),
                        (GCODE_DISABLE_STEPPERS + "\r\n").encode()):
            try:
                # Deliberately NOT calling flush(). On POSIX flush() is
                # tcdrain(), which waits for the output buffer to drain and is
                # NOT bounded by write_timeout, so it can block forever against
                # exactly the wedged board this is meant to rescue. write()
                # alone hands the bytes to the kernel, which is enough.
                self._ser.write(payload)
                wrote = True
                time.sleep(0.2)
            except Exception:
                # Includes SerialTimeoutException from write_timeout. Keep
                # trying the remaining payloads; report honestly at the end.
                pass
        return wrote

    def _send(self, cmd: str, motion: bool = False,
              timeout: Optional[float] = None) -> str:
        if self._ser is None:
            raise TransportError("serial port is not open")
        budget = timeout if timeout is not None else (
            self.motion_timeout if motion else self.ack_timeout)
        # Drop anything left over from a previous command. A single orphaned
        # "ok" in the buffer would otherwise satisfy THIS command instantly,
        # which silently disables the no-ack stall detection below.
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
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

    def _motion(self, cmd: str) -> None:
        """Issue a motion command and wait until the motion actually finishes.

        G0 and G28.2 acknowledge when the command is QUEUED, not when the
        gantry arrives. So an ack on the move itself proves nothing and a
        timeout on it cannot detect a stall. M400 blocks until the planner
        queue drains, so its ack is the real "finished" signal and is the one
        worth timing out on and firing the estop from.
        """
        self._send(cmd, motion=True)
        self._send(GCODE_WAIT_MOVES, motion=True)

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
            lo = hi = None
            try:
                lo = float(vals.get("min", ""))
            except (TypeError, ValueError):
                pass
            try:
                hi = float(vals.get("max", ""))
            except (TypeError, ValueError):
                pass
            # Deliberately NOT falling back to max_travel. max_travel is the
            # distance homing is allowed to SEARCH, and it is normally set
            # larger than the axis so homing cannot come up short. Using it as
            # the soft-limit ceiling would widen the envelope past the physical
            # end of travel, which is the exact overreach this is meant to stop.
            if lo is None or hi is None or hi <= lo:
                continue
            limits[axis.lower()] = (lo, hi)
        if not limits:
            raise TransportError(
                "the board reported no usable min/max pair for any axis, so the "
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
            self._motion(f"{GCODE_HOME} {axis}")

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
        self._motion(f"{GCODE_MOVE} {words} F{feedrate:.0f}")

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
        self._position_known = False
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

    def _sync_reference(self) -> None:
        """Adopt a reference loss the transport reported from inside a send.

        _send fires estop() directly on a motion timeout, which resets the board
        and destroys its homing reference. Without this, the driver would still
        believe it was homed and would happily issue the next absolute move from
        a datum the board no longer has.
        """
        if getattr(self.t, "reference_lost", False):
            self._homed = False
            self._homed_axes = ()
            self._position_known = False

    def _guard_motion(self) -> None:
        self._sync_reference()
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
    def estop(self) -> bool:
        """Cut motion now. Safe to call unconditionally.

        Ctrl-X resets the board, which DISCARDS its homing reference. The
        Python object must forget it too, or the next move would be commanded
        absolutely from a position the board no longer knows, re-arming the
        exact hard-stop scenario the unhomed-axis guard exists to prevent.
        """
        wrote = self.t.estop()
        self._homed = False
        self._homed_axes = ()
        self._position_known = False
        return wrote

    def endstops(self) -> Dict[str, int]:
        return self.t.endstops()

    def home(self, axes: Optional[Tuple[str, ...]] = None) -> None:
        self._guard_motion()
        # Clear the reference FIRST. If this home fails partway through, the
        # axis list from a PREVIOUS successful home must not survive and go on
        # claiming those axes are referenced when the board has been reset.
        self._homed = False
        self._homed_axes = ()
        self._position_known = False
        # BaseException, not Exception: a Ctrl-C mid-home must still cut motion.
        # Without this, KeyboardInterrupt unwinds the stack and closes the port
        # while the axis keeps driving, which is exactly the original failure.
        try:
            self.t.home(axes)
        except BaseException:
            self.estop()
            raise
        # A completed home re-establishes the reference, so clear the flag or
        # _sync_reference would keep tearing down the state we just earned.
        self.t.reference_lost = False
        self._homed = True
        self._homed_axes = tuple(axes) if axes is not None else DEFAULT_HOME_AXES
        # Read back rather than assuming home is the origin. On this hardware
        # the mount axes home to the TOP of their travel, so asserting (0,0,0)
        # would make every subsequent relative move compute from a false base.
        try:
            self._last = self.t.position()
            self._position_known = True
        except TransportError:
            # Do NOT fabricate an origin here. A failed read-back means the
            # position is genuinely unknown, and inventing (0,0,0) would give
            # every later relative move a fake datum, which could drive a mount
            # from the top of its travel down to zero. Absolute moves are still
            # fine; only relative ones need a datum, and jog() checks this flag.
            self._position_known = False

    @property
    def homed_axes(self) -> Tuple[str, ...]:
        return self._homed_axes

    @property
    def position_known(self) -> bool:
        return self._position_known

    @property
    def movable_axes(self) -> Tuple[str, ...]:
        """Cartesian axes that have been homed, so are safe to command."""
        return tuple(a for a in ("X", "Y", "Z") if a in self._homed_axes)

    @property
    def safe_axes(self) -> Tuple[str, ...]:
        """Axes a move will touch when the caller does not name any.

        Excludes axes whose direction convention is unverified, and, once the
        envelope has been learned, any axis the board gave no limits for. Name
        an axis explicitly to move it anyway.
        """
        out = [a for a in self.movable_axes if a not in UNVERIFIED_AXES]
        if self._learned_limits:
            out = [a for a in out if a.lower() in self._learned_limits]
        return tuple(out)

    def move_to(self, x: float, y: float, z: float,
                feedrate: Optional[float] = None,
                axes: Optional[Tuple[str, ...]] = None) -> Position:
        self._guard_motion()
        if not self._homed:
            raise TransportError("home() before moving; position is unknown until homed")
        target = axes if axes is not None else self.safe_axes
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
        except BaseException:
            # BaseException so a Ctrl-C mid-move also cuts motion instead of
            # unwinding the stack with the gantry still travelling.
            self.estop()
            raise
        self._last = Position(
            x if "X" in target else self._last.x,
            y if "Y" in target else self._last.y,
            z if "Z" in target else self._last.z,
        )
        return self._last

    def jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
            feedrate: Optional[float] = None,
            axes: Optional[Tuple[str, ...]] = None) -> Position:
        # A relative move needs a trustworthy starting point. Absolute moves do
        # not, which is why only this path enforces it.
        if not self._position_known:
            raise TransportError(
                "position is unknown (the read-back failed, or an estop reset "
                "the board and discarded its homing reference), so a relative "
                "move has no valid datum. Home again before jogging."
            )
        base = self._last
        return self.move_to(base.x + dx, base.y + dy, base.z + dz,
                            feedrate, axes)

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


_ACTIVE_MOTION_TRANSPORT: Optional[Transport] = None


def _install_panic_handlers(transport: Transport) -> None:
    """Make Ctrl-C and SIGTERM cut motion before the process unwinds.

    Ctrl-C is the first thing an operator reaches for on hearing a grind. Left
    unhandled it kills the process before any timeout can fire the estop, so the
    axis keeps driving and the interrupt makes the outcome strictly WORSE than
    doing nothing.
    """
    global _ACTIVE_MOTION_TRANSPORT
    _ACTIVE_MOTION_TRANSPORT = transport
    import signal

    def _handler(signum, _frame):
        t = _ACTIVE_MOTION_TRANSPORT
        print(f"\nsignal {signum} received: cutting motion first ...")
        if t is not None and t.estop():
            print("emergency stop written to the board.")
        else:
            print("emergency stop could NOT be written. CUT POWER AT THE SWITCH NOW.")
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handler)
        except Exception:
            pass


def panic_estop_serial(port: str, baud: int = SERIAL_BAUD) -> bool:
    """Open a serial port and write the stop sequence with NO handshake.

    The normal open() sleeps ~2s for the board to boot and then probes it. When
    an axis is grinding, those two seconds are two seconds of damage, so this
    path writes Ctrl-X the instant the port exists and skips every check.
    """
    try:
        import serial
    except ImportError:
        print("pyserial is not installed, so nothing can be sent.")
        return False
    try:
        ser = serial.Serial(port, baud, timeout=0.25, write_timeout=0.5)
    except Exception as e:
        print(f"could not open {port}: {e}")
        return False
    wrote = False
    try:
        for payload in (SMOOTHIE_RESET, (GCODE_ESTOP + "\r\n").encode(),
                        (GCODE_DISABLE_STEPPERS + "\r\n").encode()):
            try:
                ser.write(payload)   # no flush: tcdrain can block forever
                wrote = True
                time.sleep(0.2)
            except Exception:
                pass
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return wrote


def cmd_estop(args) -> int:
    """Panic button. Cuts motion; commands none."""
    # Fast path for serial: skip the connect handshake entirely.
    if args.transport == "serial":
        port = args.port
        if not port:
            cands = find_serial_ports()
            if not cands:
                print("no USB serial port found; pass --port explicitly.")
                print("CUT POWER AT THE SWITCH NOW.")
                return 1
            port = cands[0][0]
        print(f"sending emergency stop to {port} (no handshake) ...")
        wrote = panic_estop_serial(port, args.baud)
        if wrote:
            print("WROTE to the board: Ctrl-X reset, M112 halt, M18 steppers off.")
        else:
            print("NOTHING WAS SENT. No bytes reached the board.")
        print("If the noise continues, cut power at the switch. Do not rely on software.")
        return 0 if wrote else 1

    transport = build_transport(args)
    try:
        transport.open()
    except TransportError as e:
        # Still worth slamming the brakes: open() sets up the port before it
        # runs its handshake, so the estop write can land even if that failed.
        print(f"connect reported: {e}")
    print("sending emergency stop ...")
    wrote = transport.estop()
    transport.close()
    if wrote:
        print("WROTE to the board: Ctrl-X reset, M112 halt, M18 steppers off.")
    else:
        print("NOTHING WAS SENT. No bytes reached any board.")
        if args.transport == "sim":
            print("  Transport is 'sim', which stops nothing physical.")
            print("  Use: --transport serial --port /dev/cu.usbmodem<NNN>")
        else:
            print("  The port could not be opened, or every write failed.")
    print("If the noise continues, cut power at the switch. Do not rely on software.")
    return 0 if wrote else 1


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
    _install_panic_handlers(transport)
    try:
        driver.home(axes)
    except BaseException as e:
        print(f"HOME FAILED: {type(e).__name__}: {e}")
        if driver.estop():
            print("emergency stop written to the board.")
        else:
            print("emergency stop could NOT be written. Cut power at the switch.")
        driver.disconnect()
        return 1

    if driver.position_known:
        print(f"\nhomed. TRUE home position: {driver.position()}")
        print("Record these numbers; they are the real datum for this machine.")
    else:
        print("\nhomed, but the position READ-BACK FAILED, so the datum is unknown.")
        print("Not printing a number here: inventing one is how a bogus datum")
        print("gets recorded as fact. Re-run to retry the read.")
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
    _install_panic_handlers(transport)
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
    except BaseException as e:
        # BaseException so Ctrl-C also reaches the estop rather than unwinding
        # with the gantry still moving.
        print(f"MOTION ABORTED: {type(e).__name__}: {e}")
        if driver.estop():
            print("emergency stop written to the board.")
        else:
            print("emergency stop could NOT be written. Cut power at the switch.")
        driver.disconnect()
        return 1

    driver.disconnect()
    print("\nPOC complete: the robot moved under driver control.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Opentrons control driver")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("detect", help="find attached/reachable hardware, no motion")

    def add_conn_args(p, default_transport: str = "sim"):
        # Hardware-facing commands default to serial. Defaulting the panic
        # button to the simulator would let it report success while writing
        # nothing to any board.
        p.add_argument("--transport", choices=["sim", "serial", "http"],
                       default=default_transport)
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
    add_conn_args(sub.add_parser("estop", help="panic button: cut motion now"), "serial")
    add_conn_args(sub.add_parser("endstops", help="read endstop states, no motion"), "serial")
    add_conn_args(sub.add_parser("config", help="dump the board's axis config, no motion"), "serial")

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
