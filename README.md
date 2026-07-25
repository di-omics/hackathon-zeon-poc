# Opentrons control driver POC

A single-file control driver for Opentrons hardware, plus a zero-dependency
"prove it moves" script. Built for the hackathon bring-up of an old-school OT.

## What is here

- `ot_driver.py` - the driver. One uniform API over three transports:
  - `serial` - GCode over USB serial, for an OT-One / Smoothieboard. This is the
    path that works when a robot has no onboard robot-server, or its
    robot-server is dead.
  - `http` - the robot-server HTTP API on port 31950, for an OT-2 or a Flex.
  - `sim` - no hardware. Tracks position in memory so the control logic, the
    soft limits and a demo script are all reviewable with nothing plugged in.
- `ot_move_poc.py` - stdlib only, no imports beyond the standard library. Finds a
  robot-server on the local subnets and commands a home plus a jog over HTTP.
- `ot_one_tip_calibrator.py` - guarded interactive console for the original
  OT-One. It commands the shared Z lift only, in steps no larger than 5 mm, and
  preserves the pickup press cycle from the archived OT-One API.

## Quick start

    python3 ot_driver.py detect                     # what is attached
    python3 ot_driver.py demo --transport sim --go   # full control path, no hardware
    python3 ot_driver.py endstops --transport serial --port /dev/cu.usbmodem11201
    python3 ot_driver.py estop --transport serial --port /dev/cu.usbmodem11201
    python3 ot_one_tip_calibrator.py --port /dev/cu.usbmodem11201

The general driver moves nothing without `--go`; `estop` and `endstops` never
command motion. The calibrator connects read-only and waits at a prompt. Its
`home`, `down`, `up`, `lift`, and `pickup` commands are the explicit motion gate.

## Hardware state, honestly

Verified on the bench unit on 2026-07-25:

- The board enumerates as `Smoothieboard` (vendor `Uberclock`) on
  `/dev/cu.usbmodem11201`, firmware `v1.0.3`.
- `M114.2` reports six firmware axes, `X Y Z A B C`. The archived OT-One API
  uses `X`/`Y` for the gantry, one shared `Z` lift, and `A`/`B` for the two
  pipette plungers. It does not use `C`.
- `M119` reports `min_x min_y min_z min_a min_b`. There is no `min_c`.
- Homing `Z`, then `A`, then `X` each acknowledged cleanly during the initial
  bring-up. The focused calibrator now homes only the shared `Z` lift.
- After a full USB and main-power cycle, the board reconnected and `Z` homed to
  zero. Guarded downward moves of 5 mm, 5 mm, and 3 mm arrived cleanly at
  `Z=13.0`. On this board, increasing Smoothie `Z` moves the head down.

Not working, and not to be claimed as working:

- **`Y` does not home.** `G28.2 Y` never acknowledges and the axis stalls
  audibly against a stop. Cause not yet established. The candidates are a
  blocked axis, an inverted homing direction, or a dead or disconnected `Y`
  endstop switch. Run the `endstops` command and press the switch by hand to
  tell those apart.
- Because of that, `Y` is excluded from `DEFAULT_HOME_AXES`, and the driver
  refuses to command any axis it has not homed, since an absolute move on an
  unhomed axis can drive it into a hard stop.
- The general demo still does not command `Z`. Tip calibration uses the separate
  Z-only console, whose per-move limit is 5 mm and whose guarded envelope is
  0..95 mm.
- No full-envelope move and no liquid handling has been demonstrated.

## Learn the envelope, do not guess it

`SOFT_LIMITS` in `ot_driver.py` are documented platform defaults, not
measurements from this machine, and commanding a coordinate past the real
envelope is what overreached during bring-up. The diagnostic command is:

    python3 ot_driver.py config --transport serial --port /dev/cu.usbmodem11201

That attempts to read Smoothieware's `config-get` values and commands no motion.
On this firmware (`v1.0.3`), the queried `gamma_*` travel and homing keys report
`not in config`; the board does not expose a usable envelope through this path.
The focused calibrator therefore uses the archived OT-One 100 mm Z dimension
with a 5 mm reserve and refuses any target outside 0..95 mm.

If a config does expose `homing_direction`, a wrong value can explain a home
that drives away from its switch. This particular firmware did not expose the
value, so the Y fault remains mechanical, wiring, or firmware-config diagnosis.

On firmware that exposes a complete envelope, the general driver can use it:

    python3 ot_driver.py demo --transport serial --port /dev/cu.usbmodem11201 \
        --learn-limits --go

On this v1.0.3 board the envelope cannot be read, so `--learn-limits` refuses to
move rather than silently falling back to guesses. Use the Z-only calibrator for
the current tip-pickup work.

## Safety notes

The emergency stop sends Ctrl-X first, then `M112`, then `M18`. Ctrl-X goes
first because it is handled at the serial layer and so interrupts a move that is
already executing; `M112` alone can sit in the queue behind that very move.

Closing the serial port does **not** stop an in-flight move. An earlier version
of this driver relied on that and let a stalling axis grind. Motion commands now
carry a bounded timeout and fire the emergency stop before raising.

If a noise persists, cut power at the switch. Do not rely on software.
