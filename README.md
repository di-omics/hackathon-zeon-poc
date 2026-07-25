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

- **No endstop on ANY axis registers with the board.** This is established, not a
  candidate: `M119` was polled for 18 s while the `Z` limit switch was pressed by
  hand and no bit ever changed, and the firmware config has no axis limit entries
  (`config-get sd gamma_min` returns `not in config`). So `G28.2` never terminates
  on a limit; it drives a fixed search distance and zeroes the counter. Homing `Z`
  twice in a row took 5.44 s then 6.56 s, where a real second home would finish
  in a fraction of a second because it starts already on the switch.
- **`Y` does not home**, and this is why. `G28.2 Y` drove looking for a switch
  that never reports and ground against a hard stop. Y is not specially broken;
  `Z` does the same thing, just without an obstruction in its path. See
  `HARDWARE-FINDINGS.md`.
- **`Z=0` after homing is therefore not a physical datum**, and absolute
  positioning cannot be trusted on this machine. Use relative `G91` jogging
  (`scripts/jog_z.py`), which needs no datum.
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

If a config does expose `homing_direction`, a wrong value can explain a home that
drives away from its switch. This firmware exposes no such value, but the Y fault
no longer needs that diagnosis: the endstop poll above showed no switch is read on
any axis, which accounts for the stall on its own.

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
