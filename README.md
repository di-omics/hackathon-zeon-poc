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

## Quick start

    python3 ot_driver.py detect                     # what is attached
    python3 ot_driver.py demo --transport sim --go   # full control path, no hardware
    python3 ot_driver.py endstops --transport serial --port /dev/cu.usbmodem11201
    python3 ot_driver.py estop --transport serial --port /dev/cu.usbmodem11201

Nothing moves without `--go`. `estop` and `endstops` never command motion.

## Hardware state, honestly

Verified on the bench unit on 2026-07-25:

- The board enumerates as `Smoothieboard` (vendor `Uberclock`) on
  `/dev/cu.usbmodem11201`, firmware `v1.0.3`.
- `M114.2` reports six axes, `X Y Z A B C`. That is the Opentrons layout:
  `X`/`Y` gantry, `Z` left mount, `A` right mount, `B`/`C` plungers.
- `M119` reports `min_x min_y min_z min_a min_b`. There is no `min_c`.
- Homing `Z`, then `A`, then `X` each acknowledged cleanly.

Not working, and not to be claimed as working:

- **`Y` does not home.** `G28.2 Y` never acknowledges and the axis stalls
  audibly against a stop. Cause not yet established. The candidates are a
  blocked axis, an inverted homing direction, or a dead or disconnected `Y`
  endstop switch. Run the `endstops` command and press the switch by hand to
  tell those apart.
- Because of that, `Y` is excluded from `DEFAULT_HOME_AXES`, and the driver
  refuses to command any axis it has not homed, since an absolute move on an
  unhomed axis can drive it into a hard stop.
- `Z` is deliberately never commanded by the demo. Its sign convention is not
  verified on this machine and guessing sends the pipette into the deck.
- No full-envelope move and no liquid handling has been demonstrated.

## Values that still need verifying

`SOFT_LIMITS` and the GCode words in `ot_driver.py` are documented platform
defaults, not measurements from this machine. They are marked in the source.
Confirm them on the hardware before trusting them.

## Safety notes

The emergency stop sends Ctrl-X first, then `M112`, then `M18`. Ctrl-X goes
first because it is handled at the serial layer and so interrupts a move that is
already executing; `M112` alone can sit in the queue behind that very move.

Closing the serial port does **not** stop an in-flight move. An earlier version
of this driver relied on that and let a stalling axis grind. Motion commands now
carry a bounded timeout and fire the emergency stop before raising.

If a noise persists, cut power at the switch. Do not rely on software.
