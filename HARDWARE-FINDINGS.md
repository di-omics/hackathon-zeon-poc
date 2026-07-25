# OT-One bring-up: what the hardware actually does

Measured on the bench unit 2026-07-25. Everything here was read off the machine,
not assumed. The headline finding invalidates the usual homing approach, so read
it before writing any positioning code against this robot.

## The machine has no working endstops

Polled `M119` for 18 seconds while the Z limit switch was pressed by hand. **Not
one endstop bit ever changed.** Baseline and final state were identical:

    min_x:0 min_y:0 min_z:0 min_a:0 min_b:0

The firmware config has no axis limit entries either:

    config-get sd gamma_min             -> sd: gamma_min is not in config
    config-get sd gamma_max_travel      -> sd: gamma_max_travel is not in config
    config-get sd gamma_homing_direction-> sd: gamma_homing_direction is not in config

### What follows from that

`G28.2` does **not** home to a limit on this machine. It drives a fixed search
distance, gives up, and zeroes the position counter. Evidence: homing Z twice in
a row took 5.44 s then 6.56 s. A real second home starts already on the switch
and finishes in a fraction of a second. Both runs burned the full search.

So **`Z=0` after homing is not a physical datum.** Any absolute coordinate is
referenced to a position that was never established. Do not use absolute moves
for positioning on this robot.

This is also the true root cause of the original Y stall. Y was not specially
broken: `G28.2 Y` drove looking for a switch that never reports, ran into a hard
stop, and ground. Z has been doing the same thing all along, just without an
obstruction in the way.

## Use relative jogging instead

`G91` relative mode needs no datum, so it is the only trustworthy way to position
this machine as currently wired. `scripts/jog_z.py` does one bounded step:

    python3 scripts/jog_z.py 2      # 2 mm DOWN
    python3 scripts/jog_z.py -2     # 2 mm UP

It caps a single step at 15 mm, runs at 3 to 5 mm/s so it can be stopped by hand,
and restores `G90` afterwards.

## Verified geometry

- **Down is +Z.** Established by observation: after homing the pipette sits at
  the top of its travel and Z reads 0, so increasing Z descends.
- A controlled descent of **53 mm** from the raised position brought the nozzle
  to the tip in the rack, done in 1 and 2 mm increments.
- A **20 mm** raise from there ran in 4.00 s against 4.00 s expected.

## Motion duration is the only feedback

With no endstops and no current sensing, the sole confirmation that a move
happened is how long `M400` blocks. `G0` and `G28.2` acknowledge when a move is
**queued**, not when it arrives, so their ack proves nothing. `M400` blocks until
the planner queue drains and is the real signal.

Measured, at F600 (10 mm/s), a 40 mm move: `G0` acked in 85 ms, `M400` blocked
4.60 s against a predicted 4.00 s. At F300, 2 mm steps consistently measured
0.39 to 0.45 s against 0.40 s predicted.

**A crash is invisible.** A stalled stepper skips steps and the timing looks
identical to a clean move. The operator watching is the only protection.

## The board wedges, and USB hides it

Twice the board stopped accepting writes mid-session, surfacing as
`SerialTimeoutException: Write timeout`. It stays enumerated on USB and can still
look healthy, so this is easy to misread as working.

Two traps that follow:

- The Smoothieboard logic is powered over **USB**, while the motor rail is
  separate. With the motor supply off, the board answers normally, accepts moves,
  updates its position registers, and reports correct durations, while **nothing
  physically moves.** Only the untriggered endstops hint at it.
- A software reset does not clear it. Recovery needs a real power cycle: power
  off, **unplug USB too**, wait about 5 s, reconnect.

`write_timeout` on the port is therefore mandatory. Without it a write to a
wedged board blocks forever, including the write inside the emergency stop.

## Emergency stop notes

Order matters: `Ctrl-X` (0x18) first, then `M112`, then `M18`. Ctrl-X is handled
at the serial layer so it interrupts a move already executing; `M112` alone can
sit in the queue behind that very move. Never call `flush()` in the stop path: on
POSIX that is `tcdrain`, which is not bounded by `write_timeout` and can block
forever against exactly the wedged board the stop exists to rescue.

`M112` latches the board in HALT, where it ignores everything until `M999`.

`M18` de-energizes the steppers, so send `M17` before expecting motion again
after any stop.
