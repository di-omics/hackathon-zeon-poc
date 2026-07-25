# Tip pickup calibration: measured result

Confirmed working on the bench OT-One, 2026-07-25. The pipette was lowered onto a
tip in the rack, engaged it, and raised 20 mm with the tip retained. Numbers below
are what the machine actually did, not targets.

## The datum, and its caveat

This machine has no working endstops (see `HARDWARE-FINDINGS.md`), so `G28.2 Z`
does not stop on a switch. It drives its full search distance, reaches the
mechanical top stop, and zeroes the counter there.

That does give a repeatable reference: the top of travel. But it is reached by
driving into a hard stop rather than tripping a switch, so **every home stresses
the mechanism**. Treat it as a working datum, not a good one. Fixing the endstop
wiring or firmware config is the real repair.

Homing `Z` takes 5.4 to 6.6 s because it always runs the whole search.

## Engagement depth

    tip engagement:  53 mm below the post-home top position
    verified lift:   20 mm raised, tip retained

Approach was done in decreasing increments, which is what kept it safe:

| Phase             | Step size | Feedrate           | Cumulative depth |
|-------------------|-----------|--------------------|------------------|
| initial descent   | 2 mm      | 300 mm/min (5 mm/s)| 0 to 49 mm       |
| final approach    | 1 mm      | 240 mm/min (4 mm/s)| 26 to 33 mm      |
| tip engagement    | 2 mm      | 180 mm/min (3 mm/s)| 49 to 53 mm      |

No step ever showed elevated duration, so no mechanical load was detected on the
way down. Contact was confirmed visually by the operator, not by the software.

## Reproducing it

Relative jogging only. Absolute coordinates reference a datum that is not
physically established, so do not use them here.

    # 1. establish the datum (drives to the top stop, ~6 s)
    python3 ot_driver.py home --transport serial --port /dev/cu.usbmodem11201 \
        --axes Z --go

    # 2. descend to ~49 mm in 2 mm steps, watching
    python3 scripts/jog_z.py 2        # repeat

    # 3. last few mm in 1 mm steps
    python3 scripts/jog_z.py 1        # repeat

    # 4. raise and confirm the tip stayed on
    python3 scripts/jog_z.py -2       # repeat

`jog_z.py` caps one step at 15 mm and restores `G90` afterwards.

## What the timings do and do not prove

`M400` blocks until the planner queue drains, so its duration confirms a move
*ran*. Measured against prediction it was accurate throughout: 2 mm at 5 mm/s
came in at 0.39 to 0.45 s against 0.40 s predicted; the 20 mm raise took 4.00 s
against 4.00 s predicted.

It does **not** prove the nozzle was clear. With no endstops and no current
sensing, a stalled stepper skips steps and the timing is indistinguishable from a
clean move. **A crash is invisible to the software.** The operator watching is the
only protection, and that was true for every number on this page.

## Known interruption

Serial writes to the board timed out twice mid-session
(`SerialTimeoutException: Write timeout`), which aborted one 2 mm step before it
was issued. The board stays enumerated on USB while wedged. Recovery is a full
power cycle: power off, unplug USB, wait about 5 s, reconnect.

`write_timeout` on the port is what turns this into a clean abort instead of a
hang. Do not remove it.
