#!/usr/bin/env python3
"""RELATIVE Z jog. Needs no datum, because this machine has no working endstops.

Usage:  python3 jog_z.py <mm>      positive = DOWN, negative = UP

Uses G91 relative mode so nothing depends on the fictional home position, then
restores G90 so the machine is left in a known mode. Takes ONE step and stops.

There is labware on the deck and NO endstop safety net, so the operator watching
this is the only protection. Keep steps small.
"""
import sys
import time

import serial

PORT = "/dev/cu.usbmodem11201"
FEED = 300.0          # mm/min = 5 mm/s. Slow enough to stop by hand.
MAX_STEP = 15.0       # refuse anything larger in a single relative step

step = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
if abs(step) > MAX_STEP:
    raise SystemExit(f"refusing a {step} mm step: cap is {MAX_STEP} mm per jog "
                     f"with no endstops and labware on the deck.")

s = serial.Serial(PORT, 115200, timeout=0.25, write_timeout=0.5)
time.sleep(2.0)
s.reset_input_buffer()


def cmd(c, budget=30.0):
    s.reset_input_buffer()
    t0 = time.time()
    s.write((c + "\r\n").encode())
    s.flush()
    out = ""
    while time.time() - t0 < budget:
        ch = s.readline().decode(errors="replace")
        if ch:
            out += ch
            if "ok" in ch.lower():
                break
    return time.time() - t0, " ".join(out.split())


def estop():
    try:
        s.write(b"\x18")
        time.sleep(0.2)
        s.write(b"M112\r\n")
    except Exception:
        pass


try:
    cmd("M17")                      # ensure steppers energised
    print(f"jogging Z {step:+.1f} mm ({'DOWN' if step > 0 else 'UP'}) "
          f"at F{FEED:.0f} = {FEED/60:.1f} mm/s")
    print("WATCH IT. Cut power at the switch the moment it looks close.\n")

    cmd("G91")                      # relative mode: no datum needed
    dt_q, resp = cmd(f"G0 Z{step:.2f} F{FEED:.0f}")
    dt_m, _ = cmd("M400")           # blocks until the move actually finishes
    cmd("G90")                      # restore absolute mode

    expected = abs(step) / (FEED / 60.0)
    print(f"  queued in {dt_q*1000:.0f} ms; motion took {dt_m:.2f}s "
          f"(expected ~{expected:.2f}s for {abs(step):.1f} mm)")
    if dt_m < expected * 0.5:
        print("  WARNING: finished far too fast. The move may not have run.")
    print(f"  endstops: {cmd('M119')[1]}")
    print(f"\nOne {step:+.1f} mm step done. Re-run to continue, e.g. "
          f"python3 jog_z.py {step:+.0f}")
    print("Go UP with a negative value, e.g. python3 jog_z.py -2")
except BaseException as e:
    print(f"\nABORTED: {type(e).__name__}: {e}")
    estop()
    print("emergency stop attempted. CUT POWER if anything is still moving.")
    raise
finally:
    try:
        s.write(b"G90\r\n")
        s.close()
    except Exception:
        pass
