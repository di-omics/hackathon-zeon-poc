#!/usr/bin/env python3
"""Minimal Opentrons "make it move" POC.

Zero dependencies beyond the stdlib. Talks the robot-server HTTP API directly,
so it works over USB (link-local) or Wi-Fi, on an OT-2 or a Flex.

Usage:
    python3 ot_move_poc.py --discover              # find the robot, no motion
    python3 ot_move_poc.py --host 192.168.0.225    # target a known IP, no motion
    python3 ot_move_poc.py --host <ip> --home --yes        # HOMES the gantry
    python3 ot_move_poc.py --host <ip> --home --jog --yes  # home, then a visible jog

Nothing moves unless you pass --home/--jog AND --yes. The deck must be clear.
"""

import argparse
import json
import socket
import sys
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor

PORT = 31950
# The robot-server rejects requests without this header.
HEADERS = {"Opentrons-Version": "*", "Content-Type": "application/json"}


def call(host, path, method="GET", body=None, timeout=10):
    url = f"http://{host}:{PORT}{path}"
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method, headers=HEADERS)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw = r.read().decode()
        try:
            return r.status, json.loads(raw)
        except json.JSONDecodeError:
            return r.status, raw


def alive(host, timeout=1.5):
    """True if something is listening on the robot-server port."""
    try:
        with socket.create_connection((host, PORT), timeout=timeout):
            return True
    except OSError:
        return False


def local_subnets():
    """Every /24 this machine has an address on, plus link-local."""
    import subprocess

    nets = []
    out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("inet ") and "127.0.0.1" not in line:
            ip = line.split()[1]
            nets.append(".".join(ip.split(".")[:3]))
    # Subnet the Opentrons app last saw a robot on, and the USB link-local range.
    for extra in ("192.168.0", "169.254"):
        if extra not in nets:
            nets.append(extra)
    return nets


def discover():
    """Scan every plausible subnet for a robot-server."""
    found = []
    for net in local_subnets():
        if net == "169.254":
            # Link-local is a /16; only the .0/24s macOS self-assigns are worth a sweep.
            print(f"  skipping {net}.0.0/16 (too large; pass --host if you know it)")
            continue
        hosts = [f"{net}.{i}" for i in range(1, 255)]
        print(f"  scanning {net}.0/24 ...")
        with ThreadPoolExecutor(max_workers=128) as ex:
            for host, ok in zip(hosts, ex.map(alive, hosts)):
                if ok:
                    found.append(host)
                    print(f"  FOUND robot-server at {host}:{PORT}")
    return found


def identify(host):
    status, health = call(host, "/health")
    if status != 200:
        raise RuntimeError(f"/health returned {status}")
    model = health.get("robot_model", "unknown")
    print(f"  name:   {health.get('name')}")
    print(f"  model:  {model}")
    print(f"  api:    {health.get('api_version')}")
    print(f"  serial: {health.get('robot_serial')}")
    return health


def home(host):
    print("  POST /robot/home  (target=robot) ...")
    status, resp = call(host, "/robot/home", "POST", {"target": "robot"}, timeout=120)
    print(f"  -> {status} {resp}")
    return status == 200


def jog(host, mount, point):
    print(f"  POST /robot/move  mount={mount} point={point} ...")
    body = {"target": "pipette", "mount": mount, "point": point}
    status, resp = call(host, "/robot/move", "POST", body, timeout=120)
    print(f"  -> {status} {resp}")
    return status == 200


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", help="robot IP; omit to auto-discover")
    ap.add_argument("--discover", action="store_true", help="scan and exit")
    ap.add_argument("--home", action="store_true", help="HOME the gantry (motion)")
    ap.add_argument("--jog", action="store_true", help="after homing, move to a visible point")
    ap.add_argument("--mount", default="right", choices=["left", "right"])
    ap.add_argument("--point", default="200,200,150", help="x,y,z in mm for --jog")
    ap.add_argument("--yes", action="store_true", help="required to allow any motion")
    args = ap.parse_args()

    host = args.host
    if not host or args.discover:
        print("Discovering robots ...")
        found = discover()
        if not found:
            print(
                "\nNo robot-server found on any local subnet.\n"
                "The robot is not reachable from this machine. Check that it is\n"
                "powered on, and either on the same Wi-Fi network or connected by\n"
                "a data USB cable (a charge-only cable will not enumerate)."
            )
            return 1
        if args.discover:
            return 0
        host = found[0]
        print(f"Using {host}")

    print(f"\nIdentifying {host} ...")
    try:
        identify(host)
    except (urllib.error.URLError, OSError, RuntimeError) as e:
        print(f"  cannot reach {host}:{PORT} -> {e}")
        return 1

    if not (args.home or args.jog):
        print("\nConnection verified. No motion requested.")
        print("Add --home --yes to home the gantry (clear the deck first).")
        return 0

    if not args.yes:
        print("\nRefusing to move without --yes. Clear the deck, then re-run with --yes.")
        return 2

    print("\n*** MOTION: the gantry will move. Deck must be clear. ***")
    if not home(host):
        print("Home failed; not jogging.")
        return 1
    print("  homed.")

    if args.jog:
        point = [float(v) for v in args.point.split(",")]
        if jog(host, args.mount, point):
            print("  jog complete. The robot moved.")
        else:
            return 1

    print("\nPOC complete: commanded motion over the HTTP API.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
