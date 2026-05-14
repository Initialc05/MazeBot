"""
Phase 3a — Single-cell wall sensing test on the REAL robot.

What it does:
    1. Open bluetooth serial port (default COM4 @ 921600, configurable).
    2. Collect one full 360° lidar scan from the real firmware.
    3. Feed it to wall_sense.sense_walls() — the exact algorithm validated
       in Phase 2 simulation.
    4. Print the sensed walls AND the raw median distance per direction,
       so you can eyeball whether the threshold is right.

Usage on the real maze:
    python sense_one_cell.py                  # COM4, heading=0 (facing north/+y)
    python sense_one_cell.py --port COM3
    python sense_one_cell.py --heading 90     # if the car isn't facing +y
    python sense_one_cell.py --threshold 0.40 # override wall_sense threshold
    python sense_one_cell.py --loop           # keep reading scans forever

Expected output (car placed inside a cell, facing north):
    N: wall=True  median_dist=0.27 m
    E: wall=False median_dist=0.72 m
    S: wall=True  median_dist=0.31 m
    W: wall=False median_dist=0.68 m

Pre-flight checklist for tomorrow:
    - Close lidar_visualizer.py (only one process can hold the COM port)
    - Power car on, make sure bluetooth pairs and data is flowing
    - Place car roughly centered in one maze cell, oriented close to a
      cardinal heading (no need to be precise, algorithm is ±30°-tolerant)
"""
from __future__ import annotations
import sys
import time
import argparse
import os

# Make phase 1 & phase 2 importable
_THIS_DIR = os.path.dirname(__file__)
_PHASE1 = os.path.join(_THIS_DIR, '..', 'flood_fill_sim')
_PHASE2 = os.path.join(_THIS_DIR, '..', 'phase2_sensing')
for p in (_PHASE1, _PHASE2):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
import serial  # pyserial

from bt_protocol import read_full_scan
from wall_sense import sense_walls, SECTOR_HALFWIDTH_DEG, DEFAULT_WALL_THRESHOLD
from maze import N, E, S, W, DIR_NAMES


def _median_per_sector(scan, heading_deg, half_width):
    """Compute median distance per cardinal, same geometry as sense_walls."""
    # Mirrors scan_angle_for in wall_sense.py
    scan_for = {
        N: (-heading_deg) % 360,
        E: (-heading_deg - 90) % 360,
        S: (-heading_deg - 180) % 360,
        W: (-heading_deg - 270) % 360,
    }
    medians = {}
    counts = {}
    for d, center in scan_for.items():
        beams = []
        for a, dist, q in scan:
            diff = (a - center + 540) % 360 - 180
            if abs(diff) <= half_width:
                beams.append(dist)
        medians[d] = float(np.median(beams)) if beams else None
        counts[d] = len(beams)
    return medians, counts


def main():
    p = argparse.ArgumentParser(description='Sense walls of one maze cell from real lidar.')
    p.add_argument('--port', default='COM4', help='Serial port (default COM4)')
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--heading', type=float, default=0.0,
                   help='Car heading in degrees, 0=facing +y(N), CCW positive. Default 0.')
    p.add_argument('--threshold', type=float, default=DEFAULT_WALL_THRESHOLD,
                   help=f'Wall distance threshold in meters (default {DEFAULT_WALL_THRESHOLD})')
    p.add_argument('--loop', action='store_true', help='Keep reading scans forever')
    p.add_argument('--timeout', type=float, default=3.0, help='Scan timeout in seconds')
    args = p.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        print("  - Is lidar_visualizer.py running? It holds the COM port.")
        print("  - Is the car on and bluetooth paired?")
        sys.exit(1)

    print(f"Connected. Assuming car heading = {args.heading}° (0 = facing north/+y).")
    print(f"Wall threshold = {args.threshold} m, sector half-width = {SECTOR_HALFWIDTH_DEG}°.")
    print()

    try:
        n = 0
        while True:
            n += 1
            print(f"--- scan {n} ---")
            t0 = time.time()
            scan, odom = read_full_scan(ser, timeout_s=args.timeout)
            elapsed = time.time() - t0
            if scan is None:
                print(f"  no scan within {args.timeout}s — is the car streaming?")
                if not args.loop:
                    break
                continue

            print(f"  got {len(scan)} beams in {elapsed:.2f}s")
            if odom:
                print(f"  odom: x={odom.x_m:+.2f}m y={odom.y_m:+.2f}m θ={odom.theta_deg:+.1f}°")

            sensed = sense_walls(scan, heading_deg=args.heading,
                                 wall_threshold=args.threshold)
            medians, counts = _median_per_sector(scan, args.heading, SECTOR_HALFWIDTH_DEG)

            print(f"  {'dir':<4s} {'wall?':<6s} {'median':>10s} {'beams':>7s}")
            for d in (N, E, S, W):
                m = medians[d]
                m_str = f"{m:.3f} m" if m is not None else "  -  "
                wall = sensed[d]
                wall_str = "YES" if wall is True else ("no " if wall is False else " ? ")
                print(f"  {DIR_NAMES[d]:<4s} {wall_str:<6s} {m_str:>10s} {counts[d]:>7d}")

            sensed_str = ''.join(DIR_NAMES[d] for d in (N, E, S, W) if sensed[d])
            print(f"  summary: walls = [{sensed_str or 'none'}]")
            print()

            if not args.loop:
                break
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n(interrupted)")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
