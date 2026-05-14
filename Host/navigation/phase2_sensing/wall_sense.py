"""
Wall sensing: from a lidar scan + robot heading, infer whether walls
exist in the 4 cardinal directions of the current cell.

Approach (deliberately simple — we want robust over precise):
    For each of 4 cardinal directions (relative to robot heading):
        Take the beams within ±SECTOR_HALFWIDTH degrees of that direction
        Compute the median distance among those beams
        If median < WALL_THRESHOLD → wall present
        Else → no wall

Why median (not mean):
    - Robust to lidar dropouts (bad beams report MAX_RANGE)
    - Robust to noise spikes
    - Resistant to one beam grazing through a doorway

Why a sector (not single beam):
    - The 1° beam in the dead-center may hit a thin gap
    - Averaging a small wedge gives a stable wall reading
    - But too wide and you mix with adjacent walls

The output is in MAZE FRAME (N/E/S/W absolute), not robot frame. So if the
robot is heading East and the front sector reads "wall close", we report
"East wall present" — Flood-Fill cares about absolute walls.
"""
from __future__ import annotations
import math
import numpy as np

# Match flood_fill_sim/maze.py
N, E, S, W = 0, 1, 2, 3

# Default thresholds (tunable)
SECTOR_HALFWIDTH_DEG = 30   # ±30° wedge around each cardinal beam
DEFAULT_CELL_SIZE = 0.70    # 70 cm per cell (real maze)
# Wall threshold = half a cell + slack. 70 cm cell → 35 cm to wall, + slack.
DEFAULT_WALL_THRESHOLD = 0.50


def _normalize_angle(deg: float) -> float:
    """Wrap angle to [0, 360)."""
    return deg % 360


def _angle_in_sector(angle: float, center: float, half_width: float) -> bool:
    """Is `angle` within ±half_width of `center`? Handles wrap-around."""
    diff = (angle - center + 540) % 360 - 180  # in [-180, 180]
    return abs(diff) <= half_width


def sense_walls(scan: list[tuple[float, float, int]],
                heading_deg: float = 0.0,
                wall_threshold: float = DEFAULT_WALL_THRESHOLD,
                sector_halfwidth: float = SECTOR_HALFWIDTH_DEG,
                min_beams_per_sector: int = 5) -> dict:
    """
    Infer N/E/S/W wall presence from a lidar scan.

    Args:
        scan: list of (angle_deg, distance_m, quality). Angle is in robot
              frame, 0° = forward.
        heading_deg: robot's heading in world frame, 0° = facing N (+y).
              Increasing CCW. So heading=0 → robot forward = N;
              heading=90 → robot forward = W (+x is east, +y is north,
              so CCW from N to W is +90° in math convention).
        wall_threshold: distance below which we conclude "wall is here"
        sector_halfwidth: half-width of the angular wedge per direction
        min_beams_per_sector: if a sector has fewer beams than this
              (due to dropouts), treat result as unknown (None)

    Returns:
        dict {N: True/False/None, E: ..., S: ..., W: ...}
        True = wall present, False = no wall, None = unknown (insufficient data)

    World frame convention:
        N → +y, E → +x, S → -y, W → -x
        In the robot's frame, "forward" is the heading direction.
        So "where to look in the scan" depends on heading:
            World-N is at (0 - heading) deg in robot frame
            World-E is at (-90 - heading) deg in robot frame
            World-S is at (180 - heading) deg in robot frame
            World-W is at (90 - heading) deg in robot frame
    """
    # Map of world direction → angle to look at in scan (robot frame)
    # World angle (math): N=90°, E=0°, S=270°, W=180°
    # But our scan is in robot frame: 0° = forward = robot's heading.
    # So scan_angle = (world_angle - heading_deg) mod 360.
    # And our heading_deg uses N=0 (lidar/IMU convention), CCW positive.
    # In our heading convention:
    #   heading=0   → robot forward = N → world-N is at scan angle 0
    #   heading=90  → robot forward = W → world-N is at scan angle -90 = 270
    #   heading=180 → robot forward = S → world-N is at scan angle 180
    # So scan_angle_for_N = -heading_deg (mod 360).
    # Then E is 90° clockwise from N in world space → -90° from N in robot space:
    #   scan_angle_for_E = scan_angle_for_N - 90
    # (because robot frame rotates CCW from forward, so to look at world-E from
    #  forward=N you turn 90° CW = -90°)
    scan_angle_for = {
        N: _normalize_angle(-heading_deg),
        E: _normalize_angle(-heading_deg - 90),
        S: _normalize_angle(-heading_deg - 180),
        W: _normalize_angle(-heading_deg - 270),  # equiv to -heading + 90
    }

    result = {}
    for direction in (N, E, S, W):
        center = scan_angle_for[direction]
        beams_in_sector = [d for (a, d, _q) in scan
                           if _angle_in_sector(a, center, sector_halfwidth)]
        if len(beams_in_sector) < min_beams_per_sector:
            result[direction] = None  # unknown — too few beams
            continue
        median_dist = float(np.median(beams_in_sector))
        result[direction] = (median_dist < wall_threshold)
    return result


def update_belief(belief, cell, walls_dict):
    """
    Apply the sense_walls() result into a belief Maze.
    Only sets True walls (we don't ever delete walls — sensing once is
    enough to confirm, but a single missed beam shouldn't unset a wall).
    """
    r, c = cell
    for d, present in walls_dict.items():
        if present:
            belief.set_wall(r, c, d, present=True)


if __name__ == '__main__':
    # Smoke test against fake lidar
    import sys, os
    _PHASE1_DIR = os.path.join(os.path.dirname(__file__), '..', 'flood_fill_sim')
    if _PHASE1_DIR not in sys.path:
        sys.path.insert(0, _PHASE1_DIR)
    from maze import build_z_island_maze, DIR_NAMES
    from fake_lidar import simulate_scan, LidarConfig

    truth = build_z_island_maze()

    # Sense walls in every cell, compare to ground truth
    print("Wall sensing test on z_island_5x5:")
    print(f"{'cell':<8s} {'true':<10s} {'sensed':<10s} {'match':<6s}")
    print('-' * 40)
    matches = 0
    total = 0
    cfg = LidarConfig(seed=42, distance_noise_std=0.005)
    for r in range(truth.rows):
        for c in range(truth.cols):
            scan = simulate_scan(truth, (r, c), heading_deg=0, config=cfg)
            sensed = sense_walls(scan, heading_deg=0)
            true_walls = {d: truth.has_wall(r, c, d) for d in (N, E, S, W)}
            match = all(sensed[d] == true_walls[d] for d in (N, E, S, W))
            true_str = ''.join(DIR_NAMES[d] for d in (N, E, S, W) if true_walls[d])
            sensed_str = ''.join(DIR_NAMES[d] for d in (N, E, S, W) if sensed[d])
            print(f"({r},{c})    {true_str or '-':<10s} {sensed_str or '-':<10s} {'OK' if match else 'MISMATCH'}")
            if match:
                matches += 1
            total += 1
    print(f"\nAccuracy: {matches}/{total} = {matches / total * 100:.1f}%")
