"""
Simulated lidar scanner.

Given a Maze (with walls), the robot's current grid cell, sub-cell offset,
and heading angle, produce a 360-point lidar scan (angle, distance) that
approximates what the real RPLIDAR C1 would read.

Design notes:
- We ray-march from the robot position into the maze. For each 1-degree
  beam, walk outward until we cross a wall segment in the maze, return
  distance in meters.
- Cell size defaults to 0.30 m (what we expect for the real maze).
- Heading convention matches maze.py: 0=N(+y), 1=E(+x), 2=S(-y), 3=W(-x),
  but we also accept continuous heading in degrees (0° = north).
- Optional noise models:
    * Gaussian distance noise (~1 cm stdev — realistic for C1)
    * Near-distance distortion (points within 15 cm report ~15% shorter,
      matching user's observation "straight walls bend into arcs up close")
    * Angular jitter + occasional dropouts

If you switch this to real scans later, keep the output format identical:
    list of (angle_deg, distance_m, quality) tuples.
"""
from __future__ import annotations
import math
import numpy as np
from dataclasses import dataclass

# Import maze types — add parent dir to path so we can reuse phase 1's maze.py
import sys
import os
_PHASE1_DIR = os.path.join(os.path.dirname(__file__), '..', 'flood_fill_sim')
if _PHASE1_DIR not in sys.path:
    sys.path.insert(0, _PHASE1_DIR)
from maze import Maze, N, E, S, W, DX, DY  # noqa: E402


CELL_SIZE_M = 0.70  # 70 cm per cell — real competition maze, measured
MAX_RANGE_M = 2.5   # beams that don't hit anything in this range return max


@dataclass
class LidarConfig:
    cell_size: float = CELL_SIZE_M
    max_range: float = MAX_RANGE_M
    num_beams: int = 360               # one beam per degree
    distance_noise_std: float = 0.01   # meters, per-beam Gaussian
    near_distortion_threshold: float = 0.15  # m; below this, distortion kicks in
    near_distortion_factor: float = 0.85     # multiply distance by this when close
    dropout_probability: float = 0.0         # fraction of beams randomly discarded
    seed: int | None = None


def cell_to_world(cell: tuple[int, int], offset: tuple[float, float] = (0.5, 0.5),
                  cell_size: float = CELL_SIZE_M) -> tuple[float, float]:
    """
    Convert (row, col) + fractional offset within cell to world (x, y) meters.
    offset=(0.5, 0.5) = cell center.
    """
    r, c = cell
    ox, oy = offset
    x = (c + ox) * cell_size
    y = (r + oy) * cell_size
    return x, y


def _wall_segments(maze: Maze, cell_size: float = CELL_SIZE_M):
    """
    Yield every wall as a line segment ((x0, y0), (x1, y1)) in world coords.
    Walls are deduplicated (each wall is yielded exactly once even though it
    belongs to two cells).
    """
    for r in range(maze.rows):
        for c in range(maze.cols):
            x0, y0 = c * cell_size, r * cell_size
            # Only yield N and E walls for this cell; S of (r+1) and W of (c+1)
            # would duplicate. Edge cells also emit their S/W outer walls.
            if maze.has_wall(r, c, N):
                yield (x0, y0 + cell_size), (x0 + cell_size, y0 + cell_size)
            if maze.has_wall(r, c, E):
                yield (x0 + cell_size, y0), (x0 + cell_size, y0 + cell_size)
            if r == 0 and maze.has_wall(r, c, S):
                yield (x0, y0), (x0 + cell_size, y0)
            if c == 0 and maze.has_wall(r, c, W):
                yield (x0, y0), (x0, y0 + cell_size)


def _ray_segment_hit(origin, direction, segments, max_t):
    """
    Find the closest positive intersection distance of a ray with any segment.
    origin: (x, y); direction: (dx, dy) unit vector; segments: iterable of
    ((x0,y0),(x1,y1)). Returns distance or max_t if nothing hit within.
    """
    ox, oy = origin
    dx, dy = direction
    best_t = max_t
    for (x0, y0), (x1, y1) in segments:
        # Parametric: origin + t*dir = seg_start + s*(seg_end - seg_start)
        sx, sy = x1 - x0, y1 - y0
        denom = dx * sy - dy * sx
        if abs(denom) < 1e-12:
            continue  # parallel
        t = ((x0 - ox) * sy - (y0 - oy) * sx) / denom
        s = ((x0 - ox) * dy - (y0 - oy) * dx) / denom
        if t > 0 and 0.0 <= s <= 1.0 and t < best_t:
            best_t = t
    return best_t


def simulate_scan(maze: Maze,
                  cell: tuple[int, int],
                  heading_deg: float = 0.0,
                  offset: tuple[float, float] = (0.5, 0.5),
                  config: LidarConfig | None = None) -> list[tuple[float, float, int]]:
    """
    Produce a simulated lidar scan.

    Args:
        maze: the ground truth maze
        cell: (row, col) robot is in
        heading_deg: robot heading, 0 = facing +Y (north), increasing CCW
        offset: sub-cell offset, (0.5, 0.5) = cell center, (0.0, 0.5) = west wall
        config: LidarConfig, or None for defaults

    Returns:
        list of (lidar_angle_deg, distance_m, quality). Angle is in the robot's
        frame: 0° = straight ahead, increasing CCW to 360°. Missing beams are
        dropped (not returned) when dropout_probability > 0.
    """
    if config is None:
        config = LidarConfig()
    rng = np.random.default_rng(config.seed)

    ox, oy = cell_to_world(cell, offset, config.cell_size)
    segments = list(_wall_segments(maze, config.cell_size))
    scan = []

    heading_rad = math.radians(heading_deg)
    # Convention: heading=0 means the robot is facing NORTH (+y in world),
    # and scan angle 0 means "straight ahead". So when heading=0, scan angle 0
    # points at world angle 90° (+y). We add π/2 to align conventions.
    heading_rad += math.pi / 2
    for beam_i in range(config.num_beams):
        # Angle in robot frame (0 = forward, CCW positive)
        beam_angle_rel = 2 * math.pi * beam_i / config.num_beams
        # World frame angle
        world_angle = heading_rad + beam_angle_rel
        dx = math.cos(world_angle)
        dy = math.sin(world_angle)

        dist = _ray_segment_hit((ox, oy), (dx, dy), segments, config.max_range)

        # Near-distance distortion
        if dist < config.near_distortion_threshold:
            dist = dist * config.near_distortion_factor

        # Gaussian noise
        if config.distance_noise_std > 0:
            dist = dist + rng.normal(0, config.distance_noise_std)
            dist = max(0.01, dist)  # never return negative or zero

        # Occasional dropout
        if config.dropout_probability > 0 and rng.random() < config.dropout_probability:
            continue

        angle_deg = math.degrees(beam_angle_rel)
        quality = 15  # arbitrary, matches "good quality" flag on C1
        scan.append((angle_deg, dist, quality))

    return scan


def scan_to_xy(scan: list[tuple[float, float, int]]) -> np.ndarray:
    """
    Convert scan to Nx2 array of (x, y) points in the robot's frame.
    Useful for plotting.
    """
    pts = []
    for angle_deg, dist, _q in scan:
        a = math.radians(angle_deg)
        pts.append((dist * math.cos(a), dist * math.sin(a)))
    return np.array(pts)


if __name__ == '__main__':
    from maze import build_z_island_maze
    import matplotlib.pyplot as plt

    m = build_z_island_maze()
    # Put robot in (2,2) — middle-ish of the maze — facing east (heading 0 in our
    # convention means +y/north; east = -90°)
    scan_clean = simulate_scan(m, (2, 2), heading_deg=0,
                               config=LidarConfig(seed=42, distance_noise_std=0))
    scan_noisy = simulate_scan(m, (2, 2), heading_deg=0,
                               config=LidarConfig(seed=42, distance_noise_std=0.01))

    print(f"Clean scan: {len(scan_clean)} beams")
    print(f"Noisy scan: {len(scan_noisy)} beams")
    print(f"First 5 clean beams (angle_deg, dist_m):")
    for a, d, _ in scan_clean[:5]:
        print(f"  angle={a:6.1f}°  dist={d:.3f} m")

    # Visualize
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
    for ax, scan, title in [(ax1, scan_clean, 'clean'), (ax2, scan_noisy, 'noisy σ=1cm')]:
        pts = scan_to_xy(scan)
        ax.scatter(pts[:, 0], pts[:, 1], s=3, c='b')
        ax.plot(0, 0, 'r^', markersize=12)  # robot
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'Fake scan @ cell (2,2), {title}')
        ax.axhline(0, color='gray', lw=0.5)
        ax.axvline(0, color='gray', lw=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(__file__), 'fake_scan_preview.png'),
                dpi=100)
    print(f"\nPreview saved to fake_scan_preview.png")
