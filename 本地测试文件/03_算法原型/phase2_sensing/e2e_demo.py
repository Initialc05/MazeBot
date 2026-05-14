"""
End-to-end: robot navigates the Z-island maze using ONLY simulated lidar
scans + wall sensing, no god-view.

This is the proof that Phase 1's Flood-Fill algorithm is happy to run on
inferred walls (with noise!) instead of perfect ground truth. When this
works, switching to a real RPLIDAR just means replacing simulate_scan()
with a function that reads from the bluetooth stream.

Run:
    python e2e_demo.py                    # default Z-island, noisy lidar
    python e2e_demo.py --clean            # no noise
    python e2e_demo.py --stress           # 3cm noise + 20% dropout
"""
from __future__ import annotations
import sys
import os
import copy

# Add phase 1 to path
_PHASE1_DIR = os.path.join(os.path.dirname(__file__), '..', 'flood_fill_sim')
if _PHASE1_DIR not in sys.path:
    sys.path.insert(0, _PHASE1_DIR)

from maze import Maze, MAZES, DIR_NAMES, DIRS, DX, DY, N, E, S, W, build_z_island_maze
from flood_fill import flood_fill, choose_next_move, INF
from fake_lidar import simulate_scan, LidarConfig
from wall_sense import sense_walls, update_belief


# Heading convention in the Simulator: 0=N, 1=E, 2=S, 3=W (discrete direction index)
# Need to convert to degrees for lidar/sense
HEADING_TO_DEG = {N: 0, E: 270, S: 180, W: 90}
# (heading=N → 0°; heading=W → 90° in CCW world convention)


def run_episode(truth: Maze,
                start: tuple[int, int],
                goal: tuple[int, int],
                lidar_config: LidarConfig,
                turn_penalty: int = 2,
                max_steps: int = 300,
                verbose: bool = True) -> dict:
    """
    One full exploration run using simulated lidar + wall_sense.
    Returns stats dict.
    """
    belief = Maze(truth.rows, truth.cols)  # start: only outer walls
    pos = start
    heading = N
    steps, turns = 0, 0
    sense_errors = 0  # cells where sensed walls != truth (for reporting only)

    for t in range(max_steps):
        # 1. Sense — replace god-view with lidar-based sensing
        scan = simulate_scan(truth, pos, heading_deg=HEADING_TO_DEG[heading],
                             config=lidar_config)
        sensed = sense_walls(scan, heading_deg=HEADING_TO_DEG[heading])
        # Track sensing accuracy for reporting
        r, c = pos
        truth_walls = {d: truth.has_wall(r, c, d) for d in (N, E, S, W)}
        if any(sensed[d] != truth_walls[d] for d in (N, E, S, W) if sensed[d] is not None):
            sense_errors += 1
        update_belief(belief, pos, sensed)

        # 2. Plan
        dist = flood_fill(belief, goal)
        if pos == goal:
            if verbose:
                print(f"  step {t:3d}: at goal! done.")
            break

        # 3. Decide
        next_dir = choose_next_move(belief, pos, dist, heading, turn_penalty)
        if next_dir is None:
            if verbose:
                print(f"  step {t:3d}: stuck at {pos}!")
            break

        # 4. Act — teleport one cell (Phase 3 will replace with real motion)
        if next_dir != heading:
            turns += 1
        old_pos = pos
        pos = (pos[0] + DY[next_dir], pos[1] + DX[next_dir])
        heading = next_dir
        steps += 1
        if verbose:
            print(f"  step {t:3d}: {old_pos} ({DIR_NAMES[heading]}) "
                  f"-> {pos}  walls_sensed={''.join(DIR_NAMES[d] for d in (N,E,S,W) if sensed[d])}")

    return {
        'steps': steps,
        'turns': turns,
        'reached_goal': pos == goal,
        'sense_errors': sense_errors,
        'belief': belief,
    }


def main():
    args = sys.argv[1:]
    verbose = '-v' in args or '--verbose' in args
    if '--clean' in args:
        cfg = LidarConfig(seed=42, distance_noise_std=0, dropout_probability=0,
                          near_distortion_factor=1.0)
        label = 'clean'
    elif '--stress' in args:
        cfg = LidarConfig(seed=42, distance_noise_std=0.03, dropout_probability=0.20,
                          near_distortion_factor=0.70, near_distortion_threshold=0.20)
        label = 'stress (σ=3cm, 20% drop)'
    else:
        cfg = LidarConfig(seed=42, distance_noise_std=0.01, dropout_probability=0.05,
                          near_distortion_factor=0.85, near_distortion_threshold=0.15)
        label = 'realistic (σ=1cm, 5% drop, 15% near distortion)'

    truth = build_z_island_maze()
    print(f"=== End-to-end: z_island_5x5 maze ===")
    print(f"Start=(4,0) Goal=(2,4) Lidar config: {label}")
    print()
    if verbose:
        print("Step-by-step:")
    result = run_episode(truth, (4, 0), (2, 4), cfg, verbose=verbose)
    print()
    print(f"Reached goal: {result['reached_goal']}")
    print(f"Total steps:  {result['steps']}")
    print(f"Total turns:  {result['turns']}")
    print(f"Cells with any sensing error: {result['sense_errors']}")


if __name__ == '__main__':
    main()
