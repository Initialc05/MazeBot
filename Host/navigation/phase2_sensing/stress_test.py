"""
Stress test: how robust is wall_sense to real-world lidar problems?

Tests (on Z-island maze, 25 cells):
    1. Baseline (σ=0, no dropout, no distortion)
    2. Noise only (σ=1cm, 2cm, 5cm)
    3. Dropout only (5%, 20%, 50%)
    4. Near-distance distortion (factor 0.85, 0.7)
    5. Combined real-world (σ=1cm + 5% dropout + distortion=0.85)
"""
from __future__ import annotations
import sys
import os
_PHASE1_DIR = os.path.join(os.path.dirname(__file__), '..', 'flood_fill_sim')
if _PHASE1_DIR not in sys.path:
    sys.path.insert(0, _PHASE1_DIR)

from maze import build_z_island_maze, N, E, S, W
from fake_lidar import simulate_scan, LidarConfig
from wall_sense import sense_walls


def test_accuracy(truth, config, n_trials: int = 5) -> float:
    """Run sense on every cell `n_trials` times with different seeds, average."""
    total, matches = 0, 0
    for trial in range(n_trials):
        cfg = LidarConfig(
            seed=1000 + trial,
            distance_noise_std=config.distance_noise_std,
            dropout_probability=config.dropout_probability,
            near_distortion_factor=config.near_distortion_factor,
            near_distortion_threshold=config.near_distortion_threshold,
        )
        for r in range(truth.rows):
            for c in range(truth.cols):
                scan = simulate_scan(truth, (r, c), heading_deg=0, config=cfg)
                sensed = sense_walls(scan, heading_deg=0)
                true_walls = {d: truth.has_wall(r, c, d) for d in (N, E, S, W)}
                if all(sensed[d] == true_walls[d] for d in (N, E, S, W)):
                    matches += 1
                total += 1
    return matches / total * 100


if __name__ == '__main__':
    truth = build_z_island_maze()
    print(f"{'scenario':<50s} {'accuracy':>10s}")
    print('-' * 62)

    # Baseline
    cfg = LidarConfig(distance_noise_std=0, dropout_probability=0, near_distortion_factor=1.0)
    print(f"{'1. baseline (clean)':<50s} {test_accuracy(truth, cfg):>9.1f}%")

    # Noise tests
    for sigma in [0.005, 0.01, 0.02, 0.05]:
        cfg = LidarConfig(distance_noise_std=sigma, dropout_probability=0, near_distortion_factor=1.0)
        print(f"{f'2. noise only  σ={sigma*100:.1f}cm':<50s} {test_accuracy(truth, cfg):>9.1f}%")

    # Dropout tests
    for p in [0.05, 0.20, 0.50]:
        cfg = LidarConfig(distance_noise_std=0, dropout_probability=p, near_distortion_factor=1.0)
        print(f"{f'3. dropout only {int(p*100)}%':<50s} {test_accuracy(truth, cfg):>9.1f}%")

    # Near-distance distortion
    for factor in [0.85, 0.70]:
        cfg = LidarConfig(distance_noise_std=0, dropout_probability=0,
                          near_distortion_factor=factor, near_distortion_threshold=0.20)
        print(f"{f'4. near distortion factor={factor}':<50s} {test_accuracy(truth, cfg):>9.1f}%")

    # Combined real-world scenario
    cfg = LidarConfig(distance_noise_std=0.01, dropout_probability=0.05,
                      near_distortion_factor=0.85, near_distortion_threshold=0.15)
    print(f"{'5. realistic (σ=1cm, 5% drop, 15% near)':<50s} {test_accuracy(truth, cfg):>9.1f}%")

    # Stress: ugly conditions
    cfg = LidarConfig(distance_noise_std=0.03, dropout_probability=0.20,
                      near_distortion_factor=0.70, near_distortion_threshold=0.20)
    print(f"{'6. stress (σ=3cm, 20% drop, 30% near)':<50s} {test_accuracy(truth, cfg):>9.1f}%")
