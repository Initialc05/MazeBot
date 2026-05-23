from __future__ import annotations

from typing import Iterable, Tuple

import numpy as np

from maze_types import MatchResult


class MazeMatcher:
    def __init__(self, map_size: int, map_resolution: float):
        self.map_size = map_size
        self.map_resolution = map_resolution
        self.dx_candidates = np.arange(-0.10, 0.1001, 0.05)
        self.dy_candidates = np.arange(-0.10, 0.1001, 0.05)
        self.dtheta_candidates = np.arange(-8.0, 8.01, 2.0)
        self.accept_score = 60.0

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        grid_x = int(self.map_size / 2 + (-y) / self.map_resolution)
        grid_y = int(self.map_size / 2 + x / self.map_resolution)
        return grid_x, grid_y

    def score_pose(self, grid_map: np.ndarray, scan_points: Iterable[Tuple[float, float, int]], pose: Tuple[float, float, float]) -> float:
        x, y, theta = pose
        score = 0.0
        count = 0
        for angle_deg, distance_m, quality in scan_points:
            if quality < 10 or distance_m < 0.20 or distance_m > 1.5:
                continue
            corrected_angle = (360.0 - angle_deg) % 360.0
            world_angle = np.deg2rad(corrected_angle + theta)
            px = x + distance_m * np.cos(world_angle)
            py = y + distance_m * np.sin(world_angle)
            gx, gy = self.world_to_grid(px, py)
            if 0 <= gx < self.map_size and 0 <= gy < self.map_size:
                y0 = max(0, gy - 1)
                y1 = min(self.map_size, gy + 2)
                x0 = max(0, gx - 1)
                x1 = min(self.map_size, gx + 2)
                local = grid_map[y0:y1, x0:x1]
                score += float(np.max(local))
                count += 1
        if count == 0:
            return 0.0
        return score / count

    def match_scan(self, grid_map: np.ndarray, scan_points: Iterable[Tuple[float, float, int]], predicted_pose: Tuple[float, float, float]) -> MatchResult:
        if scan_points is None:
            return MatchResult(corrected_pose=predicted_pose)
        points = list(scan_points)
        if len(points) < 20:
            return MatchResult(corrected_pose=predicted_pose)
        base_score = self.score_pose(grid_map, points, predicted_pose)
        best_score = base_score
        best_pose = predicted_pose
        best_delta = (0.0, 0.0, 0.0)
        for dx in self.dx_candidates:
            for dy in self.dy_candidates:
                for dtheta in self.dtheta_candidates:
                    candidate = (predicted_pose[0] + float(dx), predicted_pose[1] + float(dy), predicted_pose[2] + float(dtheta))
                    score = self.score_pose(grid_map, points, candidate)
                    if score > best_score:
                        best_score = score
                        best_pose = candidate
                        best_delta = (float(dx), float(dy), float(dtheta))
        accepted = best_score >= self.accept_score or best_score >= base_score + 5.0
        return MatchResult(
            dx=best_delta[0],
            dy=best_delta[1],
            dtheta=best_delta[2],
            score=best_score,
            accepted=accepted,
            corrected_pose=best_pose if accepted else predicted_pose,
        )
