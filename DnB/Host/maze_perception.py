from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from maze_types import CELL_SIZE_M, MAZE_SIZE, Cell, CellObservation, Direction, MatchResult, WallState


class MazePerception:
    def __init__(self, map_size: int, map_resolution: float, cell_size: float = CELL_SIZE_M, maze_size: int = MAZE_SIZE):
        self.map_size = map_size
        self.map_resolution = map_resolution
        self.cell_size = cell_size
        self.maze_size = maze_size
        self.origin_pose: Optional[Tuple[float, float, float]] = None
        self.base_heading_deg: Optional[float] = None

    def set_reference(self, x: float, y: float, theta_deg: float) -> None:
        snapped = self.quantize_angle(theta_deg)
        self.origin_pose = (x, y, snapped)
        self.base_heading_deg = snapped

    def has_reference(self) -> bool:
        return self.origin_pose is not None and self.base_heading_deg is not None

    def wrap_angle(self, theta_deg: float) -> float:
        while theta_deg > 180.0:
            theta_deg -= 360.0
        while theta_deg <= -180.0:
            theta_deg += 360.0
        return theta_deg

    def quantize_angle(self, theta_deg: float) -> float:
        return round(theta_deg / 90.0) * 90.0

    def quantize_heading(self, theta_deg: float) -> Direction:
        snapped = self.quantize_angle(theta_deg) % 360.0
        mapping = {
            0.0: Direction.EAST,
            90.0: Direction.NORTH,
            180.0: Direction.WEST,
            270.0: Direction.SOUTH,
        }
        return mapping[snapped]

    def heading_to_world(self, direction: Direction) -> float:
        if self.base_heading_deg is None:
            return 0.0
        return self.wrap_angle(self.base_heading_deg + {Direction.EAST: 0.0, Direction.NORTH: 90.0, Direction.WEST: 180.0, Direction.SOUTH: -90.0}[direction])

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        grid_x = int(self.map_size / 2 + (-world_y) / self.map_resolution)
        grid_y = int(self.map_size / 2 + world_x / self.map_resolution)
        return grid_x, grid_y

    def _rotate_local_to_world(self, local_x: float, local_y: float) -> Tuple[float, float]:
        if self.base_heading_deg is None:
            return local_x, local_y
        angle_rad = math.radians(self.base_heading_deg)
        world_x = local_x * math.cos(angle_rad) - local_y * math.sin(angle_rad)
        world_y = local_x * math.sin(angle_rad) + local_y * math.cos(angle_rad)
        return world_x, world_y

    def _rotate_world_to_local(self, dx: float, dy: float) -> Tuple[float, float]:
        if self.base_heading_deg is None:
            return dx, dy
        angle_rad = math.radians(-self.base_heading_deg)
        local_x = dx * math.cos(angle_rad) - dy * math.sin(angle_rad)
        local_y = dx * math.sin(angle_rad) + dy * math.cos(angle_rad)
        return local_x, local_y

    def world_to_cell(self, world_x: float, world_y: float) -> Cell:
        if self.origin_pose is None:
            return Cell(0, 0)
        dx = world_x - self.origin_pose[0]
        dy = world_y - self.origin_pose[1]
        local_x, local_y = self._rotate_world_to_local(dx, dy)
        col = int(round(local_x / self.cell_size))
        row = int(round(local_y / self.cell_size))
        row = max(0, min(self.maze_size - 1, row))
        col = max(0, min(self.maze_size - 1, col))
        return Cell(row, col)

    def cell_to_world_center(self, cell: Cell) -> Tuple[float, float]:
        if self.origin_pose is None:
            return 0.0, 0.0
        local_x = cell.col * self.cell_size
        local_y = cell.row * self.cell_size
        dx, dy = self._rotate_local_to_world(local_x, local_y)
        return self.origin_pose[0] + dx, self.origin_pose[1] + dy

    def snapped_pose(self, pose: Tuple[float, float, float], match_result: Optional[MatchResult] = None) -> Tuple[float, float, float, Cell, Direction]:
        x, y, theta = pose
        if match_result is not None and match_result.accepted and match_result.corrected_pose is not None:
            x, y, theta = match_result.corrected_pose
        cell = self.world_to_cell(x, y)
        center_x, center_y = self.cell_to_world_center(cell)
        snapped_theta = self.quantize_angle(theta)
        heading = self.quantize_heading(snapped_theta)
        return center_x, center_y, snapped_theta, cell, heading

    def compute_progress(self, start_pose: Tuple[float, float, float], current_pose: Tuple[float, float, float], heading_world: float) -> Tuple[float, float]:
        dx = current_pose[0] - start_pose[0]
        dy = current_pose[1] - start_pose[1]
        angle_rad = math.radians(heading_world)
        ux = math.cos(angle_rad)
        uy = math.sin(angle_rad)
        forward = dx * ux + dy * uy
        lateral = -dx * uy + dy * ux
        return forward, lateral

    def estimate_forward_clearance(self, scan_points: Iterable[Tuple[float, float, int]], heading_world: float, theta_world: float) -> float:
        if scan_points is None:
            return float("inf")
        best = float("inf")
        for angle_deg, distance_m, quality in scan_points:
            if quality < 8:
                continue
            corrected_angle = (360.0 - angle_deg) % 360.0
            relative = self.wrap_angle(corrected_angle + theta_world - heading_world)
            if abs(relative) <= 18.0:
                best = min(best, distance_m)
        return best

    def _sample_rect(self, grid_map: np.ndarray, center_world: Tuple[float, float], width: float, height: float) -> np.ndarray:
        cx, cy = center_world
        xs = np.arange(-width / 2.0, width / 2.0 + self.map_resolution, self.map_resolution)
        ys = np.arange(-height / 2.0, height / 2.0 + self.map_resolution, self.map_resolution)
        values = []
        for ox in xs:
            for oy in ys:
                wx, wy = self._rotate_local_to_world(ox, oy)
                gx, gy = self.world_to_grid(cx + wx, cy + wy)
                if 0 <= gx < self.map_size and 0 <= gy < self.map_size:
                    values.append(grid_map[gy, gx])
        if not values:
            return np.array([], dtype=float)
        return np.asarray(values, dtype=float)

    def observe_cell(self, grid_map: np.ndarray, pose: Tuple[float, float, float], match_result: Optional[MatchResult] = None, scan_points: Optional[Iterable[Tuple[float, float, int]]] = None) -> CellObservation:
        snapped_x, snapped_y, snapped_theta, cell, heading = self.snapped_pose(pose, match_result)
        center_x, center_y = self.cell_to_world_center(cell)
        center_error = math.hypot(pose[0] - center_x, pose[1] - center_y)
        obs = CellObservation(center_error=center_error, match_score=match_result.score if match_result else 0.0)

        strip_half = self.cell_size / 2.0
        strip_depth = 0.10
        strip_span = 0.42
        corridor_depth = 0.20
        occ_threshold = 8.0
        free_threshold = -3.0

        offsets = {
            Direction.NORTH: (0.0, strip_half),
            Direction.EAST: (strip_half, 0.0),
            Direction.SOUTH: (0.0, -strip_half),
            Direction.WEST: (-strip_half, 0.0),
        }

        for direction, (ox, oy) in offsets.items():
            wall_center_local_x = cell.col * self.cell_size + ox
            wall_center_local_y = cell.row * self.cell_size + oy
            world_dx, world_dy = self._rotate_local_to_world(wall_center_local_x, wall_center_local_y)
            wall_center_world = (self.origin_pose[0] + world_dx, self.origin_pose[1] + world_dy) if self.origin_pose else (0.0, 0.0)
            if direction in (Direction.NORTH, Direction.SOUTH):
                wall_values = self._sample_rect(grid_map, wall_center_world, strip_span, strip_depth)
            else:
                wall_values = self._sample_rect(grid_map, wall_center_world, strip_depth, strip_span)

            corridor_local_x = cell.col * self.cell_size + ox * 1.35
            corridor_local_y = cell.row * self.cell_size + oy * 1.35
            corridor_dx, corridor_dy = self._rotate_local_to_world(corridor_local_x, corridor_local_y)
            corridor_center = (self.origin_pose[0] + corridor_dx, self.origin_pose[1] + corridor_dy) if self.origin_pose else (0.0, 0.0)
            if direction in (Direction.NORTH, Direction.SOUTH):
                corridor_values = self._sample_rect(grid_map, corridor_center, strip_span * 0.8, corridor_depth)
            else:
                corridor_values = self._sample_rect(grid_map, corridor_center, corridor_depth, strip_span * 0.8)

            occ_ratio = float(np.mean(wall_values > occ_threshold)) if wall_values.size else 0.0
            free_ratio = float(np.mean(corridor_values < free_threshold)) if corridor_values.size else 0.0
            confidence = max(occ_ratio, free_ratio)
            if occ_ratio >= 0.35:
                state = WallState.BLOCKED
            elif free_ratio >= 0.35:
                state = WallState.OPEN
            else:
                state = WallState.UNKNOWN
            obs.walls[direction] = state
            obs.confidences[direction] = confidence

        obs.forward_clearance = self.estimate_forward_clearance(scan_points or [], self.heading_to_world(heading), pose[2])
        obs.lateral_error = 0.0
        return obs
