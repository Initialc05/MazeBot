from __future__ import annotations

import time
from typing import Callable, Iterable, Optional, Tuple

from maze_perception import MazePerception
from maze_types import CELL_SIZE_M, MatchResult, MotionMode, MotionStatus


class MazeController:
    def __init__(self, send_command: Callable[[str], None], perception: MazePerception):
        self.send_command = send_command
        self.perception = perception
        self.status = MotionStatus()
        self.start_pose: Optional[Tuple[float, float, float]] = None
        self.target_heading_world: Optional[float] = None
        self.target_distance: float = CELL_SIZE_M
        self.brake_margin: float = 0.10
        self.max_lateral_error: float = 0.16
        self.min_front_clearance: float = 0.22
        self.deadline: float = 0.0

    def stop(self, reason: str = "stop") -> MotionStatus:
        self.send_command('x')
        self.status.mode = MotionMode.IDLE
        self.status.command = 'x'
        self.status.done = True
        self.status.reason = reason
        self.status.success = reason in {"goal_window", "heading_aligned", "stopped"}
        return self.status

    def start_spin(self, current_pose: Tuple[float, float, float], target_heading_world: float, turn_cmd: str, timeout: float = 4.0) -> None:
        self.start_pose = current_pose
        self.target_heading_world = target_heading_world
        self.target_distance = 0.0
        self.deadline = time.time() + timeout
        self.status = MotionStatus(mode=MotionMode.SPIN, command=turn_cmd, done=False, success=False, reason="spinning")
        self.send_command(turn_cmd)

    def start_drive(self, current_pose: Tuple[float, float, float], target_heading_world: float, distance: float = CELL_SIZE_M, timeout: float = 4.0) -> None:
        self.start_pose = current_pose
        self.target_heading_world = target_heading_world
        self.target_distance = distance
        self.deadline = time.time() + timeout
        self.status = MotionStatus(mode=MotionMode.DRIVE, command='W', done=False, success=False, reason="driving")
        self.send_command('W')

    def update(self, current_pose: Tuple[float, float, float], match_result: Optional[MatchResult] = None, scan_points: Optional[Iterable[Tuple[float, float, int]]] = None) -> MotionStatus:
        if self.status.mode == MotionMode.IDLE:
            return self.status
        if time.time() > self.deadline:
            return self.stop("timeout")
        if self.status.mode == MotionMode.SPIN:
            heading_error = self.perception.wrap_angle((self.target_heading_world or 0.0) - current_pose[2])
            if abs(heading_error) <= 8.0:
                return self.stop("heading_aligned")
            return self.status
        if self.start_pose is None:
            return self.stop("missing_start_pose")
        forward, lateral = self.perception.compute_progress(self.start_pose, current_pose, self.target_heading_world or current_pose[2])
        clearance = self.perception.estimate_forward_clearance(scan_points or [], self.target_heading_world or current_pose[2], current_pose[2])
        if clearance < self.min_front_clearance:
            return self.stop("front_clearance")
        if abs(lateral) > self.max_lateral_error:
            return self.stop("lateral_error")
        if forward >= max(0.10, self.target_distance - self.brake_margin):
            return self.stop("goal_window")
        return self.status
