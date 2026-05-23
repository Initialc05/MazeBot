from __future__ import annotations

import os
import sys
import time
from typing import Callable, Iterable, List, Optional, Tuple


AUTONAVI_DIR = os.path.join(os.path.dirname(__file__), "autonavi")
if AUTONAVI_DIR not in sys.path:
    sys.path.insert(0, AUTONAVI_DIR)

from maze_controller import MazeController
from maze_matcher import MazeMatcher
from maze_navigator import MazeNavigator
from maze_perception import MazePerception
from maze_topology import MazeTopology
from maze_types import MAZE_SIZE, Cell, Direction, WallState


class AutoNavRuntime:
    """
    Runtime bridge for the upper-computer auto-navigation stack.

    The real topology, localization, matcher, controller, navigator, and A*
    planner are constructed here. Command output is controlled separately so
    the planner can be mounted without taking control away from manual testing.
    """

    def __init__(
        self,
        map_size: int,
        map_resolution: float,
        goal: Tuple[int, int] = (4, 4),
        send_command: Optional[Callable[[str], None]] = None,
        command_output_enabled: bool = False,
    ):
        goal_row = max(0, min(MAZE_SIZE - 1, goal[0]))
        goal_col = max(0, min(MAZE_SIZE - 1, goal[1]))
        self.goal_cell = Cell(goal_row, goal_col)
        self.start_cell = Cell(0, 0)
        self.command_output_enabled = command_output_enabled
        self.serial_command_sender = send_command

        self.perception = MazePerception(map_size, map_resolution)
        self.topology = MazeTopology()
        self.matcher = MazeMatcher(map_size, map_resolution)
        self.controller = MazeController(self._send_autonav_command, self.perception)
        self.navigator = MazeNavigator(self.topology, self.perception, self.matcher, self.controller)
        self.navigator.configure_goal(self.goal_cell.row, self.goal_cell.col)

        self.last_tick = 0.0
        self.tick_interval = 0.5
        self.last_command: Optional[str] = None
        self.last_path: List[Cell] = []
        self.status = "NAV:mounted"

        self._seed_open_reference_topology()
        self._refresh_static_plan()

    def _send_autonav_command(self, command: str) -> None:
        self.last_command = command
        if self.command_output_enabled and self.serial_command_sender is not None:
            self.serial_command_sender(command)

    def _seed_open_reference_topology(self) -> None:
        for row in range(MAZE_SIZE):
            for col in range(MAZE_SIZE):
                cell = Cell(row, col)
                for direction in Direction:
                    if self.topology.neighbor(cell, direction) is not None:
                        self.topology.set_wall(cell, direction, WallState.OPEN, confidence=1.0)

    def _refresh_static_plan(self) -> None:
        self.last_path = self.topology.plan_path(self.start_cell, self.goal_cell, allow_unknown=False)
        self.navigator.current_plan = self.last_path
        self.status = f"NAV:{len(self.last_path)}cells"

    def tick(
        self,
        pose_world: Optional[Tuple[float, float, float]],
        grid_map,
        scan_points: Iterable[Tuple[float, float, int]] = (),
    ) -> str:
        now = time.time()
        if now - self.last_tick < self.tick_interval:
            return self.status
        self.last_tick = now

        if pose_world is None:
            self._refresh_static_plan()
            return self.status

        if not self.perception.has_reference():
            self.perception.set_reference(*pose_world)

        current_cell = self.perception.world_to_cell(pose_world[0], pose_world[1])
        self.last_path = self.topology.plan_path(current_cell, self.goal_cell, allow_unknown=False)
        self.navigator.current_cell = current_cell
        self.navigator.current_plan = self.last_path

        points = list(scan_points)
        if grid_map is not None and len(points) >= 20:
            self.navigator.current_match = self.matcher.match_scan(grid_map, points, pose_world)

        if self.last_path:
            self.status = f"NAV:{len(self.last_path)}cells"
        else:
            self.status = "NAV:no-path"
        return self.status

    def short_status(self) -> str:
        return self.status
