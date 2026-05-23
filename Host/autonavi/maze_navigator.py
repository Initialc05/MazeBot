from __future__ import annotations

import time
from typing import Iterable, List, Optional, Tuple

from maze_controller import MazeController
from maze_matcher import MazeMatcher
from maze_perception import MazePerception
from maze_topology import MazeTopology
from maze_types import CELL_SIZE_M, Cell, Direction, MatchResult, MotionMode, NavState, PlanStep, WallState


class MazeNavigator:
    def __init__(self, topology: MazeTopology, perception: MazePerception, matcher: MazeMatcher, controller: MazeController):
        self.topology = topology
        self.perception = perception
        self.matcher = matcher
        self.controller = controller
        self.state = NavState.IDLE
        self.enabled = False
        self.return_mode = False
        self.goal_cell = Cell(4, 4)
        self.start_cell = Cell(0, 0)
        self.current_pose_world: Optional[Tuple[float, float, float]] = None
        self.current_cell = Cell(0, 0)
        self.current_heading = Direction.EAST
        self.current_match = MatchResult(corrected_pose=(0.0, 0.0, 0.0))
        self.latest_scan_points: List[Tuple[float, float, int]] = []
        self.current_plan: List[Cell] = []
        self.pending_step: Optional[PlanStep] = None
        self.last_state_change = time.time()
        self.status_message = "manual"

    def configure_goal(self, row: int, col: int) -> None:
        self.goal_cell = Cell(max(0, min(4, row)), max(0, min(4, col)))
        self.status_message = f"goal=({self.goal_cell.row},{self.goal_cell.col})"

    def start(self) -> None:
        self.enabled = True
        self.return_mode = False
        self.state = NavState.LOCALIZE_START
        self.topology.reset()
        self.current_plan = []
        self.pending_step = None
        self.last_state_change = time.time()
        self.status_message = "auto-start"

    def abort(self, reason: str = "abort") -> None:
        self.enabled = False
        self.state = NavState.SAFE_STOP
        self.controller.stop(reason)
        self.status_message = reason

    def set_latest_scan(self, scan_points: Iterable[Tuple[float, float, int]]) -> None:
        self.latest_scan_points = list(scan_points)

    def _transition(self, new_state: NavState, message: str) -> None:
        self.state = new_state
        self.last_state_change = time.time()
        self.status_message = message

    def _update_match(self, pose: Tuple[float, float, float], grid_map) -> MatchResult:
        self.current_match = self.matcher.match_scan(grid_map, self.latest_scan_points, pose)
        corrected = self.current_match.corrected_pose or pose
        snapped_x, snapped_y, snapped_theta, snapped_cell, snapped_heading = self.perception.snapped_pose(corrected, self.current_match)
        self.current_pose_world = (snapped_x, snapped_y, snapped_theta)
        self.current_cell = snapped_cell
        self.current_heading = snapped_heading
        return self.current_match

    def _update_topology(self, grid_map) -> None:
        if self.current_pose_world is None:
            return
        obs = self.perception.observe_cell(grid_map, self.current_pose_world, self.current_match, self.latest_scan_points)
        self.topology.mark_visited(self.current_cell)
        for direction, state in obs.walls.items():
            confidence = obs.confidences.get(direction, 0.0)
            if state != WallState.UNKNOWN and confidence >= 0.20:
                self.topology.set_wall(self.current_cell, direction, state, confidence)

    def _plan_next(self) -> Optional[PlanStep]:
        target = self.start_cell if self.return_mode else self.goal_cell
        confirmed = self.topology.plan_path(self.current_cell, target, allow_unknown=False)
        if len(confirmed) >= 2:
            path = confirmed
        else:
            frontier = self.topology.choose_frontier(self.current_cell, target)
            if frontier is None:
                return None
            path = self.topology.plan_path(self.current_cell, frontier, allow_unknown=True)
            if len(path) < 2:
                return None
        self.current_plan = path
        next_cell = path[1]
        move_dir = self.topology.direction_between(self.current_cell, next_cell)
        if move_dir is None:
            return None
        return PlanStep(source_cell=self.current_cell, target_cell=next_cell, move_dir=move_dir, target_heading_world=self.perception.heading_to_world(move_dir))

    def _select_turn_cmd(self, target_dir: Direction) -> Optional[str]:
        delta = (int(target_dir) - int(self.current_heading)) % 4
        if delta == 0:
            return None
        if delta == 1:
            return 'D'
        if delta == 3:
            return 'A'
        return 'D'

    def step(self, pose_world: Tuple[float, float, float], grid_map, scan_points: Iterable[Tuple[float, float, int]]) -> str:
        self.current_pose_world = pose_world
        self.latest_scan_points = list(scan_points)
        if not self.enabled:
            return self.status_message

        if self.state == NavState.LOCALIZE_START:
            if not self.perception.has_reference():
                self.perception.set_reference(*pose_world)
            self._update_match(pose_world, grid_map)
            self.current_cell = self.start_cell
            self.current_heading = self.perception.quantize_heading(self.current_pose_world[2])
            self._transition(NavState.UPDATE_TOPOLOGY, "localized")
            return self.status_message

        if self.state == NavState.UPDATE_TOPOLOGY:
            self._update_topology(grid_map)
            self._transition(NavState.CHECK_GOAL, "topology-updated")
            return self.status_message

        if self.state == NavState.CHECK_GOAL:
            target = self.start_cell if self.return_mode else self.goal_cell
            if self.current_cell == target:
                if self.return_mode:
                    self.enabled = False
                    self._transition(NavState.MISSION_COMPLETE, "mission-complete")
                    self.controller.stop("stopped")
                    return self.status_message
                self.return_mode = True
                self._transition(NavState.PLAN, "goal-reached-plan-return")
                return self.status_message
            self._transition(NavState.PLAN, "plan")
            return self.status_message

        if self.state == NavState.PLAN:
            self.pending_step = self._plan_next()
            if self.pending_step is None:
                self.abort("no-path")
                return self.status_message
            turn_cmd = self._select_turn_cmd(self.pending_step.move_dir)
            if turn_cmd is None:
                self._transition(NavState.ADVANCE_ONE_CELL, "aligned")
            else:
                self.controller.start_spin(self.current_pose_world, self.pending_step.target_heading_world, turn_cmd)
                self._transition(NavState.ALIGN_TO_EDGE, f"turn-{turn_cmd}")
            return self.status_message

        if self.state == NavState.ALIGN_TO_EDGE:
            motion = self.controller.update(pose_world, self.current_match, self.latest_scan_points)
            if motion.done:
                if motion.success:
                    self._transition(NavState.ADVANCE_ONE_CELL, motion.reason)
                else:
                    self._transition(NavState.RECOVERY, motion.reason)
            return self.status_message

        if self.state == NavState.ADVANCE_ONE_CELL:
            if self.pending_step is None:
                self._transition(NavState.PLAN, "missing-step")
                return self.status_message
            if self.controller.status.mode == MotionMode.IDLE:
                self.controller.start_drive(self.current_pose_world, self.pending_step.target_heading_world, distance=CELL_SIZE_M)
                self.status_message = "driving"
                return self.status_message
            motion = self.controller.update(pose_world, self.current_match, self.latest_scan_points)
            if motion.done:
                if motion.success:
                    self._transition(NavState.SETTLE_AND_MATCH, motion.reason)
                else:
                    self._transition(NavState.RECOVERY, motion.reason)
            return self.status_message

        if self.state == NavState.SETTLE_AND_MATCH:
            self._update_match(pose_world, grid_map)
            if self.pending_step is not None:
                self.topology.mark_traversed(self.pending_step.source_cell, self.pending_step.target_cell)
                self.current_cell = self.pending_step.target_cell
                self.current_heading = self.pending_step.move_dir
            self._transition(NavState.UPDATE_TOPOLOGY, "matched")
            return self.status_message

        if self.state == NavState.RECOVERY:
            self.controller.stop("recovery-stop")
            self._update_match(pose_world, grid_map)
            self._transition(NavState.UPDATE_TOPOLOGY, "recovered")
            return self.status_message

        return self.status_message
