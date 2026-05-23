from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Dict, Optional, Tuple

MAZE_SIZE = 5
CELL_SIZE_M = 0.70


class Direction(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    def left(self) -> "Direction":
        return Direction((int(self) - 1) % 4)

    def right(self) -> "Direction":
        return Direction((int(self) + 1) % 4)

    def opposite(self) -> "Direction":
        return Direction((int(self) + 2) % 4)


class WallState(IntEnum):
    UNKNOWN = 0
    OPEN = 1
    BLOCKED = 2


class NavState(str, Enum):
    IDLE = "IDLE"
    LOCALIZE_START = "LOCALIZE_START"
    UPDATE_LOCAL_MAP = "UPDATE_LOCAL_MAP"
    PLAN = "PLAN"
    ALIGN_TO_EDGE = "ALIGN_TO_EDGE"
    ADVANCE_ONE_CELL = "ADVANCE_ONE_CELL"
    SETTLE_AND_MATCH = "SETTLE_AND_MATCH"
    UPDATE_TOPOLOGY = "UPDATE_TOPOLOGY"
    CHECK_GOAL = "CHECK_GOAL"
    RETURN_PLAN = "RETURN_PLAN"
    RETURNING = "RETURNING"
    RECOVERY = "RECOVERY"
    SAFE_STOP = "SAFE_STOP"
    MISSION_COMPLETE = "MISSION_COMPLETE"


class MotionMode(str, Enum):
    IDLE = "IDLE"
    SPIN = "SPIN"
    DRIVE = "DRIVE"


@dataclass(frozen=True)
class Cell:
    row: int
    col: int


@dataclass
class DiscretePose:
    cell: Cell
    heading: Direction


@dataclass
class MatchResult:
    dx: float = 0.0
    dy: float = 0.0
    dtheta: float = 0.0
    score: float = 0.0
    accepted: bool = False
    corrected_pose: Optional[Tuple[float, float, float]] = None


@dataclass
class CellObservation:
    walls: Dict[Direction, WallState] = field(default_factory=dict)
    confidences: Dict[Direction, float] = field(default_factory=dict)
    center_error: float = 0.0
    lateral_error: float = 0.0
    forward_clearance: float = 0.0
    match_score: float = 0.0


@dataclass
class MotionStatus:
    mode: MotionMode = MotionMode.IDLE
    command: str = "x"
    done: bool = False
    success: bool = False
    reason: str = "idle"


@dataclass
class PlanStep:
    source_cell: Cell
    target_cell: Cell
    move_dir: Direction
    target_heading_world: float
