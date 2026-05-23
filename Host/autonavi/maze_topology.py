from __future__ import annotations

import heapq
from collections import deque
from typing import Dict, List, Optional, Set, Tuple

from maze_types import CELL_SIZE_M, MAZE_SIZE, Cell, Direction, WallState


class MazeTopology:
    def __init__(self, size: int = MAZE_SIZE):
        self.size = size
        self.walls = [[[WallState.UNKNOWN for _ in range(4)] for _ in range(size)] for _ in range(size)]
        self.confidence = [[[0.0 for _ in range(4)] for _ in range(size)] for _ in range(size)]
        self.visited = [[0 for _ in range(size)] for _ in range(size)]
        self.edge_traversals: Dict[frozenset[Cell], int] = {}
        self.success_edges: List[Tuple[Cell, Cell]] = []

    def reset(self) -> None:
        self.__init__(self.size)

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell.row < self.size and 0 <= cell.col < self.size

    def neighbor(self, cell: Cell, direction: Direction) -> Optional[Cell]:
        offsets = {
            Direction.NORTH: (1, 0),
            Direction.EAST: (0, 1),
            Direction.SOUTH: (-1, 0),
            Direction.WEST: (0, -1),
        }
        dr, dc = offsets[direction]
        nxt = Cell(cell.row + dr, cell.col + dc)
        return nxt if self.in_bounds(nxt) else None

    def set_wall(self, cell: Cell, direction: Direction, state: WallState, confidence: float = 1.0) -> None:
        if not self.in_bounds(cell):
            return
        self.walls[cell.row][cell.col][int(direction)] = state
        self.confidence[cell.row][cell.col][int(direction)] = max(
            self.confidence[cell.row][cell.col][int(direction)], confidence
        )
        other = self.neighbor(cell, direction)
        if other is not None:
            opposite = direction.opposite()
            self.walls[other.row][other.col][int(opposite)] = state
            self.confidence[other.row][other.col][int(opposite)] = max(
                self.confidence[other.row][other.col][int(opposite)], confidence
            )

    def get_wall(self, cell: Cell, direction: Direction) -> WallState:
        if not self.in_bounds(cell):
            return WallState.BLOCKED
        other = self.neighbor(cell, direction)
        if other is None:
            return WallState.BLOCKED
        return self.walls[cell.row][cell.col][int(direction)]

    def mark_visited(self, cell: Cell) -> None:
        if self.in_bounds(cell):
            self.visited[cell.row][cell.col] += 1

    def mark_traversed(self, a: Cell, b: Cell) -> None:
        edge = frozenset((a, b))
        self.edge_traversals[edge] = self.edge_traversals.get(edge, 0) + 1
        self.success_edges.append((a, b))

    def open_neighbors(self, cell: Cell, allow_unknown: bool = False) -> List[Cell]:
        result: List[Cell] = []
        for direction in Direction:
            state = self.get_wall(cell, direction)
            if state == WallState.OPEN or (allow_unknown and state == WallState.UNKNOWN):
                nxt = self.neighbor(cell, direction)
                if nxt is not None:
                    result.append(nxt)
        return result

    def frontier_cells(self) -> List[Cell]:
        frontiers: List[Cell] = []
        for row in range(self.size):
            for col in range(self.size):
                cell = Cell(row, col)
                if self.visited[row][col] == 0:
                    continue
                if any(self.get_wall(cell, direction) == WallState.UNKNOWN for direction in Direction):
                    frontiers.append(cell)
        return frontiers

    def _heuristic(self, a: Cell, b: Cell) -> int:
        return abs(a.row - b.row) + abs(a.col - b.col)

    def plan_path(self, start: Cell, goal: Cell, allow_unknown: bool = False, unknown_cost: float = 4.0) -> List[Cell]:
        if not (self.in_bounds(start) and self.in_bounds(goal)):
            return []
        frontier: List[Tuple[float, int, Cell]] = []
        heapq.heappush(frontier, (0.0, 0, start))
        came_from: Dict[Cell, Optional[Cell]] = {start: None}
        cost_so_far: Dict[Cell, float] = {start: 0.0}
        counter = 1

        while frontier:
            _, _, current = heapq.heappop(frontier)
            if current == goal:
                break
            for direction in Direction:
                state = self.get_wall(current, direction)
                if state == WallState.BLOCKED:
                    continue
                if state == WallState.UNKNOWN and not allow_unknown:
                    continue
                nxt = self.neighbor(current, direction)
                if nxt is None:
                    continue
                edge_cost = 1.0 if state == WallState.OPEN else unknown_cost
                new_cost = cost_so_far[current] + edge_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self._heuristic(nxt, goal)
                    heapq.heappush(frontier, (priority, counter, nxt))
                    counter += 1
                    came_from[nxt] = current

        if goal not in came_from:
            return []
        path: List[Cell] = []
        cur: Optional[Cell] = goal
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()
        return path

    def direction_between(self, current: Cell, nxt: Cell) -> Optional[Direction]:
        dr = nxt.row - current.row
        dc = nxt.col - current.col
        if dr == 1 and dc == 0:
            return Direction.NORTH
        if dr == -1 and dc == 0:
            return Direction.SOUTH
        if dr == 0 and dc == 1:
            return Direction.EAST
        if dr == 0 and dc == -1:
            return Direction.WEST
        return None

    def choose_frontier(self, current: Cell, goal: Cell) -> Optional[Cell]:
        frontiers = self.frontier_cells()
        if not frontiers:
            return None
        best = None
        best_score = None
        for frontier in frontiers:
            score = self._heuristic(current, frontier) + self._heuristic(frontier, goal)
            if best_score is None or score < best_score:
                best = frontier
                best_score = score
        return best
