"""
Flood-Fill distance field.

Classic Micromouse algorithm:
    - Start from the goal, distance = 0.
    - BFS outward through every cell, distance = shortest #steps to goal
      given the currently-believed wall configuration.
    - Robot greedily moves to the neighbor with smallest distance.

The key property: if our wall belief is a subset of reality (we only know
about walls we've seen, but we never assume walls that aren't there), the
path produced is *optimistic*. It may fail when we hit an unseen wall,
but then we update the belief and re-flood. This converges to the true
shortest path. Classic "optimism under uncertainty" + re-planning.
"""
from __future__ import annotations
import numpy as np
from collections import deque
from maze import Maze, DIRS, DX, DY

INF = 10_000


def flood_fill(maze: Maze, goal: tuple[int, int]) -> np.ndarray:
    """
    Compute distance field from every cell to `goal`, respecting the walls
    currently stored in `maze`. Cells unreachable from goal get INF.

    Returns a (rows, cols) int array.
    """
    dist = np.full((maze.rows, maze.cols), INF, dtype=np.int32)
    gr, gc = goal
    dist[gr, gc] = 0
    q = deque([(gr, gc)])
    while q:
        r, c = q.popleft()
        for d, nr, nc in maze.neighbors(r, c):
            if dist[nr, nc] > dist[r, c] + 1:
                dist[nr, nc] = dist[r, c] + 1
                q.append((nr, nc))
    return dist


def choose_next_move(maze: Maze, pos: tuple[int, int],
                     dist: np.ndarray,
                     heading: int,
                     turn_penalty: int = 0) -> int | None:
    """
    Pick which direction to step. Returns a direction (N/E/S/W) or None
    if we're at the goal / stuck.

    If turn_penalty > 0, prefer directions that don't require turning
    (ties are broken in favor of going straight). This biases toward
    long straight runs — matters for speed on a real robot.
    """
    r, c = pos
    if dist[r, c] == 0:
        return None

    best_dir = None
    best_score = None
    for d, nr, nc in maze.neighbors(r, c):
        if dist[nr, nc] >= dist[r, c]:
            continue  # not a descent direction
        # Score = distance + turn cost. Lower is better.
        turn_cost = turn_penalty if d != heading else 0
        score = dist[nr, nc] + turn_cost
        if best_score is None or score < best_score:
            best_score = score
            best_dir = d
    return best_dir


if __name__ == '__main__':
    # Smoke test
    from maze import build_z_island_maze
    m = build_z_island_maze()
    d = flood_fill(m, (2, 4))  # goal = exit cell
    print(f"Distance field (goal at (2,4)):")
    # Print with row 4 on top (matches visual)
    for r in reversed(range(m.rows)):
        row_str = ' '.join(f'{d[r, c]:3d}' if d[r, c] < INF else '  .' for c in range(m.cols))
        print(f'  {row_str}')
    print(f"\nDist from entrance (4,0) to exit (2,4) = {d[4, 0]}")
