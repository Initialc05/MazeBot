"""
Maze data structure and a few hand-designed mazes for testing.

Coordinate convention (matches matplotlib imshow with origin='lower'):
    col (x) →    right
    row (y) ↑    up
Heading: 0=North(+y), 1=East(+x), 2=South(-y), 3=West(-x)

Walls are stored per-cell as a 4-bit mask:
    bit 0 = N wall, bit 1 = E wall, bit 2 = S wall, bit 3 = W wall

A cell knowing its N wall implies the cell to the north knows its S wall
(consistency is maintained by `set_wall`).
"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

N, E, S, W = 0, 1, 2, 3
DIRS = [N, E, S, W]
DIR_NAMES = ['N', 'E', 'S', 'W']
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
OPPOSITE = [S, W, N, E]


@dataclass
class Maze:
    rows: int
    cols: int
    walls: np.ndarray = field(default=None)  # shape (rows, cols), uint8, bit 0..3

    def __post_init__(self):
        if self.walls is None:
            self.walls = np.zeros((self.rows, self.cols), dtype=np.uint8)
            self._add_outer_walls()

    def _add_outer_walls(self):
        for c in range(self.cols):
            self.set_wall(self.rows - 1, c, N)
            self.set_wall(0, c, S)
        for r in range(self.rows):
            self.set_wall(r, self.cols - 1, E)
            self.set_wall(r, 0, W)

    def set_wall(self, r: int, c: int, direction: int, present: bool = True):
        if not self.in_bounds(r, c):
            return
        mask = 1 << direction
        if present:
            self.walls[r, c] |= mask
        else:
            self.walls[r, c] &= ~mask
        # Mirror to neighbor
        nr, nc = r + DY[direction], c + DX[direction]
        if self.in_bounds(nr, nc):
            opp_mask = 1 << OPPOSITE[direction]
            if present:
                self.walls[nr, nc] |= opp_mask
            else:
                self.walls[nr, nc] &= ~opp_mask

    def has_wall(self, r: int, c: int, direction: int) -> bool:
        return bool(self.walls[r, c] & (1 << direction))

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def neighbors(self, r: int, c: int):
        """Yield (direction, nr, nc) for open neighbors."""
        for d in DIRS:
            if not self.has_wall(r, c, d):
                nr, nc = r + DY[d], c + DX[d]
                if self.in_bounds(nr, nc):
                    yield d, nr, nc

    def remove_wall(self, r: int, c: int, direction: int):
        self.set_wall(r, c, direction, present=False)


def build_z_island_maze() -> Maze:
    """
    Approximation of the user's competition maze (5x5), based on their
    hand-drawn reference image:
    - Entrance on north wall at column 0 (top-left notch in outer wall)
    - Exit on east wall at row 2 (right-side notch, middle-ish)
    - A Z-shaped island of walls in the middle that does NOT touch outer walls

    Coordinate system: row 0 = bottom, row 4 = top.
    Entrance = (4,0), exit = (2,4).
    The Z island:
      - Top horizontal: walls between (3,1)-(3,2) and (3,2)-(2,2)
      - Right vertical: walls stacking down from (3,3) to (1,3)
      - Bottom horizontal: walls between (1,1)-(1,2) and (0,2)-(0,3)
      - A short divider wall at col 2 running up from bottom

    Everything approximate — for algorithm validation, not cm-accurate.
    """
    m = Maze(5, 5)

    # Entrance notch on north wall at col 0: open (no N wall for row 4 col 0)
    m.set_wall(4, 0, N, present=False)
    # Exit notch on east wall at row 2: open (no E wall for row 2 col 4)
    m.set_wall(2, 4, E, present=False)

    # Z-island internal walls (reproducing the shape in user's image)
    # Upper horizontal bar
    m.set_wall(3, 2, N)           # wall between (3,2) and (4,2)
    m.set_wall(3, 3, N)           # wall between (3,3) and (4,3)
    # Right-side vertical bar of the Z
    m.set_wall(3, 2, E)           # wall between (3,2) and (3,3)
    m.set_wall(2, 2, E)           # wall between (2,2) and (2,3)
    m.set_wall(1, 2, E)           # wall between (1,2) and (1,3)
    # Lower horizontal bar
    m.set_wall(1, 1, N)           # wall between (1,1) and (2,1)
    m.set_wall(1, 2, N)           # wall between (1,2) and (2,2)
    # Vertical divider in col 2 from bottom
    m.set_wall(0, 2, E)           # wall between (0,2) and (0,3)
    m.set_wall(1, 2, S)           # = N side of (0,2), already handled? let's be explicit

    return m


def build_simple_3x3() -> Maze:
    """Trivial 3x3 maze, one internal wall."""
    m = Maze(3, 3)
    m.set_wall(1, 1, E)  # wall between (1,1) and (1,2)
    return m


def build_corridor_5x5() -> Maze:
    """5x5 all walls present except a serpentine corridor from (0,0) to (4,4)."""
    m = Maze(5, 5)
    # Fill all internal walls first
    for r in range(5):
        for c in range(5):
            if c < 4:
                m.set_wall(r, c, E)
            if r < 4:
                m.set_wall(r, c, N)
    # Carve a serpentine path
    # row 0: go east all the way
    for c in range(4):
        m.remove_wall(0, c, E)
    # col 4: go up 1
    m.remove_wall(0, 4, N)
    # row 1: go west all the way
    for c in range(4):
        m.remove_wall(1, c + 1, W)
    # col 0: go up 1
    m.remove_wall(1, 0, N)
    # row 2: go east all the way
    for c in range(4):
        m.remove_wall(2, c, E)
    # col 4: go up 1
    m.remove_wall(2, 4, N)
    # row 3: go west all the way
    for c in range(4):
        m.remove_wall(3, c + 1, W)
    # col 0: go up 1
    m.remove_wall(3, 0, N)
    # row 4: east
    for c in range(4):
        m.remove_wall(4, c, E)
    return m


def build_trap_5x5() -> Maze:
    """
    A maze designed to penalize no-prior exploration:
    - Short-looking path ahead has a hidden dead-end wall
    - Real shortest path requires going "backward first, then around"
    No-prior robot will walk into the trap, bounce back, then re-plan.
    With-prior robot goes straight to the real best path from step 1.
    Start = (0,0), Goal = (4,4).
    """
    m = Maze(5, 5)
    # Build a maze where (0,0)->(4,4): direct diagonal corridor traps you,
    # real path goes through row 4 via (0,0)->up col 0->east row 4.
    # Add a wall blocking the "direct" route midway:
    m.set_wall(2, 2, N)  # wall stops you climbing up middle
    m.set_wall(2, 3, N)
    m.set_wall(2, 4, N)
    # Also block the right column mid-way
    m.set_wall(1, 4, N)
    # Block row 1 at col 2 so entering middle is costly
    m.set_wall(1, 2, E)
    m.set_wall(0, 2, E)
    return m


def build_competition_5x5() -> Maze:
    """
    User's hand-drawn competition maze (5x5 logical grid).
    Verified against annotated red-line image on 2026-05-13.

    Conventions:
        - Fully enclosed: outer walls remain everywhere.
        - S=(4,1) and G=(1,4) are "placed here / stop here" cells,
          not physical openings.
        - row 0 = bottom, col 0 = left.

    Self-verification anchors (from user):
        - (1, 0): S and E walls exist
        - (2, 3): N, S, E walls exist (only W is open, into (2,2))
    """
    m = Maze(5, 5)

    # --- Standalone interior walls ---
    m.set_wall(3, 0, E)    # left-upper vertical, between (3,0) and (3,1)
    m.set_wall(4, 1, S)    # S wall of S cell (= N of (3,1))
    m.set_wall(4, 4, S)    # short horizontal near top-right
    m.set_wall(1, 0, E)    # lower-left L: (1,0) east
    m.set_wall(1, 0, S)    # lower-left L: (1,0) south
    m.set_wall(1, 2, W)    # short vertical: west of (1,2), connects (1,0) south wall up to island bottom
    m.set_wall(1, 2, S)    # short horizontal: bottom of (1,2)

    # --- Central island with two openings ---
    m.set_wall(2, 2, N)    # top of (2,2)
    m.set_wall(2, 3, N)    # top of (2,3)
    m.set_wall(2, 2, W)    # left side
    m.set_wall(2, 3, E)    # right side
    m.set_wall(2, 3, S)    # bottom-right of the island — closes (2,3) southward
    m.set_wall(1, 3, E)    # vertical below the island, between (1,3) and (1,4)

    # Openings left intentionally: S of (2,2), S of (1,3)

    return m


def build_from_ascii(ascii_map: str) -> tuple[Maze, tuple[int, int] | None, tuple[int, int] | None]:
    """
    Parse a hand-drawn ASCII maze into a Maze + optional start/goal.

    Format (every cell = 3 chars wide, 2 chars tall):
        +--+--+--+
        |  |     |        a space in the cell = floor, 'S' = start, 'G' = goal
        +  +--+  +
        |S |  | G|
        +--+--+--+

    Rules:
      * `+` marks wall corners (always there, just structural)
      * Horizontal walls: '--' between two `+`s means wall present,
        '  ' (two spaces) means no wall
      * Vertical walls: '|' means wall, ' ' means no wall
      * Inside a cell (2 chars between vertical bars): ' S' / ' G' marks
        start/goal. Anything else is treated as floor.

    Row 0 is the BOTTOM row of the ASCII (to match our row-up coord system).
    So the visual top line corresponds to the highest row.

    Returns (maze, start, goal). start/goal are None if not marked.
    """
    lines = [ln.rstrip() for ln in ascii_map.splitlines() if ln.strip()]
    # Expect odd number of lines: 2*rows + 1
    if len(lines) < 3 or len(lines) % 2 == 0:
        raise ValueError(f"ASCII maze must have 2*rows+1 lines, got {len(lines)}")
    rows = len(lines) // 2
    # Width of one cell is 3 chars (+ the leading '+' or '|'), so total width = 3*cols + 1
    width = max(len(ln) for ln in lines)
    if (width - 1) % 3 != 0:
        raise ValueError(f"ASCII maze width {width} not of form 3*cols+1")
    cols = (width - 1) // 3

    # Reverse so lines[0] is the bottom ASCII row (= maze row 0)
    lines_rev = list(reversed(lines))
    # Pad to uniform width
    lines_rev = [ln.ljust(width) for ln in lines_rev]

    # Start with no walls, then add what the ASCII says
    m = Maze(rows, cols)
    m.walls[:] = 0
    start, goal = None, None

    for r in range(rows):
        h_line_below = lines_rev[2 * r]      # horizontal wall below this row
        cell_line = lines_rev[2 * r + 1]     # cell interiors + vertical walls
        h_line_above = lines_rev[2 * r + 2] if 2 * r + 2 < len(lines_rev) else h_line_below

        for c in range(cols):
            # Horizontal wall below (S side)
            seg = h_line_below[3 * c + 1: 3 * c + 3]
            if '-' in seg:
                m.set_wall(r, c, S, present=True)
            # Horizontal wall above (N side)
            seg = h_line_above[3 * c + 1: 3 * c + 3]
            if '-' in seg:
                m.set_wall(r, c, N, present=True)
            # Vertical wall on W
            if cell_line[3 * c] == '|':
                m.set_wall(r, c, W, present=True)
            # Vertical wall on E
            if cell_line[3 * c + 3] == '|':
                m.set_wall(r, c, E, present=True)
            # Interior: look for S/G
            interior = cell_line[3 * c + 1: 3 * c + 3]
            if 'S' in interior:
                start = (r, c)
            if 'G' in interior:
                goal = (r, c)

    return m, start, goal


# Example ASCII version of the Z-island maze.
# Visually: top row is row 4 in coord system.
Z_ISLAND_ASCII = """
+--+--+--+--+--+
|              |
+  +  +  +  +  +
|     |     | G|
+  +--+--+  +--+
|        |     |
+  +--+--+  +  +
|     |     |  |
+  +  +  +  +  +
|S             |
+--+--+--+--+--+
"""


def build_z_island_from_ascii() -> Maze:
    """Same Z-island maze, but via the ASCII parser (for testing parser)."""
    m, start, goal = build_from_ascii(Z_ISLAND_ASCII)
    return m


MAZES = {
    'simple_3x3': (build_simple_3x3, (0, 0), (2, 2)),
    'corridor_5x5': (build_corridor_5x5, (0, 0), (4, 4)),
    'z_island_5x5': (build_z_island_maze, (4, 0), (2, 4)),
    'z_island_ascii': (build_z_island_from_ascii, (0, 0), (4, 4)),
    'trap_5x5': (build_trap_5x5, (0, 0), (4, 4)),
    'competition_5x5': (build_competition_5x5, (4, 1), (1, 4)),
}
