"""
Simulator: robot that starts with no wall knowledge, senses its current
cell's 4 walls (mimicking lidar), updates its belief map, re-floods, and
moves one cell at a time until it reaches the goal.

This mimics the intended real-robot behavior where:
    - Ground truth: the real maze (unknown to the robot).
    - Belief: a Maze object starting with only outer walls, slowly filled in.
    - "Sensing" = peek at ground-truth walls of the current cell (stands
      in for lidar seeing 4 walls of the cell it currently occupies).
    - "Moving" = teleport one cell in the chosen direction (no kinematics).

Two-pass option:
    - Pass 1 (exploration): run the above until goal reached. This fills
      in most of the belief map.
    - Pass 2 (sprint): reset to entrance, flood on the *final* belief,
      follow the path greedily — this is the "competition run."
"""
from __future__ import annotations
from dataclasses import dataclass, field
import copy
from maze import Maze, DIRS, DX, DY, DIR_NAMES
from flood_fill import flood_fill, choose_next_move, INF


@dataclass
class SimStep:
    """One frame of the simulation, for visualization."""
    pos: tuple[int, int]
    heading: int
    belief: Maze
    dist_field: object  # np.ndarray
    action: str  # 'sense', 'move', 'turn', 'done', 'stuck'


@dataclass
class Simulator:
    truth: Maze
    start: tuple[int, int]
    goal: tuple[int, int]
    turn_penalty: int = 2
    prior: Maze | None = None  # If given, seed belief with this (teacher-provided map)
    _history: list[SimStep] = field(default_factory=list)

    def __post_init__(self):
        if self.prior is not None:
            # Start with teacher's map as our belief. Sensing can still add
            # walls if the teacher's map missed something.
            self.belief = copy.deepcopy(self.prior)
        else:
            # No map: start with only outer walls, explore from scratch.
            self.belief = Maze(self.truth.rows, self.truth.cols)
        self.pos = self.start
        self.heading = 0  # N by default
        self.total_steps = 0
        self.total_turns = 0

    def sense(self):
        """Copy truth walls of current cell into belief. Simulates lidar."""
        r, c = self.pos
        for d in DIRS:
            if self.truth.has_wall(r, c, d):
                self.belief.set_wall(r, c, d, present=True)

    def step(self) -> SimStep:
        """Do one sense-decide-move cycle. Returns a SimStep for visualization."""
        self.sense()
        dist = flood_fill(self.belief, self.goal)
        if self.pos == self.goal:
            s = SimStep(self.pos, self.heading, copy.deepcopy(self.belief),
                        dist, 'done')
            self._history.append(s)
            return s

        next_dir = choose_next_move(self.belief, self.pos, dist,
                                    self.heading, self.turn_penalty)
        if next_dir is None:
            s = SimStep(self.pos, self.heading, copy.deepcopy(self.belief),
                        dist, 'stuck')
            self._history.append(s)
            return s

        # Count turn cost
        if next_dir != self.heading:
            self.total_turns += 1

        # Move
        r, c = self.pos
        self.pos = (r + DY[next_dir], c + DX[next_dir])
        self.heading = next_dir
        self.total_steps += 1

        s = SimStep(self.pos, self.heading, copy.deepcopy(self.belief),
                    dist, f'move {DIR_NAMES[next_dir]}')
        self._history.append(s)
        return s

    def run(self, max_steps: int = 500) -> list[SimStep]:
        """Run until goal/stuck/timeout. Returns full history."""
        # Record initial state
        self.sense()
        dist = flood_fill(self.belief, self.goal)
        self._history.append(SimStep(self.pos, self.heading,
                                     copy.deepcopy(self.belief),
                                     dist, 'start'))
        for _ in range(max_steps):
            s = self.step()
            if s.action in ('done', 'stuck'):
                break
        return self._history

    def run_direct(self, max_steps: int = 500) -> list[SimStep]:
        """
        Branch B shortcut: robot trusts the prior map completely. Computes
        one distance field, walks the path without sensing/re-planning.
        Use this ONLY when you trust your prior map.
        """
        dist = flood_fill(self.belief, self.goal)
        self._history.append(SimStep(self.pos, self.heading,
                                     copy.deepcopy(self.belief),
                                     dist, 'direct_start'))
        for _ in range(max_steps):
            if self.pos == self.goal:
                self._history.append(SimStep(self.pos, self.heading,
                                             copy.deepcopy(self.belief),
                                             dist, 'done'))
                break
            next_dir = choose_next_move(self.belief, self.pos, dist,
                                        self.heading, turn_penalty=0)
            if next_dir is None:
                self._history.append(SimStep(self.pos, self.heading,
                                             copy.deepcopy(self.belief),
                                             dist, 'stuck'))
                break
            if next_dir != self.heading:
                self.total_turns += 1
            r, c = self.pos
            self.pos = (r + DY[next_dir], c + DX[next_dir])
            self.heading = next_dir
            self.total_steps += 1
            self._history.append(SimStep(self.pos, self.heading,
                                         copy.deepcopy(self.belief),
                                         dist, f'direct {DIR_NAMES[next_dir]}'))
        return self._history

    def sprint(self) -> list[SimStep]:
        """
        Pass 2: with the current belief map fully developed, re-run from
        start to goal greedily. Turn penalty = 0 here because we're
        optimizing the learned map, not exploring.
        """
        self.pos = self.start
        self.heading = 0
        history = []
        # Final belief is already there, don't re-sense
        dist = flood_fill(self.belief, self.goal)
        history.append(SimStep(self.pos, self.heading,
                               copy.deepcopy(self.belief),
                               dist, 'sprint_start'))
        for _ in range(500):
            if self.pos == self.goal:
                history.append(SimStep(self.pos, self.heading,
                                       copy.deepcopy(self.belief),
                                       dist, 'sprint_done'))
                break
            next_dir = choose_next_move(self.belief, self.pos, dist,
                                        self.heading, turn_penalty=0)
            if next_dir is None:
                history.append(SimStep(self.pos, self.heading,
                                       copy.deepcopy(self.belief),
                                       dist, 'sprint_stuck'))
                break
            r, c = self.pos
            self.pos = (r + DY[next_dir], c + DX[next_dir])
            self.heading = next_dir
            history.append(SimStep(self.pos, self.heading,
                                   copy.deepcopy(self.belief),
                                   dist, f'sprint {DIR_NAMES[next_dir]}'))
        return history


if __name__ == '__main__':
    from maze import build_z_island_maze
    m = build_z_island_maze()

    print("=== Branch A: no prior map (explore from scratch) ===")
    sim = Simulator(truth=m, start=(4, 0), goal=(2, 4))
    hist = sim.run()
    print(f"  explore: {sim.total_steps} cells, {sim.total_turns} turns, "
          f"ended '{hist[-1].action}'")
    sprint_hist = sim.sprint()
    sprint_steps = sum(1 for s in sprint_hist
                       if s.action.startswith('sprint ') and 'start' not in s.action)
    print(f"  sprint: {sprint_steps} cells, ends '{sprint_hist[-1].action}'")

    print()
    print("=== Branch B: with prior map (teacher gave it) ===")
    # Use truth as the prior — simulates "teacher gave a perfect map"
    sim2 = Simulator(truth=m, start=(4, 0), goal=(2, 4), prior=m)
    hist2 = sim2.run()
    print(f"  explore: {sim2.total_steps} cells, {sim2.total_turns} turns, "
          f"ended '{hist2[-1].action}'")
    sprint_hist2 = sim2.sprint()
    sprint_steps2 = sum(1 for s in sprint_hist2
                        if s.action.startswith('sprint ') and 'start' not in s.action)
    print(f"  sprint: {sprint_steps2} cells, ends '{sprint_hist2[-1].action}'")
