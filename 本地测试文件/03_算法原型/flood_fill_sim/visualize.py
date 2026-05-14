"""
Visualize a Simulator run with matplotlib animation.

Run:
    python visualize.py                          # default: z_island_5x5, no prior
    python visualize.py simple_3x3
    python visualize.py corridor_5x5
    python visualize.py z_island_5x5
    python visualize.py z_island_5x5 --prior     # with teacher-provided map
    python visualize.py z_island_5x5 --prior=partial   # imperfect teacher map
"""
from __future__ import annotations
import sys
import copy
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np

from maze import Maze, MAZES, DIR_NAMES, DIRS
from simulator import Simulator
from flood_fill import INF


CELL = 1.0  # cell size in plot units
HEADING_ARROWS = {0: '↑', 1: '→', 2: '↓', 3: '←'}


def draw_maze_walls(ax, maze: Maze, color='black', lw=2, alpha=1.0):
    """Draw all walls currently in `maze`."""
    for r in range(maze.rows):
        for c in range(maze.cols):
            x, y = c * CELL, r * CELL
            if maze.has_wall(r, c, 0):  # N
                ax.plot([x, x + CELL], [y + CELL, y + CELL], color=color, lw=lw, alpha=alpha)
            if maze.has_wall(r, c, 1):  # E
                ax.plot([x + CELL, x + CELL], [y, y + CELL], color=color, lw=lw, alpha=alpha)
            if maze.has_wall(r, c, 2):  # S
                ax.plot([x, x + CELL], [y, y], color=color, lw=lw, alpha=alpha)
            if maze.has_wall(r, c, 3):  # W
                ax.plot([x, x], [y, y + CELL], color=color, lw=lw, alpha=alpha)


def render_step(ax_maze, ax_dist, step, truth, start, goal):
    ax_maze.clear()
    ax_dist.clear()

    rows, cols = truth.rows, truth.cols

    # === Left: maze + robot ===
    # Light gray: truth walls (what's really there)
    draw_maze_walls(ax_maze, truth, color='lightgray', lw=3)
    # Bold black: believed walls (what the robot has sensed)
    draw_maze_walls(ax_maze, step.belief, color='black', lw=3)

    # Start & goal cells
    sr, sc = start
    gr, gc = goal
    ax_maze.add_patch(patches.Rectangle((sc * CELL, sr * CELL), CELL, CELL,
                                        facecolor='#a8e6cf', alpha=0.6, zorder=0))
    ax_maze.add_patch(patches.Rectangle((gc * CELL, gr * CELL), CELL, CELL,
                                        facecolor='#ffaaa5', alpha=0.6, zorder=0))
    ax_maze.text(sc * CELL + CELL / 2, sr * CELL + CELL / 2, 'S',
                 ha='center', va='center', fontsize=14, weight='bold', zorder=1)
    ax_maze.text(gc * CELL + CELL / 2, gr * CELL + CELL / 2, 'G',
                 ha='center', va='center', fontsize=14, weight='bold', zorder=1)

    # Robot
    r, c = step.pos
    ax_maze.add_patch(patches.Circle((c * CELL + CELL / 2, r * CELL + CELL / 2),
                                     0.25, facecolor='#3498db', edgecolor='black',
                                     lw=2, zorder=5))
    ax_maze.text(c * CELL + CELL / 2, r * CELL + CELL / 2,
                 HEADING_ARROWS[step.heading],
                 ha='center', va='center', fontsize=14, color='white',
                 weight='bold', zorder=6)

    ax_maze.set_xlim(-0.3, cols * CELL + 0.3)
    ax_maze.set_ylim(-0.3, rows * CELL + 0.3)
    ax_maze.set_aspect('equal')
    ax_maze.set_xticks([])
    ax_maze.set_yticks([])
    ax_maze.set_title(f'Maze (bold=known, faint=truth)\naction: {step.action}', fontsize=11)

    # === Right: distance field ===
    field_display = step.dist_field.copy().astype(float)
    field_display[field_display >= INF] = np.nan
    im = ax_dist.imshow(field_display, cmap='viridis_r', origin='lower',
                        extent=[0, cols * CELL, 0, rows * CELL],
                        vmin=0, vmax=max(rows + cols, 10))
    draw_maze_walls(ax_dist, step.belief, color='white', lw=2, alpha=0.8)

    # Overlay numbers
    for rr in range(rows):
        for cc in range(cols):
            v = step.dist_field[rr, cc]
            if v < INF:
                ax_dist.text(cc * CELL + CELL / 2, rr * CELL + CELL / 2,
                             str(v), ha='center', va='center',
                             color='white', fontsize=10, weight='bold')
    # Robot dot on distance field too
    ax_dist.add_patch(patches.Circle((c * CELL + CELL / 2, r * CELL + CELL / 2),
                                     0.18, facecolor='red', edgecolor='white',
                                     lw=1.5, zorder=5))

    ax_dist.set_xlim(0, cols * CELL)
    ax_dist.set_ylim(0, rows * CELL)
    ax_dist.set_aspect('equal')
    ax_dist.set_xticks([])
    ax_dist.set_yticks([])
    ax_dist.set_title('Flood-Fill distance field\n(numbers = steps to goal)', fontsize=11)


def run(maze_name: str = 'z_island_5x5', prior_mode: str = 'none'):
    """
    prior_mode:
        'none'    — Branch A: robot has no map, explores from scratch
        'full'    — Branch B: teacher gave a perfect map (== truth)
        'partial' — Branch B': teacher's map is missing some walls
                    (still useful, but sensing will add to it)
    """
    builder, start, goal = MAZES[maze_name]
    truth = builder()

    prior = None
    if prior_mode == 'full':
        prior = copy.deepcopy(truth)
    elif prior_mode == 'partial':
        # Take the truth but drop ~half the internal walls — simulates a
        # teacher map that's approximately right but not surveyed to cm accuracy
        prior = copy.deepcopy(truth)
        # Clear non-outer walls probabilistically (deterministic for reproducibility)
        rng = np.random.default_rng(42)
        for r in range(prior.rows):
            for c in range(prior.cols):
                for d in DIRS:
                    # Only touch internal walls: has a neighbor in that direction
                    from maze import DX, DY
                    nr, nc = r + DY[d], c + DX[d]
                    if prior.in_bounds(nr, nc) and prior.has_wall(r, c, d):
                        if rng.random() < 0.5:
                            prior.set_wall(r, c, d, present=False)

    sim = Simulator(truth=truth, start=start, goal=goal, turn_penalty=2, prior=prior)

    if prior_mode == 'full':
        # Branch B shortcut: trust the prior, go straight to goal.
        # No explore, no sprint — single pass.
        hist_full = sim.run_direct()
        mode_label = f'full prior (直接冲刺, 跳过探索)'
        print(f'[{maze_name}] mode={mode_label}')
        print(f"  direct run: {sim.total_steps} cells, {sim.total_turns} turns, "
              f"ended {hist_full[-1].action}")
        full_history = hist_full
    else:
        hist_explore = sim.run(max_steps=300)
        mode_label = {'none': 'no prior (从零探索)',
                      'partial': 'partial prior (部分图)'}[prior_mode]
        print(f'[{maze_name}] mode={mode_label}')
        print(f"  Explore: {sim.total_steps} cells, {sim.total_turns} turns, "
              f"ended {hist_explore[-1].action}")

        # Pass 2: sprint with learned map
        hist_sprint = sim.sprint()
        sprint_cells = sum(1 for s in hist_sprint
                           if s.action.startswith('sprint ') and 'start' not in s.action)
        sprint_turns = sum(1 for i in range(1, len(hist_sprint))
                           if hist_sprint[i].heading != hist_sprint[i - 1].heading
                           and hist_sprint[i].action.startswith('sprint '))
        print(f"  Sprint:  {sprint_cells} cells, {sprint_turns} turns, "
              f"ended {hist_sprint[-1].action}")
        full_history = hist_explore + hist_sprint

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    fig.suptitle(f'Flood-Fill — {maze_name} — {mode_label}', fontsize=13)

    def update(frame_idx):
        render_step(ax1, ax2, full_history[frame_idx], truth, start, goal)
        return []

    ani = FuncAnimation(fig, update, frames=len(full_history),
                        interval=500, repeat=True, blit=False)

    plt.tight_layout()
    plt.show()
    # Keep ani referenced so Python doesn't GC it
    return ani


if __name__ == '__main__':
    args = sys.argv[1:]
    maze_name = 'z_island_5x5'
    prior_mode = 'none'
    for a in args:
        if a == '--prior' or a == '--prior=full':
            prior_mode = 'full'
        elif a == '--prior=partial':
            prior_mode = 'partial'
        elif a == '--no-prior':
            prior_mode = 'none'
        elif a in MAZES:
            maze_name = a
        else:
            print(f"Unknown arg '{a}'. Mazes: {list(MAZES)}. Options: --prior, --prior=partial, --no-prior")
            sys.exit(1)
    run(maze_name, prior_mode)
