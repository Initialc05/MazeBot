"""Render competition_5x5 maze to a PNG so user can verify against hand-drawn."""
import sys
import os
_THIS = os.path.dirname(__file__)
sys.path.insert(0, _THIS)

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from maze import MAZES


def render(maze_name: str, out_path: str):
    builder, start, goal = MAZES[maze_name]
    m = builder()

    fig, ax = plt.subplots(figsize=(7, 7))

    # Outer frame & internal walls
    for r in range(m.rows):
        for c in range(m.cols):
            x, y = c, r
            if m.has_wall(r, c, 0):  # N
                ax.plot([x, x + 1], [y + 1, y + 1], 'k-', lw=3)
            if m.has_wall(r, c, 1):  # E
                ax.plot([x + 1, x + 1], [y, y + 1], 'k-', lw=3)
            if m.has_wall(r, c, 2):  # S
                ax.plot([x, x + 1], [y, y], 'k-', lw=3)
            if m.has_wall(r, c, 3):  # W
                ax.plot([x, x], [y, y + 1], 'k-', lw=3)

    # Grid lines to help counting
    for i in range(m.cols + 1):
        ax.plot([i, i], [0, m.rows], color='lightgray', lw=0.5, zorder=0)
    for i in range(m.rows + 1):
        ax.plot([0, m.cols], [i, i], color='lightgray', lw=0.5, zorder=0)

    # Start/goal
    sr, sc = start
    gr, gc = goal
    ax.add_patch(patches.Rectangle((sc, sr), 1, 1,
                                   facecolor='#a8e6cf', alpha=0.7, zorder=1))
    ax.add_patch(patches.Rectangle((gc, gr), 1, 1,
                                   facecolor='#ffaaa5', alpha=0.7, zorder=1))
    ax.text(sc + 0.5, sr + 0.5, f'S\n({sr},{sc})',
            ha='center', va='center', fontsize=11, fontweight='bold')
    ax.text(gc + 0.5, gr + 0.5, f'G\n({gr},{gc})',
            ha='center', va='center', fontsize=11, fontweight='bold')

    # Cell coordinates in each cell for reference
    for r in range(m.rows):
        for c in range(m.cols):
            if (r, c) != start and (r, c) != goal:
                ax.text(c + 0.5, r + 0.5, f'({r},{c})',
                        ha='center', va='center', fontsize=8,
                        color='#888', zorder=2)

    ax.set_xlim(-0.3, m.cols + 0.3)
    ax.set_ylim(-0.3, m.rows + 0.3)
    ax.set_aspect('equal')
    ax.set_xticks(range(m.cols + 1))
    ax.set_yticks(range(m.rows + 1))
    ax.set_xlabel('col')
    ax.set_ylabel('row (0 = bottom)')
    ax.set_title(f'Maze: {maze_name}  (S=start, G=goal)')
    ax.grid(False)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    print(f"Saved: {out_path}")


if __name__ == '__main__':
    name = sys.argv[1] if len(sys.argv) > 1 else 'competition_5x5'
    out = os.path.join(_THIS, f'preview_{name}.png')
    render(name, out)
