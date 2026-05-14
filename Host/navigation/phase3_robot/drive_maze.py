"""
Phase 3 — Main controller: drive the real car through the maze.

Architecture (one iteration per cell):

    loop:
        1. Read one full 360° lidar scan.
        2. Run wall_sense on it -> {N,E,S,W: wall?}.
        3. Update belief map: copy sensed walls in.
        4. If we're at goal: stop.
        5. flood_fill(belief, goal) -> distance field.
        6. choose_next_move(...) -> direction.
        7. Compute required turn (deg) to face that direction.
        8. Send L<deg> / R<deg> if needed, wait for TURN_DONE.
        9. Send F<cm> for one cell, wait for MOVE_DONE.
       10. Update position + heading in our head. Goto 1.

Heading model:
    We use integer direction indices N=0, E=1, S=2, W=3 (same as maze.py).
    To turn from current heading h to new direction d, the shortest turn is:
        delta = ((d - h + 2) % 4) - 2    # result in {-2, -1, 0, 1, 2}
        delta = -2 means 180° turn; we pick either L180 or R180 arbitrarily.
        delta = +1  → R90
        delta = -1  → L90
        delta =  0  → no turn needed
        delta = +2 or -2 → 180°; we send R180.

    Turns are sent as L90 (counter-clockwise, +y -> -x = N -> W in our world)
    OR R90 (clockwise, N -> E). Firmware convention: L = counter-clockwise,
    R = clockwise (see bt_cmd.c: type=='L' → angle positive).
    In our grid: going N->E is clockwise = R90. N->W is CCW = L90.

Usage:

    # Branch A: no prior map, explore from scratch
    python drive_maze.py --start 4 0 --goal 2 4 --rows 5 --cols 5

    # Branch B: prior map is the full Z-island (whose shape is hard-coded
    # in flood_fill_sim/maze.py as build_z_island_maze)
    python drive_maze.py --start 4 0 --goal 2 4 --prior z_island_5x5

    # Dry-run: don't actually send motion commands, just print what we WOULD do
    python drive_maze.py --start 4 0 --goal 2 4 --dry-run

    # Sense-then-stop: send the first sense loop, print result, exit
    python drive_maze.py --start 4 0 --goal 2 4 --once

Safety:
    - Before each scan we DRAIN the serial buffer so we don't act on stale data.
    - If we ever get ESTOP or TURN_TIMEOUT / MOVE_TIMEOUT, we halt and exit.
    - Ctrl+C at any time sends 'x' to stop the motors, then exits.
    - Hit the physical E-STOP button on the car if you want a hard stop.
"""
from __future__ import annotations
import sys
import os
import time
import argparse
import copy

_THIS_DIR = os.path.dirname(__file__)
_PHASE1 = os.path.join(_THIS_DIR, '..', 'flood_fill_sim')
_PHASE2 = os.path.join(_THIS_DIR, '..', 'phase2_sensing')
for p in (_PHASE1, _PHASE2):
    if p not in sys.path:
        sys.path.insert(0, p)

import serial
from bt_protocol import read_full_scan, wait_for_ack, drain
from maze import Maze, MAZES, N, E, S, W, DIR_NAMES, DX, DY
from flood_fill import flood_fill, choose_next_move, INF
from wall_sense import sense_walls, update_belief

HEADING_TO_DEG = {N: 0, E: 270, S: 180, W: 90}  # 0=facing +y (N); CCW positive


def shortest_turn(cur_heading: int, target_heading: int) -> tuple[int, str]:
    """
    Return (delta_index, cmd_string) for the shortest rotation.
    delta_index in {-2, -1, 0, 1, 2}: +1 = R90 CW, -1 = L90 CCW, ±2 = 180°.
    cmd_string is e.g. 'R90\\n', 'L90\\n', 'R180\\n', or '' if no turn needed.
    """
    delta = ((target_heading - cur_heading + 2) % 4) - 2  # {-2,-1,0,1,2}
    if delta == 0:
        return 0, ''
    if delta == 1:
        return 1, 'R90\n'
    if delta == -1:
        return -1, 'L90\n'
    # |delta| == 2: 180° — pick R180 arbitrarily
    return 2, 'R180\n'


def send_cmd(ser, cmd: str, ack_timeout: float = 12.0, dry_run: bool = False) -> str | None:
    """Send a command, wait for ack. Returns ack string or None on timeout."""
    if dry_run:
        print(f"    [DRY-RUN] would send: {cmd.strip()}")
        return 'MOVE_DONE' if cmd[0] in ('F', 'B') else 'TURN_DONE'
    drain(ser, 0.1)
    ser.write(cmd.encode('ascii'))
    ser.flush()
    ack = wait_for_ack(ser, timeout_s=ack_timeout)
    return ack


def send_estop(ser):
    """Best-effort stop command."""
    try:
        ser.write(b'x')
        ser.flush()
    except Exception:
        pass


def pose_from_maze_frame(start_cell, heading_deg):
    """Initial pose in world coords — just for logging, not used for control."""
    return start_cell, heading_deg


HEADING_NAME_TO_IDX = {'N': N, 'E': E, 'S': S, 'W': W}


class Logger:
    """Write to stdout AND optionally to a file. Flushes every line so tail -f works."""
    def __init__(self, path: str | None = None):
        self.file = open(path, 'w', encoding='utf-8') if path else None
        if self.file:
            import time as _t
            self.file.write(f"# drive_maze run log, started {_t.strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.file.flush()

    def log(self, msg: str = ''):
        print(msg)
        if self.file:
            self.file.write(msg + '\n')
            self.file.flush()

    def close(self):
        if self.file:
            self.file.close()


def run(args):
    logger = Logger(args.log)
    log = logger.log

    # Build prior belief
    if args.prior:
        if args.prior not in MAZES:
            log(f"Unknown prior: {args.prior}. Options: {list(MAZES)}")
            sys.exit(1)
        builder, _, _ = MAZES[args.prior]
        belief = builder()
        log(f"Using prior map: {args.prior}")
    else:
        belief = Maze(args.rows, args.cols)
        log(f"No prior map. Belief starts with outer walls only ({args.rows}x{args.cols}).")

    start = tuple(args.start)
    goal = tuple(args.goal)
    pos = start
    heading = HEADING_NAME_TO_IDX[args.heading]
    cell_cm = args.cell_cm

    log(f"Start={start}, Goal={goal}, cell size={cell_cm}cm, start heading={DIR_NAMES[heading]}")
    log(f"Wall threshold={args.wall_thresh}m, turn penalty={args.turn_penalty}")

    # Connect
    if not args.dry_run:
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.05)
            log(f"Connected to {args.port} @ {args.baud}")
        except serial.SerialException as e:
            log(f"ERROR opening {args.port}: {e}")
            sys.exit(1)
    else:
        ser = None
        log("DRY-RUN mode: no serial port opened.")

    try:
        for step in range(args.max_steps):
            log(f"\n--- cell {step}: at {pos} heading={DIR_NAMES[heading]} ---")

            # 1. Sense
            if not args.dry_run:
                scan, odom = read_full_scan(ser, timeout_s=args.scan_timeout)
                if scan is None:
                    log("  ERROR: no scan received — is the car streaming? halting.")
                    break
                sensed = sense_walls(scan, heading_deg=HEADING_TO_DEG[heading],
                                     wall_threshold=args.wall_thresh)
                if odom:
                    log(f"  odom: x={odom.x_m:+.2f}m y={odom.y_m:+.2f}m θ={odom.theta_deg:+.1f}°  scan={len(scan)} beams")
            else:
                sensed = {d: belief.has_wall(pos[0], pos[1], d) for d in (N, E, S, W)}

            walls_str = ''.join(DIR_NAMES[d] for d in (N, E, S, W) if sensed[d])
            log(f"  sensed walls: [{walls_str or 'none'}]")
            update_belief(belief, pos, sensed)

            # 2. Goal check
            if pos == goal:
                log(f"  *** GOAL REACHED at {pos} in {step} cells ***")
                break

            # 3. Plan
            dist = flood_fill(belief, goal)
            if dist[pos] >= INF:
                print("  ERROR: goal unreachable given current belief. halting.")
                break

            next_dir = choose_next_move(belief, pos, dist, heading,
                                        turn_penalty=args.turn_penalty)
            if next_dir is None:
                print("  ERROR: no valid move. halting.")
                break
            log(f"  plan: go {DIR_NAMES[next_dir]} (dist at cur={dist[pos]}, at next={dist[pos[0]+DY[next_dir], pos[1]+DX[next_dir]]})")

            if args.once:
                log("  (--once specified, stopping here)")
                break

            # 4. Turn if needed
            _, turn_cmd = shortest_turn(heading, next_dir)
            if turn_cmd:
                log(f"  turning: {turn_cmd.strip()}")
                ack = send_cmd(ser, turn_cmd, ack_timeout=args.turn_timeout, dry_run=args.dry_run)
                if ack != 'TURN_DONE':
                    log(f"  ERROR: turn ack = {ack}, halting.")
                    break

            # 5. Forward one cell
            move_cmd = f"F{cell_cm}\n"
            log(f"  moving: {move_cmd.strip()}")
            ack = send_cmd(ser, move_cmd, ack_timeout=args.move_timeout, dry_run=args.dry_run)
            if ack != 'MOVE_DONE':
                log(f"  ERROR: move ack = {ack}, halting.")
                break

            # 6. Update our mental state
            heading = next_dir
            pos = (pos[0] + DY[next_dir], pos[1] + DX[next_dir])

            # Brief pause so lidar has time to settle between cells
            time.sleep(args.pause_between_cells)

        else:
            log(f"\nHIT MAX_STEPS={args.max_steps} without reaching goal.")
    except KeyboardInterrupt:
        log("\n\n(interrupted by user)")
    finally:
        if ser is not None:
            send_estop(ser)
            ser.close()
        logger.close()


def main():
    p = argparse.ArgumentParser(description='Drive MazeBot through a maze via Flood-Fill.')
    p.add_argument('--port', default='COM4')
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--start', nargs=2, type=int, required=True,
                   help='Start cell as two ints: "--start 4 0"')
    p.add_argument('--goal', nargs=2, type=int, required=True,
                   help='Goal cell: "--goal 2 4"')
    p.add_argument('--rows', type=int, default=5)
    p.add_argument('--cols', type=int, default=5)
    p.add_argument('--prior', default=None,
                   help='Name of a prebuilt maze (e.g. z_island_5x5) to use as prior belief')
    p.add_argument('--cell-cm', type=int, default=70, help='Cell size in cm (default 70)')
    p.add_argument('--wall-thresh', type=float, default=0.50,
                   help='Wall detection threshold in meters (default 0.50)')
    p.add_argument('--turn-penalty', type=int, default=2)
    p.add_argument('--scan-timeout', type=float, default=3.0)
    p.add_argument('--turn-timeout', type=float, default=6.0)
    p.add_argument('--move-timeout', type=float, default=12.0)
    p.add_argument('--max-steps', type=int, default=50)
    p.add_argument('--pause-between-cells', type=float, default=0.3)
    p.add_argument('--dry-run', action='store_true',
                   help="Don't actually send commands; simulate using belief as truth")
    p.add_argument('--once', action='store_true',
                   help='Sense one cell, print plan, then exit (no motion)')
    p.add_argument('--heading', default='N', choices=['N', 'E', 'S', 'W'],
                   help="Car's initial heading: N (+row), E (+col), S (-row), W (-col). Default N.")
    p.add_argument('--log', default=None,
                   help='Write a detailed run log to this file (in addition to stdout)')
    args = p.parse_args()

    run(args)


if __name__ == '__main__':
    main()
