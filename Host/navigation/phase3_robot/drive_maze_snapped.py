"""
drive_maze_snapped.py — drive_maze.py + pose snapping (observer-only).

⚠️ IMPORTANT: we do NOT try to hard-stop the motors mid-move.
Reason (see memory: feedback_no_abrupt_stop.md):
  - Sending 'x' (Motor_Brake) at speed produces drift / skid / twist.
  - Even F<cm>'s own Motor_Brake at target is jarring, but it's more
    predictable because the car is at steady state when it triggers.

So the loop is:
    1. Sense walls on current cell.
    2. Flood-fill on belief, pick next cell.
    3. Turn if needed (L90 / R90), wait TURN_DONE.
    4. Send F<forward_cm>, wait MOVE_DONE (let firmware handle its own
       deceleration).
    5. Read the next odom packet, feed it to snapper, log where we ACTUALLY
       ended up (for diagnostic only — we still assume we reached the
       target cell).
    6. Pause a bit so the post-brake twist stabilizes.
    7. Loop.

The snapper is a *diagnostic observer* — it tells us drift/heading_err
after each move so we can see in the log if the car is drifting out of
cells. If it flags large drift we print a warning but keep going.

Usage:
    python drive_maze_snapped.py --start 4 0 --goal 0 4 --heading S \\
        --prior competition_5x5 --log run.log

    Calibration notes (if F is known to be off):
        --forward-cm 54     # if F70 actually travels ~90 cm, send F54
        --pause-between-cells 0.5   # let post-brake twist settle
        --wall-thresh 0.55  # slightly more tolerant walls if drift is high
"""
from __future__ import annotations
import sys
import os
import time
import argparse

_THIS_DIR = os.path.dirname(__file__)
_PHASE1 = os.path.join(_THIS_DIR, '..', 'flood_fill_sim')
_PHASE2 = os.path.join(_THIS_DIR, '..', 'phase2_sensing')
for p in (_PHASE1, _PHASE2):
    if p not in sys.path:
        sys.path.insert(0, p)

import serial
from bt_protocol import (read_full_scan, wait_for_ack, drain, read_one_packet,
                         ACK_MESSAGES)
from maze import Maze, MAZES, N, E, S, W, DIR_NAMES, DX, DY
from flood_fill import flood_fill, choose_next_move, INF
from wall_sense import sense_walls, update_belief
from pose_grid_snap import PoseGridSnapper

HEADING_TO_LIDAR_DEG = {N: 0, E: 270, S: 180, W: 90}
HEADING_NAME_TO_IDX = {'N': N, 'E': E, 'S': S, 'W': W}


class Logger:
    def __init__(self, path=None):
        self.file = open(path, 'w', encoding='utf-8') if path else None
        if self.file:
            self.file.write(f"# drive_maze_snapped log started {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.file.flush()

    def log(self, msg=''):
        print(msg)
        if self.file:
            self.file.write(msg + '\n')
            self.file.flush()

    def close(self):
        if self.file:
            self.file.close()


def shortest_turn(cur_heading, target_heading):
    delta = ((target_heading - cur_heading + 2) % 4) - 2
    if delta == 0:
        return 0, ''
    if delta == 1:
        return 1, 'R90\n'
    if delta == -1:
        return -1, 'L90\n'
    return 2, 'R180\n'


def send_cmd_wait_ack(ser, cmd, ack_timeout_s, log):
    """Send a line command, wait for its ACK. Return ACK string or None."""
    drain(ser, 0.1)
    ser.write(cmd.encode('ascii'))
    ser.flush()
    ack = wait_for_ack(ser, timeout_s=ack_timeout_s)
    log(f"    [ack] {ack}")
    return ack


def read_latest_odom(ser, max_wait_s=0.5):
    """Drain packets for max_wait_s, return the newest Odom we saw."""
    deadline = time.time() + max_wait_s
    last_odom = None
    while time.time() < deadline:
        pkt = read_one_packet(ser)
        if pkt is None:
            continue
        if isinstance(pkt, dict) and 'odom' in pkt:
            last_odom = pkt['odom']
    return last_odom


def run(args):
    logger = Logger(args.log)
    log = logger.log

    if args.prior:
        if args.prior not in MAZES:
            log(f"Unknown prior: {args.prior}. Options: {list(MAZES)}")
            sys.exit(1)
        builder, _, _ = MAZES[args.prior]
        belief = builder()
        log(f"Using prior map: {args.prior}")
    else:
        belief = Maze(args.rows, args.cols)
        log(f"No prior. Belief = outer walls only ({args.rows}x{args.cols}).")

    start = tuple(args.start)
    goal = tuple(args.goal)
    heading = HEADING_NAME_TO_IDX[args.heading]
    cell_cm = args.cell_cm
    forward_cm = args.forward_cm

    log(f"Start={start}, Goal={goal}, cell={cell_cm}cm, F per move={forward_cm}cm, heading={DIR_NAMES[heading]}")
    log(f"Wall threshold={args.wall_thresh}m, pause_between_cells={args.pause_between_cells}s")

    snapper = PoseGridSnapper(start[0], start[1], heading, cell_cm=cell_cm)
    pos = start

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
        log(f"Connected to {args.port} @ {args.baud}")
    except serial.SerialException as e:
        log(f"ERROR opening {args.port}: {e}")
        sys.exit(1)

    try:
        log("Waiting for first odom packet to anchor snapper origin...")
        odom0 = None
        for _ in range(500):
            pkt = read_one_packet(ser)
            if isinstance(pkt, dict) and 'odom' in pkt:
                odom0 = pkt['odom']
                break
        if odom0 is None:
            log("ERROR: never received an odom packet. Bluetooth alive?")
            return
        snapper.set_origin(odom0.x_m, odom0.y_m, odom0.theta_deg)
        log(f"Origin anchored: x={odom0.x_m:.3f} y={odom0.y_m:.3f} θ={odom0.theta_deg:.1f}")

        for step in range(args.max_steps):
            log(f"\n--- step {step}: at {pos} heading={DIR_NAMES[heading]} ---")

            # 1. Sense
            scan, odom = read_full_scan(ser, timeout_s=args.scan_timeout)
            if scan is None:
                log("  ERROR: no scan within timeout")
                break
            if odom:
                snap = snapper.update(odom.x_m, odom.y_m, odom.theta_deg)
                log(f"  odom: ({odom.x_m:+.2f}, {odom.y_m:+.2f}, {odom.theta_deg:+.1f}°)  "
                    f"snap=({snap.row},{snap.col},{DIR_NAMES[snap.heading]}) "
                    f"drift={snap.drift_cm:.1f}cm θerr={snap.heading_err_deg:.1f}°")
                if (snap.row, snap.col) != pos:
                    log(f"  ⚠️  snap disagrees with tracked pos {pos} — trusting tracked pos")
                if snap.drift_cm > 25:
                    log(f"  ⚠️  drift {snap.drift_cm:.0f}cm >25cm — car may be off-center")
                if snap.heading_err_deg > 25:
                    log(f"  ⚠️  heading error {snap.heading_err_deg:.0f}° >25° — car may be yawed")
            sensed = sense_walls(scan, heading_deg=HEADING_TO_LIDAR_DEG[heading],
                                 wall_threshold=args.wall_thresh)
            walls_str = ''.join(DIR_NAMES[d] for d in (N, E, S, W) if sensed[d])
            log(f"  sensed walls: [{walls_str or 'none'}]  beams={len(scan)}")
            update_belief(belief, pos, sensed)

            # 2. Goal?
            if pos == goal:
                log(f"  *** GOAL REACHED at {pos} in {step} steps ***")
                break

            # 3. Plan
            dist = flood_fill(belief, goal)
            if dist[pos] >= INF:
                log("  ERROR: goal unreachable. halting.")
                break
            next_dir = choose_next_move(belief, pos, dist, heading,
                                        turn_penalty=args.turn_penalty)
            if next_dir is None:
                log("  ERROR: no valid move.")
                break
            target_cell = (pos[0] + DY[next_dir], pos[1] + DX[next_dir])
            log(f"  plan: go {DIR_NAMES[next_dir]} to {target_cell} "
                f"(dist {dist[pos]}->{dist[target_cell]})")

            if args.once:
                log("  --once specified, exit.")
                break

            # 4. Turn if needed
            _, turn_cmd = shortest_turn(heading, next_dir)
            if turn_cmd:
                log(f"  turning: {turn_cmd.strip()}")
                ack = send_cmd_wait_ack(ser, turn_cmd,
                                        ack_timeout_s=args.turn_timeout, log=log)
                if ack != 'TURN_DONE':
                    log(f"  ERROR: turn ack={ack}, halting.")
                    break
                heading = next_dir
                time.sleep(args.pause_between_cells)

            # 5. Forward — send F, WAIT for MOVE_DONE (let firmware finish)
            move_cmd = f"F{forward_cm}\n"
            log(f"  moving: {move_cmd.strip()} (target cell {target_cell})")
            ack = send_cmd_wait_ack(ser, move_cmd,
                                    ack_timeout_s=args.move_timeout, log=log)
            if ack not in ('MOVE_DONE', 'MOVE_TIMEOUT'):
                log(f"  ERROR: move ack={ack}, halting.")
                break
            if ack == 'MOVE_TIMEOUT':
                log("  ⚠️  firmware reports MOVE_TIMEOUT — may have stalled")

            # 6. Pause for post-brake twist to settle
            time.sleep(args.pause_between_cells)

            # 7. Read latest odom (diagnostic)
            odom_after = read_latest_odom(ser, max_wait_s=0.5)
            if odom_after:
                snap_after = snapper.update(odom_after.x_m, odom_after.y_m,
                                            odom_after.theta_deg)
                log(f"  after move: snap=({snap_after.row},{snap_after.col},"
                    f"{DIR_NAMES[snap_after.heading]}) drift={snap_after.drift_cm:.1f}cm "
                    f"θerr={snap_after.heading_err_deg:.1f}°")
                if (snap_after.row, snap_after.col) != target_cell:
                    log(f"  ⚠️  ended up at ({snap_after.row},{snap_after.col}) "
                        f"but expected {target_cell}")

            # 8. Update mental position (assume we reached target regardless)
            pos = target_cell

        else:
            log(f"\nHIT MAX_STEPS={args.max_steps}")
    except KeyboardInterrupt:
        log("\n(interrupted)")
    finally:
        try:
            ser.write(b'x')
            ser.flush()
        except Exception:
            pass
        ser.close()
        logger.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='COM4')
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--start', nargs=2, type=int, required=True)
    p.add_argument('--goal', nargs=2, type=int, required=True)
    p.add_argument('--rows', type=int, default=5)
    p.add_argument('--cols', type=int, default=5)
    p.add_argument('--prior', default=None)
    p.add_argument('--cell-cm', type=int, default=70,
                   help='Logical cell size for snapper (cm). Default 70.')
    p.add_argument('--forward-cm', type=int, default=70,
                   help="F<cm> to send per cell. Tune this to calibrate: if F70 actually travels 90cm, set 54.")
    p.add_argument('--heading', default='N', choices=['N', 'E', 'S', 'W'])
    p.add_argument('--wall-thresh', type=float, default=0.50)
    p.add_argument('--turn-penalty', type=int, default=2)
    p.add_argument('--scan-timeout', type=float, default=3.0)
    p.add_argument('--turn-timeout', type=float, default=8.0)
    p.add_argument('--move-timeout', type=float, default=10.0)
    p.add_argument('--max-steps', type=int, default=30)
    p.add_argument('--pause-between-cells', type=float, default=0.5,
                   help="Pause after each turn/move to let post-brake motion settle.")
    p.add_argument('--once', action='store_true')
    p.add_argument('--log', default=None)
    args = p.parse_args()
    run(args)


if __name__ == '__main__':
    main()
