"""
PoseGridSnapper — snap continuous (x, y, heading) to a discrete maze grid.

Input: real-time odometry from firmware (cm/degrees) via the bluetooth
       fused packet stream (bt_protocol.Odom).
Output: which maze cell the robot is currently in, which way it's facing
        (snapped to the nearest N/E/S/W), and how far from cell center it is.

Design:
    - We do NOT read lidar_visualizer's SLAM position (too complex to
      share across processes, and SLAM has its own scaling issues).
    - We use the firmware's odom_x/odom_y/odom_theta directly (encoder
      fused with IMU). These are in cm and degrees respectively.
    - The "world origin" is wherever the robot was placed at startup.
      We install an initial offset so the START cell maps to its grid
      coordinates exactly.
    - We track the robot's travel in the START cell's local frame,
      then the user tells us "start = (4, 0), heading = S" and we do the
      math to convert.

Key ideas:
    - Odom from firmware gives us (x_m, y_m, theta_deg) in IMU's frame.
      That frame is: x points in the robot's initial forward direction,
      y points to its initial left. So if the car starts facing S, the
      IMU x-axis points south in the maze's world frame.
    - We need to map IMU coords -> maze coords (row, col).

Math:
    Let start_row, start_col = user-specified start cell.
    Let initial_heading = N/E/S/W (user specified).
    Cell size = CELL_CM cm.
    Cell centers live at (col * cell_cm + cell_cm/2, row * cell_cm + cell_cm/2)
    in some maze world frame where +x is E (+col), +y is N (+row).

    IMU delta (dx_imu, dy_imu, dθ) are relative to where the robot was
    when the fused stream started. We zero this at startup by recording
    the initial odom values as the "odom origin".

    Depending on initial_heading, the IMU's +x maps onto different maze
    world axes:
        initial=N: imu_x -> world_y, imu_y -> -world_x  (IMU left = world W)
        initial=E: imu_x -> world_x, imu_y -> world_y
        initial=S: imu_x -> -world_y, imu_y -> world_x
        initial=W: imu_x -> -world_x, imu_y -> -world_y

    After transforming we get (dx_world, dy_world) in meters, meaning how
    far the robot has moved along world-E and world-N from the start.
    Snap it to a grid:
        col = start_col + round(dx_world / cell_m)
        row = start_row + round(dy_world / cell_m)

    Robot's current absolute heading (world frame):
        world_theta_deg = initial_heading_deg + dθ_deg
    Snap to nearest 90°:
        heading_index = round(world_theta_deg / 90) mod 4
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional

N, E, S, W = 0, 1, 2, 3
DIR_NAMES = ('N', 'E', 'S', 'W')

# Maze world convention: +x = E, +y = N, heading angle 0° = facing N (+y),
# increases CCW (so 90° = W, 180° = S, 270° = E).
HEADING_TO_WORLD_DEG = {N: 0.0, E: 270.0, S: 180.0, W: 90.0}


@dataclass
class Snap:
    row: int
    col: int
    heading: int            # 0=N, 1=E, 2=S, 3=W
    drift_cm: float         # distance from nearest cell center (cm)
    heading_err_deg: float  # absolute angular error from nearest cardinal
    dx_world_m: float       # debug: cumulative world-frame displacement
    dy_world_m: float
    world_theta_deg: float  # debug: robot's absolute heading in world frame


class PoseGridSnapper:
    def __init__(self, start_row: int, start_col: int, initial_heading: int,
                 cell_cm: float = 70.0):
        self.start_row = start_row
        self.start_col = start_col
        self.initial_heading = initial_heading
        self.cell_cm = cell_cm
        self.cell_m = cell_cm / 100.0

        self._origin: Optional[tuple[float, float, float]] = None  # (x0, y0, theta0)
        self._last: Optional[Snap] = None

    def set_origin(self, odom_x_m: float, odom_y_m: float, odom_theta_deg: float):
        """
        Call this once on the first odom sample of the session.
        Declares "this is where the robot was placed at start".
        """
        self._origin = (odom_x_m, odom_y_m, odom_theta_deg)

    def update(self, odom_x_m: float, odom_y_m: float, odom_theta_deg: float) -> Snap:
        """
        Feed the latest odom reading. Returns the snapped grid location.
        Automatically sets origin on the first call if not set yet.
        """
        if self._origin is None:
            self.set_origin(odom_x_m, odom_y_m, odom_theta_deg)

        x0, y0, th0 = self._origin
        dx_imu = odom_x_m - x0
        dy_imu = odom_y_m - y0
        dth = odom_theta_deg - th0

        # Map IMU frame to maze world frame based on initial heading
        ih = self.initial_heading
        if ih == N:
            # car initially facing N: imu_x = world_y, imu_y = -world_x
            dx_world = -dy_imu
            dy_world = dx_imu
        elif ih == E:
            dx_world = dx_imu
            dy_world = dy_imu
        elif ih == S:
            dx_world = dy_imu
            dy_world = -dx_imu
        elif ih == W:
            dx_world = -dx_imu
            dy_world = -dy_imu
        else:
            raise ValueError(f"Bad initial heading: {ih}")

        # Snap position
        col_f = self.start_col + dx_world / self.cell_m
        row_f = self.start_row + dy_world / self.cell_m
        col = int(round(col_f))
        row = int(round(row_f))

        # Drift: distance from nearest cell center, in cm
        col_err_m = (col_f - col) * self.cell_m
        row_err_m = (row_f - row) * self.cell_m
        drift_cm = math.hypot(col_err_m, row_err_m) * 100

        # Snap heading: world_theta_deg = initial_world_heading + dth (positive = CCW)
        world_theta = HEADING_TO_WORLD_DEG[ih] + dth
        # Normalize to [0, 360)
        world_theta = world_theta % 360.0
        # Round to nearest 90°
        idx_f = world_theta / 90.0
        idx = int(round(idx_f)) % 4
        # Map world-theta quadrant -> heading enum:
        #   0° = N, 90° = W, 180° = S, 270° = E   (CCW)
        quadrant_to_heading = {0: N, 1: W, 2: S, 3: E}
        heading = quadrant_to_heading[idx]
        # Heading error: how far from the nearest cardinal
        heading_err = abs(world_theta - idx * 90.0)
        if heading_err > 45:
            heading_err = abs(heading_err - 90)

        snap = Snap(row=row, col=col, heading=heading, drift_cm=drift_cm,
                    heading_err_deg=heading_err, dx_world_m=dx_world,
                    dy_world_m=dy_world, world_theta_deg=world_theta)
        self._last = snap
        return snap

    def get_last(self) -> Optional[Snap]:
        return self._last


# ---- quick tests (run this file directly) ----

if __name__ == '__main__':
    def assert_snap(s, row, col, heading, msg=''):
        assert s.row == row and s.col == col and s.heading == heading, \
            f"{msg}: expected ({row},{col},{DIR_NAMES[heading]}), got ({s.row},{s.col},{DIR_NAMES[s.heading]})"
        print(f"  OK: {msg} -> ({s.row},{s.col}) heading={DIR_NAMES[s.heading]} drift={s.drift_cm:.1f}cm")

    # Test 1: start facing S at (4,0). Walk forward 70cm in IMU frame (meaning: south in world).
    print("Test 1: start (4,0) heading S, walk 70cm forward")
    snapper = PoseGridSnapper(4, 0, S, cell_cm=70)
    s = snapper.update(0, 0, 180)
    assert_snap(s, 4, 0, S, "initial")
    s = snapper.update(0.70, 0, 180)
    assert_snap(s, 3, 0, S, "after F70")
    s = snapper.update(1.40, 0, 180)
    assert_snap(s, 2, 0, S, "after F140")

    # Test 2: then turn left 90° (CCW, so world heading goes from S to E)
    print("Test 2: after 2 forwards, turn left 90°")
    s = snapper.update(1.40, 0, 270)
    assert_snap(s, 2, 0, E, "after L90")

    # Test 3: now IMU's +x is still world-south (because initial was S), but
    # robot is physically pointing E. Walking forward 70cm means moving east,
    # so IMU sees dy_imu = 70cm (it moved 'left' from its initial perspective).
    # Let me think again...
    # Actually odom_x/y is in IMU's body-fixed frame AT EACH MOMENT? or at start?
    # Looking at bt_cmd.c: odom_x/y come from encoder integration, which tracks
    # how much each wheel moved. These should be in a FIXED frame (IMU's initial).
    # In the IMU frame where initial x = forward, initial y = left:
    #   robot initially facing N, goes forward: x increases
    #   robot at initial S, turns L (CCW) 90°, now facing E (world)
    #   robot now moves forward (east) -> in IMU world frame, that's +y direction
    #     (because left from initial-S is east-ish... no wait, initial-S means
    #      IMU +x = south in world, IMU +y = east in world (left of south))
    # So after turning L90 from S->E, going forward means +x in world (east).
    # And world-east = IMU +y. So the next forward should put dy_imu += 0.70.
    # The snapper transforms dy_imu (IMU-left) -> dx_world = dy_imu when initial=S.
    # Wait I had: initial=S: dx_world = dy_imu, dy_world = -dx_imu. Yes.
    # So if robot at IMU pos (1.40, 0), turns then walks east, new IMU pos becomes
    # (1.40, 0.70). dx_world = 0.70 -> col moves from 0 to +1. Good.
    s = snapper.update(1.40, 0.70, 270)
    assert_snap(s, 2, 1, E, "forward after L90")

    print("\nAll tests passed")
