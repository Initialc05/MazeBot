# Technical Report: Fully Embedded 2D-LiDAR Based Autonomous Mobile Robot

**Module:** EBU6475 Microprocessor Systems Design  
**Coursework:** End-of-Module Project  
**System:** 2D-LiDAR based Autonomous Mobile Robot for 5 x 5 maze navigation  
**Submission:** Technical Documentation  

## Abstract

This report documents the design, implementation and validation of a 2D-LiDAR based autonomous mobile robot for a 5 x 5 maze. The final runtime architecture is fully embedded: the STM32 NUCLEO-F446RE performs LiDAR processing, encoder/IMU odometry, occupancy-grid mapping, scan matching, topology update, A*/frontier planning, local obstacle checks, navigation state control, PID motor control, OLED display and emergency-stop protection. The Python Host is retained only for visualisation, logging and early development, not runtime navigation decisions. The report follows the CDIO cycle from requirement interpretation, through architecture and firmware implementation, to validation and limitations.

## Team Members and Responsibilities

| Name | Student ID | Main Responsibility |
|---|---:|---|
| Yifan Chen | 231221526 | Team leader; led STM32 firmware architecture, FreeRTOS task integration, UART/DMA communication, motor control and final system integration. |
| Cheng Wei | 231222822 | Automatic navigation; developed the STM32-side maze navigation logic, maze topology model, planning state machine and motion decision layer. |
| Mandan Dong | 231220714 | LiDAR visualisation and embedded mapping support; contributed to scan processing, telemetry display and saved-log workflow. |
| Maiyuan Cao | 231220253 | Sensor integration and calibration; supported LiDAR, IMU and encoder data checking, odometry calibration and telemetry validation. |
| Xiaoqin Liu | 231221571 | User interface, safety and documentation; contributed to OLED state display, button behaviour, E-stop workflow and report/video preparation. |
| Xingrui Li | 231220482 | Motor and encoder testing; supported PID tuning, benchmark preparation, code packaging and README organisation. |

## CDIO Evidence Map

The report is organised around the CDIO lifecycle. Conceive explains the fully embedded runtime boundary and maze constraints. Design covers Sense-Think-Act architecture, FreeRTOS scheduling, embedded topology, scan matching, PID control and safety. Implement describes firmware integration and STM32-side AutoNav. Operate presents staged validation, benchmark scoring and limitations. Reflection records how GenAI suggestions were checked against the brief, code and observed behaviour.

## 1. System Overview

The project objective was to design and implement a 2D-LiDAR based autonomous mobile robot capable of operating in a 5 x 5 maze, navigating from the Start cell to the Exit cell, and then returning to the Start cell. The robot platform used the STM32 NUCLEO-F446RE, RPLIDAR C1, wheel encoders, IMU, geared DC motors, AT8236 motor driver, HC-04 Bluetooth module, SSD1306 OLED display, push buttons and potentiometers.

The final system uses a fully embedded runtime architecture. The STM32 firmware implements sensing, localisation, mapping, planning, local obstacle handling and motor control. The Python Host is only a supervisory development tool for telemetry display, saved logs, map visualisation and debugging evidence; Bluetooth latency therefore cannot affect navigation decisions.

## 2. Conceive: Requirement Interpretation and Engineering Problem

The main engineering challenge was to combine sensing, mapping, decision-making and motor control into a system that could operate reliably under real-time constraints. The robot needed to sense maze walls using 2D LiDAR, estimate its pose using encoder and IMU information, construct a usable representation of the maze, decide where to move next, and execute each movement with enough accuracy to remain aligned to the cell grid.

The brief was interpreted as a fully embedded Sense-Think-Act problem. Sense covers LiDAR UART/DMA, wheel encoders and IMU heading. Think covers OGM update, pose confidence, 5 x 5 topology, wall/open-passage detection, Start-to-Exit and Exit-to-Start planning. Act covers precise forward/turn/stop commands through closed-loop motor control. User interaction is limited to OLED status, buttons, E-stop and potentiometer tuning.

The fixed 5 x 5 maze drove the main abstraction. Instead of continuous trajectory optimisation, the embedded planner reasons over cell-to-cell moves and validates each action using LiDAR clearance, wall distance, wall-angle error, heading error and scan-match confidence. This keeps planning compact enough for the STM32 while preserving a closed-loop decision process.

Key constraints were limited STM32 memory/CPU time, high LiDAR serial bandwidth, wheel slip, odometry drift, Bluetooth latency and the need for immediate E-stop response. These constraints led to five early decisions: use a cell-level maze abstraction; keep all runtime autonomy on the STM32; combine a small OGM with a 5 x 5 topology; keep the Host for observation only; and use binary Bluetooth telemetry with packet framing and checksum.

## 3. Design: System Architecture and Trade-Offs

### 3.1 Sense-Think-Act Architecture

The system is organised as a layered Sense-Think-Act pipeline. In Sense, the STM32 reads RPLIDAR C1 through UART5 DMA, encoder counters through TIM1/TIM3 hardware encoder mode and IMU heading through USART DMA. In Think, it updates a compact OGM, performs lightweight scan matching, estimates local corridor metrics, updates the 5 x 5 topology and chooses the next action. In Act, it executes alignment, one-cell movement, wall-adjust, recovery and stop states using the precise movement state machine and cascaded PID motor control.

The STM32 firmware was designed around FreeRTOS tasks:

| Task | Period | Priority | Purpose |
|---|---:|---|---|
| `CommandTask` | 1 ms | Above Normal | Reads Bluetooth DMA buffer and parses incoming motion commands. |
| `MotorControlTask` | 5 ms | Above Normal | Samples encoder deltas and updates cascaded PID motor control. |
| `IMU900Task` | 2 ms | Normal | Processes IMU packets and updates yaw/odometry state. |
| `LidarTask` | 2 ms | Normal | Parses LiDAR nodes, feeds embedded AutoNav metrics/OGM update, and sends telemetry packets for observation. |
| `AutoNavTask` | 50 ms | Normal | Runs embedded localisation, topology update, goal check, planning, local obstacle assessment and navigation state transitions. |
| `ButtonTask` | 20 ms | Normal | Debounces buttons and handles mode changes/E-stop reset. |
| `UITask` | 200 ms | Below Normal | Updates potentiometer values and refreshes the OLED display. |

The priorities keep command handling and motor control ahead of non-critical UI refresh.

### 3.2 STM32 Firmware Design

The firmware is split into focused modules: UART DMA and telemetry (`uart_device.c`), LiDAR parsing (`lidar.c`), encoder odometry (`encoder.c`), IMU parsing (`im948.c`), command/PID movement (`bt_cmd.c`), embedded navigation (`autonav.c`), motor GPIO/PWM (`motor.c`), buttons/E-stop (`button.c`, `robot_state.c`) and OLED/potentiometer UI (`ui_task.c`).

LiDAR points with low quality, distance below 0.20 m or distance above 1.50 m are rejected. Each valid point is also passed into `AutoNav_ObserveLidar()` on the STM32, where scan-sector minima and recent scan-point buffers are updated. At each scan boundary, `AutoNav_NotifyScanStart()` freezes a scan snapshot; AutoNav first scores this snapshot against the existing OGM for pose correction, then integrates the accepted snapshot into the occupancy grid. The same filtered point is packed with odometry and yaw into a binary packet with header `0xAA55` and XOR checksum for optional Host visualisation.

The encoder design uses hardware timer encoder mode rather than GPIO interrupts, reducing CPU load and missed-pulse risk. The left encoder uses TIM3 and the right encoder uses TIM1; counter deltas are sampled every 5 ms in the motor control task. These deltas are accumulated in pending odometry counters and consumed exactly once when a fresh IMU heading update arrives, preventing repeated use or loss of encoder deltas when the encoder and IMU tasks run at different rates.

### 3.3 Embedded AutoNav Design

The automatic navigation system is implemented in `MSD/Core/Src/autonav.c`. It is designed as a lightweight embedded demo rather than a desktop SLAM system. LiDAR measurements are processed on the STM32: the firmware updates angular clearance sectors, freezes recent scan snapshots at rotation boundaries, performs lightweight scan matching against the previously integrated OGM, then applies Bresenham-style ray updates into a compact occupancy grid. This avoids using the same raw scan both to create and to immediately validate its own map evidence.

For maze navigation, the STM32 maintains a discrete 5 x 5 topology. Each cell has four edge states (`UNKNOWN`, `OPEN` or `BLOCKED`) updated from local LiDAR clearance and OGM evidence. A* searches this topology, while frontier selection allows unknown edges with higher cost when no confirmed path is available. The perception layer supplies `forward_clearance_m`, side-wall distance, wall-angle error, heading error, centre error, scan-match confidence and junction score to the state machine.

The navigation state machine localises the start pose, observes the current cell, checks the goal using odometry cell, cell-centre distance and scan-match confidence, plans the next cell, aligns to the target heading, checks local clearance, advances or adjusts, then settles and updates topology. When Exit is reached, the same logic switches to Exit-to-Start return mode.

The main states are:

```text
LOCALIZE_START -> UPDATE_TOPOLOGY -> CHECK_GOAL -> PLAN
PLAN -> ALIGN_TO_EDGE -> ADVANCE_ONE_CELL
ADVANCE_ONE_CELL -> WALL_ADJUST / SETTLE_AND_MATCH / RECOVERY
SETTLE_AND_MATCH -> UPDATE_TOPOLOGY
```

At a junction, the STM32 scores candidate directions by Manhattan distance to the goal, turn cost, unknown-edge cost and confidence penalty, rather than following a fixed left/right rule. A straight cell move is only allowed when forward clearance is above 0.35 m, side clearance above 0.22 m, wall angle error below 10 degrees, heading error below 8 degrees and pose confidence is acceptable. If the robot is close to a wall or not parallel to the corridor, `WALL_ADJUST` computes a bounded correction from wall-angle and lateral-distance error before re-observing.

This structure was chosen because the benchmark maze is grid-based. The cell-level abstraction maps directly to the environment, while the embedded local metrics prevent the robot from blindly executing a planned path when sensor evidence shows poor alignment or insufficient clearance.

### 3.4 Motion Control Design

The STM32 motion controller supports continuous commands (`W`, `S`, `A`, `D`, `x`) and precise commands (`F`, `B`, `L`, `R` followed by a value). Embedded AutoNav uses the same precise movement and turn state machine by default, so navigation and manual calibration share the same closed-loop motor controller.

Motor control uses three PID loops: heading PID converts yaw error to target velocity difference, velocity-difference PID converts left-right encoder speed mismatch to PWM correction, and turn PID controls rotation duty. This cascaded design was chosen over open-loop PWM because the motors do not respond identically to the same duty cycle. Potentiometers tune base duty, turn duty and heading gain without recompilation.

### 3.5 Safety Design

The main safety mechanism is a latched emergency stop. When activated, `RobotState_LatchEstop()` sets the E-stop flag, moves the state to `ROBOT_ESTOP`, and directly clears the TIM2 PWM compare registers. This avoids relying on a lower-priority task to stop the motors. Once latched, the command parser rejects further motion commands; reset requires a deliberate button combination.

Additional safety behaviours include motor command timeout, forward-clearance stop, lateral-error stop, rotation/forward-motion timeout, safe stop command `x`, Bluetooth AutoNav stop command `N` and latched E-stop protection.

For the assessed firmware build, `AUTONAV_COMMAND_OUTPUT` is enabled by default, so AutoNav decisions are connected to the precise turn/move functions and the PID motor-control path. A separate explicit compile-time `AUTONAV_DRY_RUN` option exists only for bench logging without motor motion; it is not the default runtime configuration.

## 4. Implement: Realisation and Integration

The final implementation separates runtime autonomy from development support:

1. STM32 firmware in `MSD` performs sensing, mapping, localisation, planning, local obstacle checks and motor control at runtime.
2. The Host tools display telemetry, saved-map evidence and debug logs only.

The STM32 initialisation sequence configures GPIO, DMA, UARTs, timers, ADC and I2C, then starts UART, motor, encoder, robot state, potentiometer, Bluetooth command and AutoNav modules before FreeRTOS schedules the application tasks.

The LiDAR implementation was integrated with embedded navigation before telemetry transmission. Each valid point is filtered in `lidar.c`, passed to `AutoNav_ObserveLidar()` for local metric and scan-buffer updates, and then sent with `odom_x`, `odom_y` and `AngleZ` as optional telemetry. Sync markers identify full scan rotations and call `AutoNav_NotifyScanStart()` so the embedded navigation layer can stabilise sector clearances, freeze a scan snapshot for matching, and only then integrate the scan into the OGM.

The embedded navigation code is split across `autonav.h` for public API/states/metrics, `autonav.c` for OGM, scan matching, topology, A*/frontier planning and state control, `lidar.c` for scan feeding, `bt_cmd.c` for precise movement and commands `G/H/N`, `button.c` for physical start/return input, `main.c` for `AutoNavTask`, and `ui_task.c` for OLED AutoNav status. Runtime commands and telemetry have clear roles: `0xAA55` carries fused LiDAR/odometry telemetry to the Host, `0xBB55` carries pose-only telemetry, scan sync marks LiDAR rotations, `G`/START begins Start-to-Exit, `H`/RETURN begins Exit-to-Start, `N` stops AutoNav, and `F/B/L/R + value` is used by both manual calibration and AutoNav execution.

## 5. Operate: Testing, Validation and Performance Evaluation

The system was validated through staged testing. Embedded peripherals were tested individually: encoders by signed tick changes, PWM through manual commands, LiDAR through stable wall points in the Host visualiser, and Bluetooth through packet headers, checksums and scan sync markers.

Closed-loop motion was tuned by driving forward and rotating while observing heading stability and motor response. Heading and velocity-difference PID tuning reduced curved forward motion, while the turn PID reduced overshoot during 90-degree turns. Potentiometers were used to adjust base duty and turn duty during testing.

Mapping behaviour was then validated in the maze by checking whether the embedded OGM and local sector metrics responded consistently to straight walls, blocked directions and open junctions. Bluetooth visualisation was used only to observe the STM32's telemetry and `NAV:...` status messages.

Finally, autonomous navigation was tested in the benchmark maze. The STM32 AutoNav task updated wall topology, checked goal conditions, planned the next cell, assessed local safety metrics, selected straight/turn/adjust/recovery actions, issued precise motor commands, and re-observed the cell after settling. The same embedded system was used for Start-to-Exit and Exit-to-Start operation.

Submitted videos are used as primary visual evidence. They show encoder direction/tick accumulation, LiDAR wall points and telemetry reception, motor closed-loop forward/turning behaviour, OLED state display, E-stop/stop behaviour, STM32 `NAV:...` decision logs and the benchmark Start-to-Exit / Exit-to-Start attempt.

Benchmark scoring record:

| Metric | Recorded Time Used for Scoring | Maximum Allowed Time | Result |
|---|---:|---:|---|
| Start to Exit | 120 s | 120 s | Timeout / not credited |
| Exit to Start | 120 s | 120 s | Timeout / not credited |

Using the brief's linear scaling formula, each run scores `100 * (120 - T_measured) / (120 - 30)`. Because no faster repeatable timed run is recorded in this report, the conservative scoring value is 120 s for each direction, giving 0 for benchmark speed while still documenting the embedded execution path and the tuning gap.

## 6. Engineering Issues, Refinement and Limitations

**LiDAR noise and mapping artefacts.** Low-quality and near-field LiDAR points caused early map instability, so the firmware filters by quality/distance and uses sector minima rather than single points. Scan snapshots are matched before integration, and settle-and-match states reduce rotation artefacts before topology updates.

**Odometry drift.** Wheel slip, motor mismatch and wheel-diameter assumptions still accumulate error. Encoder scale calibration and IMU yaw improve short-term odometry, while lightweight scan matching over small `dx`, `dy` and `dtheta` candidates helps keep the estimated cell consistent.

**Communication and scheduling.** Dense text telemetry was replaced by compact binary packets with headers and checksums. UART DMA and FreeRTOS priorities keep command/motor-control tasks ahead of UI and debug output. Navigation decisions run on the STM32, so Bluetooth latency only affects observation.

The main limitation is that embedded AutoNav still needs more real-maze tuning for speed and repeatability. The wall-line estimate is lightweight rather than full ICP, and thresholds such as wall distance, match score and heading tolerance still require calibration. The default assessed build drives the precise motor-control functions; dry-run logging is only an explicit bench/debug compile option.

## 7. GenAI Use and Verification

GenAI was useful for separating navigation into topology, planner and low-level motion-control layers, which became `autonav.c` functions for metrics, OGM update, scan matching, topology, A*/frontier planning and state-machine control. It also helped compare open-loop duty correction with cascaded heading and velocity-difference PID, later checked against robot heading and encoder behaviour. A misleading suggestion was that navigation could remain on the PC while the STM32 only controlled motors; this was rejected after checking the brief's fully embedded requirement. The team rule was that no GenAI suggestion was accepted unless it connected to code, observed behaviour or a clear hardware/resource constraint.

## 8. Conclusion

The project produced an integrated 2D-LiDAR AMR with STM32-side sensing, odometry, OGM update, scan matching, topology planning, local obstacle checks, PID motor control, OLED interface, buttons and E-stop protection. The Host provides only optional telemetry visualisation and logging. The system demonstrates a fully embedded Sense-Think-Act workflow; future work should focus on threshold calibration, repeatability and faster benchmark runs.
