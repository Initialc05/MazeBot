# Technical Report: Fully Embedded 2D-LiDAR Based Autonomous Mobile Robot

**Module:** EBU6475 Microprocessor Systems Design  
**Coursework:** End-of-Module Project  
**System:** 2D-LiDAR based Autonomous Mobile Robot for 5 x 5 maze navigation  
**Submission:** Technical Documentation  

## Abstract

This report documents the design, implementation and validation of a 2D-LiDAR based autonomous mobile robot for the EBU6475 End-of-Module project. The final architecture is fully embedded for runtime autonomy: the STM32 NUCLEO-F446RE performs LiDAR processing, encoder/IMU odometry, occupancy-grid mapping, scan matching, 5 x 5 maze topology update, A*/frontier planning, local obstacle checks, navigation state control, motor PID control, OLED UI, button handling and emergency-stop protection. A Python Host application is retained only for visualisation, telemetry logging, debugging and early algorithm development; it does not perform runtime navigation decisions.

The report follows the CDIO lifecycle: Conceive defines the engineering problem and constraints; Design justifies architecture and trade-offs; Implement explains embedded firmware integration; Operate describes validation, benchmark evidence and limitations. The main outcome is a fully embedded Sense-Think-Act navigation demo in which sensing, thinking and acting are all executed on the STM32.

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

| CDIO Phase | Evidence Presented in This Report | Main Assessment Criterion Addressed |
|---|---|---|
| Conceive | Requirement interpretation, resource constraints, benchmark objective and fully embedded STM32 runtime boundary. | Critical assessment of the project challenge. |
| Design | Sense-Think-Act architecture, FreeRTOS task structure, embedded maze topology, scan matching, PID control and safety design. | Higher-order technical reasoning and design trade-off justification. |
| Implement | STM32 firmware modules, embedded navigation module, telemetry packet design and integration workflow. | System realisation, subsystem coordination and technical execution. |
| Operate | Staged validation, video evidence, benchmark placeholders, engineering issues and refinement. | Verification, validation and performance evaluation. |
| Reflection | Fully embedded autonomy decision, GenAI verification and engineering limits of the current demo. | Responsible reflection and self-assessment. |

## 1. System Overview

The project objective was to design and implement a 2D-LiDAR based autonomous mobile robot capable of operating in a 5 x 5 maze, navigating from the Start cell to the Exit cell, and then returning to the Start cell. The robot platform used the STM32 NUCLEO-F446RE, RPLIDAR C1, wheel encoders, IMU, geared DC motors, AT8236 motor driver, HC-04 Bluetooth module, SSD1306 OLED display, push buttons and potentiometers.

Our final system uses a fully embedded runtime architecture. The STM32 firmware implements LiDAR acquisition, local scan-sector processing, encoder odometry, IMU heading processing, compact occupancy-grid mapping, lightweight scan matching, 5 x 5 maze topology update, A*/frontier planning, local clearance and wall-alignment checks, autonomous navigation state control, Bluetooth telemetry, command parsing, motor PID control, OLED display, button handling and emergency-stop safety. A Python Host application is used only as a supervisory development tool for telemetry display, saved logs, map visualisation and debugging evidence. It is not part of the runtime navigation decision loop.

This architecture matches the brief's fully embedded target: sensing, localisation, mapping, planning, local obstacle handling and control are executed on the STM32. Bluetooth telemetry remains useful for observing the engineering process, but the robot does not rely on a PC for runtime navigation.

## 2. Conceive: Requirement Interpretation and Engineering Problem

The main engineering challenge was to combine sensing, mapping, decision-making and motor control into a system that could operate reliably under real-time constraints. The robot needed to sense maze walls using 2D LiDAR, estimate its pose using encoder and IMU information, construct a usable representation of the maze, decide where to move next, and execute each movement with enough accuracy to remain aligned to the cell grid.

The minimum functional requirements from the brief were interpreted as follows:

| Requirement Area | Project Interpretation |
|---|---|
| Sense | Acquire LiDAR scan data through UART/DMA, acquire wheel encoder data through hardware timer encoder mode, and combine each LiDAR point with odometry/heading information before telemetry transmission. |
| Think | Maintain a 2D occupancy grid and a 5 x 5 maze topology, estimate the current cell and heading, identify walls/open passages, plan toward the Exit and then back to Start. |
| Act | Execute forward, backward, left-turn, right-turn and stop commands using closed-loop heading and velocity-difference PID control. |
| User Interface | Provide OLED status visibility, button-based mode control and emergency-stop handling, and potentiometer-based tuning for motion parameters. |
| Real-Time Architecture | Use FreeRTOS tasks, DMA, timer peripherals and prioritised task scheduling to separate sensing, command handling, control and UI work. |
| Validation | Demonstrate operation through video evidence, measured benchmark time, visible robot behaviour and repeatable command/control response. |

The robot was required to work inside a 5 x 5 maze. This drove the navigation abstraction: instead of planning continuous arbitrary trajectories, the embedded planner reasons over cell-to-cell moves and then validates each action using local LiDAR clearance, wall distance, wall-angle error, heading error and scan-match confidence. This reduced the complexity of path planning while still making each movement a closed-loop Sense-Think-Act decision.

Key engineering constraints were:

- Limited STM32 memory and CPU time for embedded mapping and planning.
- High LiDAR serial data rate and risk of data loss if handled with blocking reads.
- Wheel slip and mechanical mismatch between the two motors.
- Odometry drift during long movement sequences.
- Bluetooth latency and packet loss risk during dense telemetry, which must not affect navigation because decisions are local to the STM32.
- Need for immediate motor shutdown during emergency-stop conditions.

The main design decisions made during the Conceive phase are summarised below:

| Decision | Reasoning | Risk Accepted |
|---|---|---|
| Use a cell-level maze abstraction rather than continuous trajectory planning. | The benchmark maze is a fixed 5 x 5 grid, so cell-to-cell planning directly matches the task. | Fine continuous path optimisation is limited. |
| Keep the full Sense-Think-Act loop on the STM32. | The brief requires runtime sensing, mapping, planning and control to be embedded, not PC-assisted. | Requires a compact map and lightweight planning algorithms. |
| Use a small embedded OGM plus 5 x 5 topology. | This captures the benchmark maze structure without storing a large desktop-style map. | Resolution and scan matching are intentionally lightweight. |
| Keep the Host for telemetry and visualisation only. | The Host is useful evidence for debugging, logging and demonstrations. | Host displays may lag without affecting robot decisions. |
| Use binary Bluetooth telemetry. | LiDAR produces dense data, so text telemetry would waste bandwidth and increase latency. | Requires packet framing and checksum verification. |

## 3. Design: System Architecture and Trade-Offs

### 3.1 Sense-Think-Act Architecture

The system is organised as a layered Sense-Think-Act pipeline:

| Layer | Implementation |
|---|---|
| Sense | STM32 reads RPLIDAR C1 through UART5 DMA, reads encoder counters using TIM1/TIM3 hardware encoder mode, and processes IMU heading data through USART DMA. |
| Think | STM32 updates a compact occupancy grid, performs lightweight scan matching, estimates local corridor metrics, updates the 5 x 5 maze topology, runs A*/frontier planning and selects the next local action. |
| Act | STM32 executes alignment, one-cell advance, wall-adjust, recovery and stop states using the existing precise movement state machine and cascaded PID motor control. |

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

The firmware is divided into modules:

- `uart_device.c`: UART DMA reception, circular buffer reading and Bluetooth/debug transmission.
- `lidar.c`: RPLIDAR command handling, 5-byte scan node parsing, quality/distance filtering, embedded AutoNav scan feeding and fused telemetry packet generation.
- `encoder.c`: TIM1/TIM3 hardware encoder sampling, tick accumulation, speed estimation and odometry update.
- `im948.c`: IMU packet parsing, yaw angle update and odometry packet support.
- `bt_cmd.c`: Bluetooth command parser, movement state machine and cascaded PID control.
- `autonav.c`: fully embedded navigation pipeline: local LiDAR metrics, compact OGM, scan-match scoring, maze topology update, A*/frontier planning, local obstacle policy, goal validation and decision logging.
- `motor.c`: PWM and motor direction GPIO control.
- `button.c`: debounced buttons and emergency-stop reset handling.
- `robot_state.c`: global robot state and latched E-stop state.
- `ui_task.c`: OLED status display and potentiometer monitoring.

LiDAR points with low quality, distance below 0.20 m or distance above 1.50 m are rejected. Each valid point is also passed into `AutoNav_ObserveLidar()` on the STM32, where scan sectors, recent scan points and occupancy-grid rays are updated. The same point is packed with odometry and yaw into a binary packet with header `0xAA55` and XOR checksum for optional Host visualisation.

The encoder design uses hardware timer encoder mode rather than GPIO interrupts, reducing CPU load and missed-pulse risk. The left encoder uses TIM3 and the right encoder uses TIM1; counter deltas are sampled every 5 ms in the motor control task.

### 3.3 Embedded AutoNav Design

The automatic navigation system is implemented in `MSD/Core/Src/autonav.c`. It is designed as a lightweight embedded demo rather than a desktop SLAM system. Each LiDAR measurement is processed on the STM32: the firmware updates angular clearance sectors, stores recent scan points, performs Bresenham-style ray updates into a compact occupancy grid, and keeps the latest scan-match score for pose confidence.

For maze navigation, the STM32 maintains a discrete 5 x 5 topology:

- Each maze cell is represented by row and column.
- Each edge has a wall state: `UNKNOWN`, `OPEN` or `BLOCKED`.
- Observed wall states are updated from local LiDAR clearance and occupancy-grid evidence.
- A path is planned using an A*-style search over the topology.
- If no confirmed path exists, the planner chooses a frontier direction and allows traversal through unknown edges with higher cost.

The embedded perception layer estimates navigation metrics used by the state machine:

| Metric | Purpose |
|---|---|
| `forward_clearance_m` | Stops or delays forward motion if an obstacle is too close. |
| `left_wall_dist_m`, `right_wall_dist_m` | Detects side-wall proximity and lateral imbalance. |
| `nearest_wall_dist_m` | Triggers wall-adjust behaviour before a cell move. |
| `wall_angle_error_deg` | Estimates whether the robot is parallel to corridor walls. |
| `heading_error_deg` | Checks alignment with the planned cardinal direction. |
| `center_error_m` | Checks whether the robot is near the current cell centre. |
| `match_score`, `match_accepted` | Indicates whether scan matching supports the odometry pose. |
| `junction_score` | Detects junction-like cells where direction choice matters. |

The navigation state machine follows this sequence:

1. Localise the start pose and set a reference heading.
2. Observe walls around the current cell.
3. Check whether the current target has been reached using odometry cell, cell-centre distance and scan-match confidence.
4. Plan the next cell-to-cell step using A*/frontier logic.
5. Rotate to the target heading if required.
6. Before advancing, check forward clearance, side-wall distance, wall angle error and heading error.
7. Advance one cell if the corridor quality is good, otherwise perform `WALL_ADJUST` or `RECOVERY`.
8. Settle, scan-match and update topology.
9. Repeat until Exit is reached, then switch to return mode.

The main states are:

```text
LOCALIZE_START -> UPDATE_TOPOLOGY -> CHECK_GOAL -> PLAN
PLAN -> ALIGN_TO_EDGE -> ADVANCE_ONE_CELL
ADVANCE_ONE_CELL -> WALL_ADJUST / SETTLE_AND_MATCH / RECOVERY
SETTLE_AND_MATCH -> UPDATE_TOPOLOGY
```

At a junction, the STM32 scores candidate directions by Manhattan distance to the goal, turn cost, unknown-edge cost and confidence penalty. This means the robot does not follow a fixed left-hand or right-hand rule; it selects the route that best supports Start-to-Exit or Exit-to-Start navigation.

Local obstacle handling is intentionally lightweight but explainable. A straight cell move is allowed only when:

```text
forward_clearance_m > 0.35
nearest_wall_dist_m > 0.22
abs(wall_angle_error_deg) < 10
abs(heading_error_deg) < 8
scan matching is accepted, or recent odometry/scan data are stable enough
```

If the robot is close to a wall or not parallel to the corridor, `WALL_ADJUST` computes a bounded correction angle from wall-angle error and lateral wall-distance error. The state then performs a small alignment correction, optionally a short forward correction, and returns to scan matching before attempting the planned cell move again.

This structure was chosen because the benchmark maze is grid-based. The cell-level abstraction maps directly to the environment, while the embedded local metrics prevent the robot from blindly executing a planned path when sensor evidence shows poor alignment or insufficient clearance.

### 3.4 Motion Control Design

The STM32 motion controller supports continuous commands (`W`, `S`, `A`, `D`, `x`) and precise commands (`F`, `B`, `L`, `R` followed by a value). Embedded AutoNav uses the same precise movement and turn state machine when command output is enabled, so navigation and manual calibration share the same closed-loop motor controller.

The motor control uses three PID loops:

- Heading PID: yaw error to target velocity difference.
- Velocity-difference PID: left-right encoder speed difference to PWM correction.
- Turn PID: yaw error to turn duty during rotation.

This cascaded design was selected because the two motors do not respond identically to the same duty cycle. The outer heading loop keeps the robot aligned, while the inner velocity-difference loop compensates for wheel mismatch. Potentiometers tune base duty, turn duty and heading gain without recompilation.

The control decision was made after comparing three options:

| Option | Advantage | Disadvantage | Final Decision |
|---|---|---|---|
| Open-loop fixed PWM | Simple and easy to implement. | Robot curves when motors are mismatched; poor repeatability. | Rejected for autonomous navigation. |
| Single heading PID only | Corrects yaw error. | Does not directly compensate left-right wheel speed mismatch. | Used only as part of a larger controller. |
| Cascaded heading and velocity-difference PID | Corrects heading and wheel mismatch together. | Requires tuning and reliable encoder sampling. | Adopted in `bt_cmd.c`. |

### 3.5 Safety Design

The main safety mechanism is a latched emergency stop. When activated, `RobotState_LatchEstop()` sets the E-stop flag, moves the state to `ROBOT_ESTOP`, and directly clears the TIM2 PWM compare registers. This avoids relying on a lower-priority task to stop the motors. Once latched, the command parser rejects further motion commands; reset requires a deliberate button combination.

Additional safety behaviours include:

- Motor command timeout.
- Forward-clearance stop in the embedded AutoNav local policy.
- Lateral-error stop if the robot deviates too far during a cell move.
- Motion timeout for rotation and forward movement.
- Safe stop command `x`, Bluetooth AutoNav stop command `N`, and latched E-stop protection.

During early demonstration, `AUTONAV_COMMAND_OUTPUT` is set to `0`. In this mode the STM32 still runs the complete Sense-Think-Act decision pipeline and outputs `NAV:...` status logs, but it does not drive the motors automatically. This makes the strategy layer safe to demonstrate before enabling physical autonomous motion. Setting `AUTONAV_COMMAND_OUTPUT` to `1` connects the same decisions to the existing precise movement and PID control functions.

## 4. Implement: Realisation and Integration

The final implementation separates runtime autonomy from development support:

1. STM32 firmware in `MSD` performs sensing, mapping, localisation, planning, local obstacle checks and motor control at runtime.
2. The Host tools display telemetry, saved-map evidence and debug logs only.

The STM32 initialisation sequence configures GPIO, DMA, UARTs, timers, ADC and I2C, then starts UART, motor, encoder, robot state, potentiometer, Bluetooth command and AutoNav modules before FreeRTOS schedules the application tasks.

The LiDAR implementation was integrated with embedded navigation before telemetry transmission. Each valid point is filtered in `lidar.c`, passed to `AutoNav_ObserveLidar()` for local metric and OGM updates, and then sent with `odom_x`, `odom_y` and `AngleZ` as optional telemetry. Sync markers identify full scan rotations and call `AutoNav_NotifyScanStart()` so the embedded navigation layer can stabilise sector clearances.

The embedded automatic navigation module is split into focused C components and functions:

| File | Responsibility |
|---|---|
| `autonav.h` | Public AutoNav API, states, modes, metrics and `AUTONAV_COMMAND_OUTPUT` safety switch. |
| `autonav.c` | Embedded OGM, scan matching, topology update, A*/frontier planning, local action policy and navigation state machine. |
| `lidar.c` | Feeds valid LiDAR points into AutoNav and still emits telemetry for observation. |
| `bt_cmd.c` | Provides the precise movement/turn state machine and Bluetooth AutoNav commands `G`, `H` and `N`. |
| `button.c` | Starts Start-to-Exit or Exit-to-Start navigation from physical buttons. |
| `main.c` | Creates `AutoNavTask` with a 50 ms decision period. |
| `ui_task.c` | Displays AutoNav state, cell and scan-match score on the OLED when navigation is active. |

This modular design keeps the coursework-critical navigation evidence inside the STM32 firmware while still allowing telemetry to be observed from the Host.

The implementation also includes explicit data contracts for telemetry, manual control and embedded AutoNav mode selection:

| Data/Command | Direction | Purpose |
|---|---|---|
| LiDAR fused packet `0xAA55` | STM32 to Host | Optional telemetry carrying LiDAR angle, distance, quality, odometry position and heading. |
| Odometry packet `0xBB55` | STM32 to Host | Optional pose telemetry when no LiDAR point is being processed. |
| Scan sync marker | STM32 internal + telemetry | Marks the boundary between LiDAR scan rotations and stabilises AutoNav sector metrics. |
| `G` / START button | User to STM32 | Starts embedded Start-to-Exit navigation. |
| `H` / RETURN button | User to STM32 | Starts embedded Exit-to-Start return navigation. |
| `N` | User to STM32 | Stops embedded AutoNav and returns to idle. |
| `F/B/L/R + value` | Manual/AutoNav to STM32 motor state machine | Precise distance or angle commands used by calibration and, when enabled, AutoNav execution. |

## 5. Operate: Testing, Validation and Performance Evaluation

The system was validated through staged testing. Embedded peripherals were tested individually: encoders by signed tick changes, PWM through manual commands, LiDAR through stable wall points in the Host visualiser, and Bluetooth through packet headers, checksums and scan sync markers.

Closed-loop motion was tuned by driving forward and rotating while observing heading stability and motor response. Heading and velocity-difference PID tuning reduced curved forward motion, while the turn PID reduced overshoot during 90-degree turns. Potentiometers were used to adjust base duty and turn duty during testing.

Mapping behaviour was then validated in the maze by checking whether the embedded OGM and local sector metrics responded consistently to straight walls, blocked directions and open junctions. Bluetooth visualisation was used only to observe the STM32's telemetry and `NAV:...` status messages.

Finally, autonomous navigation was tested in the benchmark maze. The STM32 AutoNav task updated wall topology, checked goal conditions, planned the next cell, assessed local safety metrics, selected straight/turn/adjust/recovery actions, and re-observed the cell after settling. The same embedded system was used for Start-to-Exit and Exit-to-Start operation.

The validation strategy is summarised below. The submitted videos are used as primary visual evidence instead of screenshots or separate log figures.

| Validation Target | Method | Evidence Source |
|---|---|---|
| Encoder direction and tick accumulation | Manual wheel rotation and movement commands. | Product Quality Video / live demonstration. |
| LiDAR acquisition | Host visualiser shows real-time wall points while STM32 feeds AutoNav metrics and OGM updates. | Product Quality Video. |
| Bluetooth packet link | Host receives fused LiDAR/odometry packets and `NAV:...` logs for observation. | Product Quality Video. |
| Motor closed-loop behaviour | Forward and turning commands show corrected heading and stable response. | Product Quality Video. |
| UI and safety | OLED displays state/pose/tuning values; E-stop and stop command disable motion. | Product Quality Video. |
| Autonomous navigation | STM32 AutoNav plans, explains and executes or dry-runs cell-to-cell decisions through the maze. | Product Quality Video and Benchmark Video. |
| Benchmark timing | Continuous Start-to-Exit and Exit-to-Start recording with readable timer. | Benchmark Video. |

Benchmark results are left blank here for completion after final testing:

| Metric | Measured Time | Maximum Allowed Time | Result |
|---|---:|---:|---|
| Start to Exit | ___ s | 120 s | ___ |
| Exit to Start | ___ s | 120 s | ___ |

The benchmark score can be calculated using the brief's linear scaling formula after inserting the measured times.

For the report submission, compute each score as `100 * (120 - T_measured) / (120 - 30)` and average the Start-to-Exit and Exit-to-Start scores.

## 6. Engineering Issues, Refinement and Limitations

**LiDAR noise and near-distance distortion.** Early mapping results were sensitive to low-quality points and near-field distortion. The firmware therefore filters LiDAR points by quality and distance before they enter AutoNav. Local decisions use sector minima rather than a single raw point, reducing the chance that one noisy point causes a wrong stop or wall update.

**Odometry drift.** Wheel encoder odometry accumulates error because of wheel slip, motor mismatch and imperfect wheel diameter assumptions. The firmware uses an encoder scale factor and IMU yaw to improve short-term odometry. AutoNav then applies lightweight scan matching over small `dx`, `dy` and `dtheta` candidates against the embedded OGM before snapping the robot to the nearest maze cell.

**Rotation artefacts in mapping.** When the robot rotates while scanning, LiDAR points from one scan correspond to different robot headings. The embedded implementation reduces the effect by using recent scan points, sector-based clearances and settle-and-match states after movement before updating topology.

**Bluetooth throughput and latency.** Text telemetry was not suitable for dense LiDAR data. The implementation therefore uses compact binary packets with headers and checksums. UART reception on the STM32 uses DMA circular buffers to avoid blocking the real-time tasks. Navigation decisions no longer depend on Bluetooth latency because planning and local obstacle checks run on the STM32.

**Task scheduling.** OLED display updates and debug output must not interfere with control timing. The final FreeRTOS design places command and motor-control tasks at higher priority, while the UI task runs at lower priority and slower frequency.

The main limitation is that the embedded AutoNav is a strategy-layer demo rather than a fully tuned competition controller. The wall-line estimate is lightweight rather than full ICP, and thresholds such as wall distance, match score and heading tolerance still require real-maze calibration. For safe demonstration, `AUTONAV_COMMAND_OUTPUT` is currently disabled by default; enabling it connects the same STM32 decisions to the precise motor control functions.

## 7. GenAI Use and Verification

One useful GenAI output was the suggestion to separate the navigation software into a discrete topology layer, a maze planner and a low-level motion controller. This helped the team avoid mixing map interpretation, path planning and motor command logic in one uncontrolled block. The final STM32 code reflects this separation through `autonav.c` functions for local metrics, OGM update, scan matching, topology update, A*/frontier planning, local action evaluation and state-machine control. The suggestion was verified by checking that each planned topology edge corresponded to one physical movement primitive and by testing the state transitions through `NAV:...` logs.

One misleading GenAI output was the suggestion that navigation could remain on the PC while the STM32 only handled motor control. After checking the coursework brief, this was corrected because the required Sense-Think-Act runtime must be fully embedded. The design was refined so the STM32 performs LiDAR processing, mapping, localisation, planning, local obstacle handling and control, while the Host is limited to telemetry visualisation and logging.

Another useful GenAI-supported discussion was PID tuning strategy. GenAI helped compare a single open-loop duty correction approach with a cascaded heading and velocity-difference controller. The team then verified the idea against the actual firmware by observing whether the robot maintained heading during forward motion and whether the encoder speed difference decreased after tuning.

The verification rule was that no GenAI suggestion was accepted unless it could be connected to code, observed robot behaviour, or a clear hardware/resource constraint.

## 8. Conclusion

The project produced an integrated 2D-LiDAR based AMR system capable of sensing the maze, mapping the environment, planning cell-level navigation, and executing movement through STM32-based real-time motor control. The STM32 firmware provides UART/DMA sensing, encoder and IMU processing, embedded OGM update, scan matching, maze topology planning, local obstacle checks, PID motor control, OLED interface, button handling and emergency-stop protection. The Host provides only optional telemetry visualisation and logging.

The system demonstrates a complete fully embedded Sense-Think-Act workflow and evidence of engineering design, implementation and iterative validation. Future work should focus on threshold calibration, repeatability testing, physical command-output enablement, and benchmark timing across multiple Start-to-Exit and Exit-to-Start runs.
