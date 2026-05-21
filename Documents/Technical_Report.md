# Technical Report: Fully Embedded 2D-LiDAR Based Autonomous Mobile Robot

**Module:** EBU6475 Microprocessor Systems Design  
**Coursework:** End-of-Module Project  
**System:** 2D-LiDAR based Autonomous Mobile Robot for 5 x 5 maze navigation  
**Submission:** Technical Documentation  

## Abstract

This report documents the design, implementation and validation of a 2D-LiDAR based autonomous mobile robot for the EBU6475 End-of-Module project. The system combines an STM32 NUCLEO-F446RE real-time embedded controller with a Python Host application for high-level mapping, visualisation and navigation. The STM32 firmware implements LiDAR acquisition, encoder/IMU processing, Bluetooth telemetry, motor PID control, OLED UI, button handling and emergency-stop protection. The Host receives fused LiDAR/odometry packets, constructs an occupancy grid, estimates maze topology, plans cell-level movement and sends motion commands back to the robot.

The report follows the CDIO lifecycle: Conceive defines the engineering problem and constraints; Design justifies architecture and trade-offs; Implement explains firmware/Host integration; Operate describes validation, benchmark evidence and limitations. The main outcome is a working hybrid autonomy system demonstrating Sense-Think-Act behaviour. The main limitation is that high-level navigation remains Host-assisted rather than fully embedded on the STM32.

## Team Members and Responsibilities

| Name | Student ID | Main Responsibility |
|---|---:|---|
| Yifan Chen | 231221526 | Team leader; led STM32 firmware architecture, FreeRTOS task integration, UART/DMA communication, motor control and final system integration. |
| Cheng Wei | 231222822 | Automatic navigation; developed the Host-side maze navigation logic, maze topology model, planning state machine and motion decision layer. |
| Mandan Dong | 231220714 | LiDAR visualisation and mapping support; contributed to scan processing, occupancy-grid display and saved-map workflow. |
| Maiyuan Cao | 231220253 | Sensor integration and calibration; supported LiDAR, IMU and encoder data checking, odometry calibration and telemetry validation. |
| Xiaoqin Liu | 231221571 | User interface, safety and documentation; contributed to OLED state display, button behaviour, E-stop workflow and report/video preparation. |
| Xingrui Li | 231220482 | Motor and encoder testing; supported PID tuning, benchmark preparation, code packaging and README organisation. |

## CDIO Evidence Map

| CDIO Phase | Evidence Presented in This Report | Main Assessment Criterion Addressed |
|---|---|---|
| Conceive | Requirement interpretation, resource constraints, benchmark objective and architecture boundary between STM32 and Host. | Critical assessment of the project challenge. |
| Design | Sense-Think-Act architecture, FreeRTOS task structure, Host maze topology, scan matching, PID control and safety design. | Higher-order technical reasoning and design trade-off justification. |
| Implement | STM32 firmware modules, Host navigation modules, telemetry packet design and integration workflow. | System realisation, subsystem coordination and technical execution. |
| Operate | Staged validation, video evidence, benchmark placeholders, engineering issues and refinement. | Verification, validation and performance evaluation. |
| Reflection | Hybrid architecture limitation, GenAI verification, future migration to embedded autonomy. | Responsible reflection and self-assessment. |

## 1. System Overview

The project objective was to design and implement a 2D-LiDAR based autonomous mobile robot capable of operating in a 5 x 5 maze, navigating from the Start cell to the Exit cell, and then returning to the Start cell. The robot platform used the STM32 NUCLEO-F446RE, RPLIDAR C1, wheel encoders, IMU, geared DC motors, AT8236 motor driver, HC-04 Bluetooth module, SSD1306 OLED display, push buttons and potentiometers.

Our final system uses a hybrid embedded-supervisory architecture. The STM32 firmware implements the real-time embedded layer: LiDAR acquisition, encoder odometry, IMU heading processing, Bluetooth telemetry, command parsing, motor PID control, OLED display, button handling and emergency-stop safety. A Python Host application is used as the high-level navigation, mapping, visualisation and debugging layer. The Host receives fused LiDAR/odometry packets from the STM32, builds an occupancy grid, maintains a discrete maze topology, plans one-cell motion decisions, and sends motion commands back to the robot.

This architecture allowed the team to demonstrate Sense-Think-Act behaviour while maintaining reliable real-time motor and sensor operation on the STM32. It is also a limitation relative to the brief's fully embedded target: STM32 handles time-critical sensing, actuation and safety, while the Host performs high-level mapping and navigation.

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

The robot was required to work inside a 5 x 5 maze with 0.70 m cells. This cell size drove the navigation abstraction: instead of planning continuous arbitrary trajectories, the system plans in cell-to-cell moves. Each high-level decision is converted into either a rotation command or a forward one-cell command. This reduced the complexity of path planning and made it easier to validate movement using encoder/IMU feedback.

Key engineering constraints were:

- Limited STM32 memory and CPU time compared with a desktop computer.
- High LiDAR serial data rate and risk of data loss if handled with blocking reads.
- Wheel slip and mechanical mismatch between the two motors.
- Odometry drift during long movement sequences.
- Bluetooth latency and packet loss risk during dense telemetry.
- Need for immediate motor shutdown during emergency-stop conditions.

The main design decisions made during the Conceive phase are summarised below:

| Decision | Reasoning | Risk Accepted |
|---|---|---|
| Use a cell-level maze abstraction rather than continuous trajectory planning. | The benchmark maze is a fixed 5 x 5 grid, so cell-to-cell planning directly matches the task. | Fine continuous path optimisation is limited. |
| Keep motor control and safety on the STM32. | These functions are timing-critical and must continue even if Host communication is delayed. | Requires robust command parsing and timeout handling. |
| Use the Host for high-level map processing and navigation. | This improves debugging, visualisation and development speed under project time constraints. | High-level autonomy is not fully embedded. |
| Use binary Bluetooth telemetry. | LiDAR produces dense data, so text telemetry would waste bandwidth and increase latency. | Requires packet framing and checksum verification. |

## 3. Design: System Architecture and Trade-Offs

### 3.1 Sense-Think-Act Architecture

The system is organised as a layered Sense-Think-Act pipeline:

| Layer | Implementation |
|---|---|
| Sense | STM32 reads RPLIDAR C1 through UART5 DMA, reads encoder counters using TIM1/TIM3 hardware encoder mode, and processes IMU heading data through USART DMA. |
| Think | Host receives fused scan/odometry data, updates the occupancy grid, performs scan matching, observes wall states and plans the next maze-cell movement. |
| Act | STM32 parses Bluetooth commands, runs motor PID control at a fixed period, drives PWM outputs through TIM2 and enforces command timeout and emergency-stop behaviour. |

The STM32 firmware was designed around FreeRTOS tasks:

| Task | Period | Priority | Purpose |
|---|---:|---|---|
| `CommandTask` | 1 ms | Above Normal | Reads Bluetooth DMA buffer and parses incoming motion commands. |
| `MotorControlTask` | 5 ms | Above Normal | Samples encoder deltas and updates cascaded PID motor control. |
| `IMU900Task` | 2 ms | Normal | Processes IMU packets and updates yaw/odometry state. |
| `LidarTask` | 2 ms | Normal | Parses LiDAR nodes and sends fused LiDAR/odometry packets to the Host. |
| `ButtonTask` | 20 ms | Normal | Debounces buttons and handles mode changes/E-stop reset. |
| `UITask` | 200 ms | Below Normal | Updates potentiometer values and refreshes the OLED display. |

The priorities keep command handling and motor control ahead of non-critical UI refresh.

### 3.2 STM32 Firmware Design

The firmware is divided into modules:

- `uart_device.c`: UART DMA reception, circular buffer reading and Bluetooth/debug transmission.
- `lidar.c`: RPLIDAR command handling, 5-byte scan node parsing, quality/distance filtering and fused telemetry packet generation.
- `encoder.c`: TIM1/TIM3 hardware encoder sampling, tick accumulation, speed estimation and odometry update.
- `im948.c`: IMU packet parsing, yaw angle update and odometry packet support.
- `bt_cmd.c`: Bluetooth command parser, movement state machine and cascaded PID control.
- `motor.c`: PWM and motor direction GPIO control.
- `button.c`: debounced buttons and emergency-stop reset handling.
- `robot_state.c`: global robot state and latched E-stop state.
- `ui_task.c`: OLED status display and potentiometer monitoring.

LiDAR points with low quality, distance below 0.20 m or distance above 1.50 m are rejected. Each valid point is packed with odometry and yaw into a binary packet with header `0xAA55` and XOR checksum, reducing Bluetooth bandwidth compared with text telemetry.

The encoder design uses hardware timer encoder mode rather than GPIO interrupts, reducing CPU load and missed-pulse risk. The left encoder uses TIM3 and the right encoder uses TIM1; counter deltas are sampled every 5 ms in the motor control task.

### 3.3 Host Navigation Design

The mapping system uses a 500 x 500 occupancy grid with 0.05 m resolution. Each LiDAR measurement is transformed into world coordinates using the associated robot pose. A Bresenham ray update marks free space along the beam and increases occupancy at the measured obstacle point. The Host also stores the robot trajectory for monitoring and saved-map analysis.

For maze navigation, the Host uses a discrete 5 x 5 topology:

- Each maze cell is represented by row and column.
- Each edge has a wall state: `UNKNOWN`, `OPEN` or `BLOCKED`.
- Observed wall states are updated from local occupancy samples near each cell boundary.
- A path is planned using an A*-style search over the topology.
- If no confirmed path exists, the planner chooses a frontier cell and allows traversal through unknown edges with higher cost.

The navigation state machine follows this sequence:

1. Localise the start pose and set a reference heading.
2. Observe walls around the current cell.
3. Check whether the current target has been reached.
4. Plan the next cell-to-cell step.
5. Rotate to the target heading if required.
6. Advance one cell.
7. Settle, scan-match and update topology.
8. Repeat until Exit is reached, then switch to return mode.

This structure was chosen because the benchmark maze is grid-based. The cell-level abstraction maps directly to the environment and produces simple movement commands for the STM32.

### 3.4 Motion Control Design

The STM32 motion controller supports continuous commands (`W`, `S`, `A`, `D`, `x`) and precise commands (`F`, `B`, `L`, `R` followed by a value). The Host mainly uses simple movement commands, while precise movement/turn states support calibration and manual testing.

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
- Forward-clearance stop in the Host controller.
- Lateral-error stop if the robot deviates too far during a cell move.
- Motion timeout for rotation and forward movement.
- Safe stop command `x` from Host or keyboard.

## 4. Implement: Realisation and Integration

The final implementation connects three software layers:

1. STM32 firmware in `MSD`.
2. General Host visualisation tools in `Host`.
3. Automatic navigation Host application in `DnB/Host`.

The STM32 initialisation sequence configures GPIO, DMA, UARTs, timers, ADC and I2C, then starts UART, motor, encoder, robot state, potentiometer and Bluetooth command modules before FreeRTOS schedules the application tasks.

The LiDAR implementation was integrated with odometry before telemetry transmission. Each valid point is sent with `odom_x`, `odom_y` and `AngleZ`, allowing the Host to map each observation using the corresponding robot pose. Sync markers identify full scan rotations.

The Host application uses a scan buffer to reduce distortion during motion. A complete scan stores the start pose and end pose, and each point is assigned an interpolated pose according to its scan angle. This was necessary because the robot can move while a full scan is being collected. Without this correction, walls become curved or duplicated during movement.

The automatic navigation module is split into focused components:

| File | Responsibility |
|---|---|
| `maze_types.py` | Shared data types for cells, directions, wall states, navigation states and motion status. |
| `maze_perception.py` | Coordinate conversion, heading quantisation, cell observation and forward-clearance estimation. |
| `maze_matcher.py` | Lightweight scan matching by searching small pose offsets and scoring against the occupancy grid. |
| `maze_topology.py` | 5 x 5 maze graph, wall confidence, visited cells, frontier selection and path planning. |
| `maze_controller.py` | Motion execution guard using progress, lateral error, clearance and timeout checks. |
| `maze_navigator.py` | High-level autonomous navigation state machine. |
| `Host.py` | Serial communication, visualisation, mapping loop, manual controls and automatic navigation integration. |

This modular design allowed mapping, topology planning and movement control to be tested independently before full integration.

The implementation also includes explicit data contracts between the embedded and Host layers:

| Data/Command | Direction | Purpose |
|---|---|---|
| LiDAR fused packet `0xAA55` | STM32 to Host | Carries LiDAR angle, distance, quality, odometry position and heading. |
| Odometry packet `0xBB55` | STM32 to Host | Provides pose updates when no LiDAR point is being processed. |
| Scan sync marker | STM32 to Host | Marks the boundary between LiDAR scan rotations. |
| `W/S/A/D/x` | Host to STM32 | Continuous forward, backward, turn-left, turn-right and stop commands. |
| `F/B/L/R + value` | Host/manual to STM32 | Precise distance or angle commands used for testing and calibration. |

## 5. Operate: Testing, Validation and Performance Evaluation

The system was validated through staged testing. Embedded peripherals were tested individually: encoders by signed tick changes, PWM through manual commands, LiDAR through stable wall points in the Host visualiser, and Bluetooth through packet headers, checksums and scan sync markers.

Closed-loop motion was tuned by driving forward and rotating while observing heading stability and motor response. Heading and velocity-difference PID tuning reduced curved forward motion, while the turn PID reduced overshoot during 90-degree turns. Potentiometers were used to adjust base duty and turn duty during testing.

Mapping behaviour was then validated in the maze by checking whether straight walls remained straight, rotation caused map dragging, and repeated passes reinforced the same wall locations. The rotation filter, scan buffer and distance correction reduced map artefacts.

Finally, autonomous navigation was tested in the benchmark maze. The Host updated wall topology, planned the next cell, sent commands to the STM32, waited for motion completion or guard-stop conditions, then re-observed the cell before continuing. The same system was used for Start-to-Exit and Exit-to-Start operation.

The validation strategy is summarised below. The submitted videos are used as primary visual evidence instead of screenshots or separate log figures.

| Validation Target | Method | Evidence Source |
|---|---|---|
| Encoder direction and tick accumulation | Manual wheel rotation and movement commands. | Product Quality Video / live demonstration. |
| LiDAR acquisition | Host visualiser shows real-time wall points and map updates. | Product Quality Video. |
| Bluetooth packet link | Host receives fused LiDAR/odometry packets and responds to commands. | Product Quality Video. |
| Motor closed-loop behaviour | Forward and turning commands show corrected heading and stable response. | Product Quality Video. |
| UI and safety | OLED displays state/pose/tuning values; E-stop and stop command disable motion. | Product Quality Video. |
| Autonomous navigation | Host planner drives cell-to-cell movement through the maze. | Product Quality Video and Benchmark Video. |
| Benchmark timing | Continuous Start-to-Exit and Exit-to-Start recording with readable timer. | Benchmark Video. |

Benchmark results are left blank here for completion after final testing:

| Metric | Measured Time | Maximum Allowed Time | Result |
|---|---:|---:|---|
| Start to Exit | ___ s | 120 s | ___ |
| Exit to Start | ___ s | 120 s | ___ |

The benchmark score can be calculated using the brief's linear scaling formula after inserting the measured times.

For the report submission, compute each score as `100 * (120 - T_measured) / (120 - 30)` and average the Start-to-Exit and Exit-to-Start scores.

## 6. Engineering Issues, Refinement and Limitations

**LiDAR noise and near-distance distortion.** Early mapping results were sensitive to low-quality points and near-field distortion. The firmware therefore filters LiDAR points by quality and distance. The Host also applies distance scaling and optional near-distance nonlinear correction to better align measured wall positions with the physical maze.

**Odometry drift.** Wheel encoder odometry accumulates error because of wheel slip, motor mismatch and imperfect wheel diameter assumptions. The firmware uses an encoder scale factor and IMU yaw to improve short-term odometry. The Host then applies lightweight scan matching over small `dx`, `dy` and `dtheta` candidates to reduce drift before snapping the robot to the nearest maze cell.

**Rotation artefacts in mapping.** When the robot rotates while scanning, LiDAR points from one scan correspond to different robot headings. This produced wall smearing in the occupancy grid. The Host was improved with scan buffering, pose interpolation and rotation detection, so points collected during unstable rotation are either corrected or given reduced influence.

**Bluetooth throughput and latency.** Text telemetry was not suitable for dense LiDAR data. The implementation therefore uses compact binary packets with headers and checksums. UART reception on the STM32 uses DMA circular buffers to avoid blocking the real-time tasks. The Host also refreshes active motion commands during autonomous control so that command timeout does not stop the robot unintentionally.

**Task scheduling.** OLED display updates and debug output must not interfere with control timing. The final FreeRTOS design places command and motor-control tasks at higher priority, while the UI task runs at lower priority and slower frequency.

The most important limitation is that the final autonomy is hybrid rather than fully embedded. The project demonstrates embedded real-time integration and autonomous behaviour, but high-level mapping, topology planning and navigation decisions remain on the Host. The next step would be to replace the full Host occupancy grid with a smaller embedded maze-topology representation and run the cell-level planner directly on STM32 memory.

## 7. GenAI Use and Verification

One useful GenAI output was the suggestion to separate the navigation software into a discrete topology layer, a maze planner and a low-level motion controller. This helped the team avoid mixing map interpretation, path planning and motor command logic in one large script. The final Host code reflects this separation through `MazeTopology`, `MazeNavigator`, `MazePerception`, `MazeMatcher` and `MazeController`. The suggestion was verified by checking that each planned topology edge corresponded to one physical movement command and by testing the state transitions during integrated operation.

One misleading GenAI output was the suggestion to implement full ICP or particle-filter SLAM entirely on the STM32. After checking the STM32 memory limits, integration risk and available project time, this was judged unrealistic for the current implementation. The design was refined into a lightweight hybrid architecture: the STM32 performs real-time sensing, motor control, telemetry and safety, while the Host performs high-level map processing and navigation. This limitation is explicitly reported rather than hidden.

Another useful GenAI-supported discussion was PID tuning strategy. GenAI helped compare a single open-loop duty correction approach with a cascaded heading and velocity-difference controller. The team then verified the idea against the actual firmware by observing whether the robot maintained heading during forward motion and whether the encoder speed difference decreased after tuning.

The verification rule was that no GenAI suggestion was accepted unless it could be connected to code, observed robot behaviour, or a clear hardware/resource constraint.

## 8. Conclusion

The project produced an integrated 2D-LiDAR based AMR system capable of sensing the maze, mapping the environment, planning cell-level navigation, and executing movement through STM32-based real-time motor control. The STM32 firmware provides UART/DMA sensing, encoder and IMU processing, PID motor control, OLED interface, button handling and emergency-stop protection. The Host provides occupancy-grid mapping, scan matching, maze topology planning and automatic motion decisions.

The system demonstrates a complete Sense-Think-Act workflow and evidence of engineering design, implementation and iterative validation. The main remaining limitation is that high-level autonomy is not yet fully embedded on the STM32. Future work should migrate the Host-side topology planner and map representation into embedded firmware, reduce Bluetooth dependence and test repeatability across multiple benchmark runs.
