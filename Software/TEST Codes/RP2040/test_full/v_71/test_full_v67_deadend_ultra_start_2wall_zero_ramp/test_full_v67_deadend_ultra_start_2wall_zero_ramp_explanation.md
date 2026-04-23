# `test_full_v67_deadend_ultra_start_2wall_zero_ramp` code walkthrough

## 1. What this firmware is

This is a single-file firmware for an omni-wheel maze robot on Arduino Nano RP2040. The file combines:

- hardware setup
- persistent runtime settings
- OLED menu/UI
- sensor acquisition
- event recognition
- wall PID
- IMU yaw PID
- IMU no-wall lateral compensation
- servo drive mixing
- route planning / sequential attempts
- event-test and preview modes

The code is intentionally monolithic: instead of splitting modules into separate files, it keeps all behavior in one `.ino`, grouped by sections and helper functions.

## 2. Hardware layer

The top of the file defines the robot hardware:

- rotary encoder pins and encoder button
- 4 continuous-rotation servo outputs
- 4 IR inputs
- OLED over I2C
- LSM6DSOX IMU

The servo section defines:

- stop centers
- drive ranges
- inversion per wheel

Those compile-time values are later overridden by menu-tunable runtime values for servo neutral calibration and deadband.

## 3. Settings model

The firmware uses two parallel configuration structures:

- `RuntimeSettings`
  - fixed-point integers used for menu editing and storage in memory while the UI is running
- `RuntimeConfig`
  - live floating-point values used by the control logic

The flow is:

1. menu edits `workingSettings`
2. `commitRuntimeConfig()` copies `workingSettings` into `activeSettings`
3. the same function converts fixed-point values into `gCfg`
4. all runtime logic uses `gCfg`

That makes the UI predictable:

- the editor always works with integers
- the drive/control code always works with normalized floats

## 4. High-level behavior types

Important enums organize the firmware:

- `Heading`
  - North / East / South / West
- `RouteMode`
  - Sequence / Right / Left / Selection
- `EventType`
  - Start / Corridor / LeftTurn / RightTurn / TJunction / TJunctionStraight / DeadEnd / Finish / SensorWait / SafetyStop
- `NavState`
  - Idle / StartSeek / Corridor / ApproachWall / FrontNormalize / AcquireCorridor / SafetyStop / Finished
- `Screen`
  - all OLED menus and runtime/test screens

These enums are the backbone of the UI, event logic, and runtime state machine.

## 5. Sensor model

The firmware uses:

- 4 IR sensors
- 4 ultrasonic distances from UART
- IMU acceleration and gyro

### IR

IR is interpreted in heading-relative form:

- front-left
- front-right
- rear-right
- rear-left

That means the same physical sensors are remapped according to the robot’s logical heading.

### Ultrasonic

Ultrasonic distances are stored as:

- north
- east
- south
- west

Then converted into heading-relative:

- front
- left
- right
- back

This version uses averaged ultrasonic values in the perception path instead of mixing raw values into event logic.

### IMU

The IMU block tracks:

- raw accel / gyro
- gyro bias
- integrated yaw
- reference capture
- yaw error
- deadband/tolerance values

It supports both:

- rotational yaw hold
- lateral inertial compensation when no walls exist

## 6. Perception frame

`buildPerceptionFrame(...)` is the central sensor fusion helper.

It collects into one structure:

- current IR state
- current averaged ultrasonic geometry
- candidate event
- corridor signature
- PID source

Most of the runtime logic does not read raw globals directly. It asks for a `PerceptionFrame`, then makes decisions from that.

That is one of the most important structural choices in the code.

## 7. Event recognition

Event recognition is split into stages.

### Normal candidate classification

`classifyCandidateEvent(...)` decides the current environment shape using:

- finish conditions
- dead-end conditions
- start corridor pattern
- left/right/T-junction geometry
- T-junction-straight arming

In this version:

- start uses front IR + side walls
- dead-end still exists as a legacy front-IR + front-distance rule while driving
- left/right/T-junction come mainly from ultrasonic geometry

### Correction-stage event handling

After the robot has already stopped, correction-phase logic can:

- re-check turn events
- re-check finish
- use the newer ultrasonic-only dead-end override in the stopped/corrected state

That separation is important because it prevents noisy or premature dead-end takeover while the robot is already correcting for another event.

## 8. Planner / attempt logic

The planner stores attempt history:

- right attempt
- left attempt
- selection attempt

It tracks:

- duration
- whether a dead-end was encountered
- sequence slot
- chosen route mode

`planNextAttempt(...)` decides which mode to run next in sequence behavior. The code can compare previous attempts and reuse the better branch depending on:

- time
- dead-end history

So the planner is not only a menu mode selector. It is also the runtime memory for repeated maze attempts.

## 9. Wall PID system

The wall controller supports:

- 2-wall mode
- 1-wall symmetric mode
- 1-wall asymmetric mode
- No-wall mode

### 2-wall mode

The 2-wall error is not just a raw difference. It is scaled by a configurable “full error” distance:

- `pidTwoWallFullErrorCm`

That gives a bounded centering signal.

### 1-wall mode

One-wall control uses:

- target wall distance
- separate optional right-wall gains if symmetry is disabled

### PID shaping

The PID block includes:

- error shaping with `curveB`
- deadbands
- filtered derivative (`pidDFilterAlpha`)
- left/right output scaling

So the wall controller is more than a plain P/I/D. It is a shaped and filtered controller with menu-tunable asymmetry.

## 10. IMU control system

There are two IMU-related controllers.

### IMU yaw PID

This controls rotation only:

- capture reference yaw
- compare live yaw against reference
- produce rotational correction

Menu parameters:

- IMU P/I/D
- sign
- threshold
- pure IMU yaw test tolerance

### IMU lateral compensation

This is active when there are no side walls:

- rotate IMU accel into body axes
- filter the lateral component
- run a PD-like compensation
- inject that only into the lateral command channel

So wall PID and IMU are not doing the same job:

- wall PID corrects geometry relative to walls
- IMU yaw PID corrects heading
- IMU lateral controller damps sideways drift in no-wall corridors

## 11. Front-distance hold / stop correction

When the robot reaches a stop-worthy event, it does not turn immediately.

Instead it enters `FrontNormalize`, where it can hold:

- front distance
- yaw
- wall alignment

This stage is used before heading swaps for turn events.

The front-hold logic has its own PID:

- `frontPidP`
- `frontPidI`
- `frontPidD`
- `frontHoldReferenceCm`

Dead-end and Start correction are special:

- they do not necessarily use the same front-hold behavior as turn events

## 12. Motion mixing and drive output

The robot uses a 45 degree omni X-drive.

The drive stack is:

1. maze-relative command
2. heading-relative transform
3. body-frame translation terms
4. add compensation terms
5. apply rotational term
6. normalize wheel envelope
7. write wheel outputs

Important pieces:

- `driveRobotFrame(...)`
- `driveRobotFrameAdvanced(...)`
- `driveBodyFrameLimited(...)`
- `writeWheelCommand(...)`

This version also uses:

- `overallSpeedScale` as the configured ceiling
- `gOverallSpeedActual` as the live ramped speed used by the mixer

That separation lets the firmware ramp only the base travel while preserving correction authority.

## 13. Servo neutral calibration

Each wheel now has menu-tunable:

- stop center
- deadband

That is applied at the low-level wheel output stage:

- tiny commands inside the wheel deadband are forced to the calibrated stop value

This is specifically meant to remove wheel creep at zero command.

## 14. Runtime navigation state machine

The main runtime state machine lives in `updateExecutive()`.

Typical flow:

1. `StartSeek`
2. `Corridor`
3. `ApproachWall`
4. `FrontNormalize`
5. `AcquireCorridor`
6. back to `Corridor`
7. eventually `Finished`

### Start

The robot first seeks a valid start corridor signature.

### Corridor

This is the free-driving state:

- choose PID source continuously
- run event recognition
- drive forward at travel speed

### ApproachWall

Used when the robot has recognized a turn-type event and is approaching the stop gate.

### FrontNormalize

This is the correction stage:

- wall/yaw/front alignment depending on event
- event re-checks
- heading swap only when conditions are satisfied

### AcquireCorridor

This is the release / re-entry phase after correction or a heading change.

It prevents the robot from immediately re-triggering the same stop condition before it actually departs into the new corridor.

## 15. Preview and event-test runners

There are two additional execution frameworks besides the real runtime:

- preview run
- event-test run

They reuse the same core logic, but with different state containers.

That lets the code test:

- corridor logic
- turn logic
- start behavior
- dead-end behavior
- T-junction behavior
- IMU behavior

without needing full autonomous runtime every time.

## 16. Menu/UI system

The menu system is encoder-driven.

Main visible screens:

- Root
- Start Settings
- Settings
- PID
- IMU PID Menu
- Motor Tune
- Event Test Menu
- Event Test Config
- IMU screens
- IR / Ultrasonic test screens
- Start confirmation
- Run screen

Each screen has:

- `handle...Input(...)`
- `draw...Screen()`

That keeps input and drawing separated even though everything is still in one file.

## 17. Digit editor

`openDigitEditor(...)` and `handleDigitEditorInput(...)` provide the reusable numeric editor.

This editor:

- knows decimal formatting by precision
- supports encoder stepping by highlighted digit
- writes to `workingSettings`
- returns to the parent screen

This is why nearly every numeric tuning field is easy to reuse across multiple menus.

## 18. Setup and loop

### `setup()`

Initializes:

- pins
- encoder ISR
- display
- servos
- UART ultrasonic stream
- IMU
- menu state
- runtime configuration

### `loop()`

Regularly performs:

- sensor refresh
- button/encoder processing
- runtime update
- test/preview update
- OLED redraw

The whole firmware is effectively a periodic cooperative control loop with interrupt-assisted input edges.

## 19. What is special about this exact variant

Compared with earlier branches, this variant combines several ideas in one file:

- newer dead-end handling split between driving and correction phases
- start correction + release logic
- 2-wall PID scaled by a configurable full-error distance
- overall-speed ramping architecture
- servo-neutral calibration
- IMU yaw and no-wall lateral compensation
- unified event-test configuration screen

So this version is not “just maze logic”. It is also a tuning platform and diagnostics environment.

## 20. Practical reading order if you want to study the file

If you want to understand the code efficiently, this is the best order:

1. `RuntimeSettings`, `RuntimeConfig`, enums
2. `commitRuntimeConfig()`
3. `buildPerceptionFrame(...)`
4. `classifyCandidateEvent(...)`
5. wall PID helpers + IMU helpers
6. `driveBodyFrameLimited(...)`
7. `followSmart(...)`
8. `beginExecution()` and `updateExecutive()`
9. preview/event-test flows
10. menu handlers and draw screens

That order mirrors how the robot actually thinks:

- configure
- perceive
- classify
- control
- move
- navigate
- display
