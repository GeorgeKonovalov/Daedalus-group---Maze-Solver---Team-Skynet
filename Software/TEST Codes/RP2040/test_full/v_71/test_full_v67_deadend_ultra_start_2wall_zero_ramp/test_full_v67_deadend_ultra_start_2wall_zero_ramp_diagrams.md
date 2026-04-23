# `test_full_v67_deadend_ultra_start_2wall_zero_ramp` diagrams

## 1. System architecture

```mermaid
flowchart LR
  UI["Encoder + Button + OLED UI"] --> CFG["workingSettings / activeSettings / gCfg"]
  CFG --> EXEC["Runtime State Machine"]
  CFG --> TEST["Preview + Event-Test Runners"]

  IR["IR Sensors"] --> PER["Perception Frame"]
  US["UART Ultrasonic Stream"] --> PER
  IMU["LSM6DSOX IMU"] --> PER

  PER --> EVT["Event Classification"]
  PER --> PIDSEL["PID Source Selection"]

  EVT --> EXEC
  PIDSEL --> CTRL["Wall PID + IMU Yaw PID + IMU Lateral PID + Front PID"]
  EXEC --> CTRL
  TEST --> CTRL

  CTRL --> MIX["Omni X-Drive Mixer"]
  CFG --> MIX
  MIX --> SERVO["Per-Wheel Servo Output + Neutral Deadband"]
  SERVO --> ROBOT["Robot Motion"]
  ROBOT --> IR
  ROBOT --> US
  ROBOT --> IMU
```

## 2. Runtime navigation flow

```mermaid
stateDiagram-v2
  [*] --> Idle
  Idle --> StartSeek: beginExecution()
  StartSeek --> FrontNormalize: start confirmed
  StartSeek --> StartSeek: keep seeking start

  FrontNormalize --> AcquireCorridor: correction complete
  AcquireCorridor --> Corridor: corridor reacquired / release complete

  Corridor --> Corridor: normal drive
  Corridor --> ApproachWall: left/right/T-junction confirmed
  Corridor --> FrontNormalize: front < FrontStop
  Corridor --> Finished: finish confirmed

  ApproachWall --> FrontNormalize: front < FrontStop
  FrontNormalize --> AcquireCorridor: heading swap / reverse / release
  AcquireCorridor --> FrontNormalize: blocked again before corridor acquired

  Finished --> [*]
```

## 3. Control stack

```mermaid
flowchart TD
  A["Perception Frame"] --> B{"Side walls present?"}
  B -->|Both| C["2-wall PID"]
  B -->|Left only| D["1-wall PID (left)"]
  B -->|Right only| E["1-wall PID (right)"]
  B -->|None| F["IMU lateral compensation"]

  A --> G["IMU yaw-hold PID"]
  A --> H{"Stop correction stage?"}
  H -->|Yes| I["Front-distance PID"]
  H -->|No| J["No front hold"]

  C --> K["Lateral correction"]
  D --> K
  E --> K
  F --> K

  G --> L["Rotation correction"]
  I --> M["Forward compensation"]
  J --> M

  K --> N["driveRobotFrame / driveRobotFrameAdvanced"]
  L --> N
  M --> N
  N --> O["driveBodyFrameLimited"]
  O --> P["FL / FR / RR / RL wheel commands"]
```

## 4. Menu map

```mermaid
flowchart TD
  ROOT["Root Menu"] --> STARTCFG["Start Settings"]
  ROOT --> SETTINGS["Settings"]
  ROOT --> EVENTMENU["Event Test Menu"]
  ROOT --> IMUTEST["IMU Test"]
  ROOT --> IMUYAW["IMU Yaw Test"]
  ROOT --> MAZEPREVIEW["Maze Preview Run"]
  ROOT --> IRTEST["IR Sensor Test"]
  ROOT --> USTEST["Ultrasonic Test"]
  ROOT --> STARTCONF["Start Confirmation"]

  SETTINGS --> PID["PID Menu"]
  SETTINGS --> MOTORTUNE["Motor Tune"]

  PID --> IMUPID["IMU PID Menu"]
  PID --> PIDTEST["PID Test"]

  EVENTMENU --> EVENTCFG["Event Test Config"]
  EVENTCFG --> EVENTRUN["Event Test Run"]

  STARTCFG --> ROOT
  IMUPID --> PID
  MOTORTUNE --> SETTINGS
  PIDTEST --> PID
  EVENTRUN --> EVENTCFG
```

## 5. Settings data path

```mermaid
flowchart LR
  EDITOR["Digit Editor / Toggle / Cycle"] --> WORK["workingSettings"]
  WORK --> COMMIT["commitRuntimeConfig()"]
  COMMIT --> ACTIVE["activeSettings"]
  COMMIT --> GCFG["gCfg (float runtime config)"]
  GCFG --> NAV["Navigation + PID + Motion"]
  ACTIVE --> UI["Draw screens / menu values"]
```

## 6. Event recognition overview

```mermaid
flowchart TD
  FRAME["buildPerceptionFrame()"] --> FIN{"Finish armed\nand finish geometry?"}
  FIN -->|Yes| FINISH["Finish"]
  FIN -->|No| DE{"Front < deadEnd\nand both front IR?"}
  DE -->|Yes| DEAD["DeadEnd"]
  DE -->|No| ST{"Front IR pair on\nand both side walls?"}
  ST -->|Yes| START["Start"]
  ST -->|No| TURN{"Turn geometry\nfrom ultrasonics?"}
  TURN -->|Left| LT["LeftTurn"]
  TURN -->|Right| RT["RightTurn"]
  TURN -->|Both open + front close| TJ["TJunction"]
  TURN -->|Side branch + front open| TJS["TJunctionStraight"]
  TURN -->|None| COR["Corridor"]
```
