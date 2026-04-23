# test_full_v67_deadend_ultra_start_2wall_zero_ramp Logic Block Schemes

Audience-facing PNG block schemes for the runtime logic, control loops, sensors, menu system, and planner.

## Generalised Firmware Logic

```mermaid
flowchart TD
  A["Power on / setup"] --> B["Configure pins, OLED, UART, IMU, servos"]
  B --> C["Load active settings and commit runtime config"]
  C --> D["Main loop"]
  D --> E["Read encoder and button events"]
  D --> F["refreshSensors"]
  F --> G["Build heading-relative perception frame"]
  G --> H["Classify event candidate"]
  G --> I["Select PID source from side walls"]
  H --> J["Navigation / event-test state machine"]
  I --> K["Control outputs"]
  J --> K
  K --> L["Drive mixer"]
  L --> M["Per-wheel servo calibration and write"]
  J --> N["OLED drawUI"]
  N --> D
  M --> D
```

## PID Logic Selection

```mermaid
flowchart TD
  A["Perception frame with averaged left and right ultrasonic distances"] --> B{"Left <= WallThreshold?"}
  B -->|Yes| C{"Right <= WallThreshold?"}
  B -->|No| D{"Right <= WallThreshold?"}
  C -->|Yes| E["PID source: TwoWall"]
  C -->|No| F["PID source: OneWallLeft"]
  D -->|Yes| G["PID source: OneWallRight"]
  D -->|No| H["PID source: None / NoWalls"]
  E --> I["stabilizedPidSource"]
  F --> I
  G --> I
  H --> I
  I --> J{"Wall PID enabled?"}
  J -->|No| K["Wall correction = 0"]
  J -->|Yes| L{"Active source is None?"}
  L -->|No| M["computeWallCorrection"]
  M --> N{"TwoWall or OneWall?"}
  N -->|TwoWall| O["2-wall scaled error and 2-wall PID"]
  N -->|OneWall| P["1-wall target error and 1-wall PID"]
  L -->|Yes| Q["Wall correction = 0"]
  Q --> R{"IMU lateral enabled and no walls?"}
  R -->|Yes| S["IMU lateral compensation becomes lateral command"]
  R -->|No| T["No lateral correction"]
  K --> U["Final lateral command"]
  O --> U
  P --> U
  S --> U
  T --> U
```

## Event Logic Selection

```mermaid
flowchart TD
  A["Build candidate event from perception frame"] --> B{"Ultrasonic fresh and valid?"}
  B -->|No| C["SensorWait"]
  B -->|Yes| D["Compute wall flags: leftWall, rightWall, frontClose"]
  D --> E{"Finish conditions true?"}
  E -->|Yes| F["Finish"]
  E -->|No| G{"Legacy dead-end true? Front IR pair triggered and front < deadEndDistance"}
  G -->|Yes| H["DeadEnd"]
  G -->|No| I{"Start corridor true? Front IR pair triggered and both side walls"}
  I -->|Yes| J["StartCorridor"]
  I -->|No| K{"Left side open, right wall, front close?"}
  K -->|Yes| L["LeftTurn"]
  K -->|No| M{"Right side open, left wall, front close?"}
  M -->|Yes| N["RightTurn"]
  M -->|No| O{"Both sides open and front close?"}
  O -->|Yes| P["TJunction"]
  O -->|No| Q{"T-straight armed, side branch open, front open?"}
  Q -->|Yes| R["TJunctionStraight"]
  Q -->|No| S["Corridor"]
```

## IR Sensor Processing

```mermaid
flowchart TD
  A["Four IR digital inputs"] --> B["readIRSensors"]
  B --> C["Build raw IR mask"]
  C --> D["Stabilise mask over time"]
  D --> E["relativeIrForHeading"]
  E --> F["Front pair: frontLeft and frontRight"]
  E --> G["Rear pair: rearLeft and rearRight"]
  F --> H["Start check: both front IR triggered"]
  F --> I["Legacy DeadEnd check: both front IR triggered"]
  F --> J["Finish check: front pair open"]
  G --> K["Finish check: rear pair open"]
  H --> L["StartCorridor can be classified if side walls also exist"]
  I --> M["DeadEnd can be classified if front ultrasonic is below dead-end threshold"]
  J --> N["Finish needs all IR open plus ultrasonic openness"]
  K --> N
```

## Ultrasonic Sensor Processing

```mermaid
flowchart TD
  A["ATmega ultrasonic sender at 115200 baud"] --> B["UART line: D1,D2,D3,D4"]
  B --> C["parseDistancesLine"]
  C --> D{"Line valid and numeric?"}
  D -->|No| E["Reject sample and keep last valid data"]
  D -->|Yes| F["Store raw N/E/S/W distances"]
  F --> G["Averaging buffer with configurable US avg k"]
  G --> H["Averaged N/E/S/W distances"]
  H --> I["Map to robot heading frame"]
  I --> J["front, left, right, back"]
  J --> K{"Sample fresh?"}
  K -->|No| L["SensorWait / safety stop path"]
  K -->|Yes| M["Used by events, PID source, wall PID, front correction"]
```

## IMU Processing

```mermaid
flowchart TD
  A["IMU sample"] --> B{"IMU hardware ready and enabled?"}
  B -->|No| C["Yaw correction = 0 and lateral IMU correction = 0"]
  B -->|Yes| D["Gyro yaw integration and bias correction"]
  D --> E["IMU yaw reference"]
  E --> F["Yaw error"]
  F --> G["IMU yaw PID"]
  G --> H["Rotation command only"]
  A --> I["Planar acceleration ax, ay"]
  I --> J["45 degree body-frame transform"]
  J --> K["Signed lateral acceleration"]
  K --> L["Low-pass filter"]
  L --> M["IMU lateral PD/PID"]
  M --> N{"NoWalls active?"}
  N -->|Yes| O["Add to lateral command"]
  N -->|No| P["Ignore lateral IMU term"]
```

## Error Processing And Failsafes

```mermaid
flowchart TD
  A["Runtime data"] --> B{"Ultrasonic stale or invalid?"}
  B -->|Yes| C["SensorWait, reset drive PID, stopAll"]
  B -->|No| D["Continue event and PID logic"]
  D --> E{"IMU unavailable or disabled?"}
  E -->|Yes| F["IMU yaw and lateral corrections forced to zero"]
  E -->|No| G["IMU corrections allowed"]
  D --> H["PID error handling"]
  H --> I["Deadbands remove noise-level errors"]
  H --> J["Integral terms are clamped"]
  H --> K["Derivative terms are filtered"]
  H --> L["Outputs clamped to command envelope"]
  D --> M["Correction-stage event checks"]
  M --> N["Legacy dead-end is not overridden"]
  M --> O["New ultrasonic dead-end check only after non-dead-end stop"]
  L --> P["Drive mixer limits impossible wheel commands"]
  P --> Q["Per-wheel servo deadband forces neutral near zero"]
```

## Drive And Mixer Logic

```mermaid
flowchart TD
  A["lateralCmd vx"] --> D["Robot-frame mixer"]
  B["forwardCmd vy"] --> D
  C["rotationCmd w"] --> D
  D --> E["Translation base: basePlus = vy + vx; baseMinus = vy - vx"]
  E --> F["Normalize base translation if needed"]
  F --> G["Scale base by overallSpeedActual"]
  D --> H["Compensation terms from wall, front, and IMU lateral"]
  H --> I["Clamp compensation toward zero so it can reduce or stop translation but not reverse it"]
  I --> J["Apply left and right drive scales"]
  C --> K["Add rotational spin pattern"]
  J --> L["Wheel commands FL, FR, RR, RL"]
  K --> L
  L --> M["Clamp/normalize full wheel envelope"]
  M --> N["writeWheelCommand"]
  N --> O{"abs(command) <= wheel deadband?"}
  O -->|Yes| P["Write calibrated wheel stop center"]
  O -->|No| Q["Apply inversion, range, and servo angle output"]
```

## Runtime Navigation State Machine

```mermaid
flowchart TD
  A["Idle / menu"] --> B["Start Confirmation"]
  B --> C{"Start accepted?"}
  C -->|No| A
  C -->|Yes| D["StartCorridor"]
  D --> E["Start correction with PID active"]
  E --> F{"Correction complete?"}
  F -->|No| E
  F -->|Yes| G["Corridor driving with speed ramp"]
  G --> H{"Ultrasonic valid?"}
  H -->|No| I["SensorWait / stopAll"]
  I --> H
  H -->|Yes| J{"Event candidate"}
  J -->|Corridor or T-straight| G
  J -->|Left / Right / TJ| K["Approach front wall"]
  K --> L{"front < FrontStop?"}
  L -->|No| K
  L -->|Yes| M["Post-stop correction and event confirmation"]
  M --> N{"Correction complete and event confirmed?"}
  N -->|No| M
  N -->|Yes| O["Commit direction from confirmed event"]
  O --> G
  J -->|Legacy DeadEnd| P["DeadEnd correction only"]
  P --> Q{"Correction complete?"}
  Q -->|No| P
  Q -->|Yes| R["Reverse heading / opposite direction"]
  R --> G
  J -->|Finish| S["Save attempt time and route flags"]
  S --> B
```

## Menu Architecture

```mermaid
flowchart TD
  A["Root Menu"] --> B["Start Confirmation"]
  A --> C["Start Settings"]
  A --> D["Settings"]
  A --> E["PID"]
  A --> F["IMU PID"]
  A --> G["PID Test"]
  A --> H["Motor Tune"]
  A --> I["Event Test"]
  A --> J["Sensor Monitor"]
  A --> K["IR Test"]
  A --> L["Ultrasonic Test"]
  D --> M["Motion settings: speed, front stop, wall thresholds, confirmation"]
  E --> N["2-wall PID, 1-wall PID, deadbands, scales, filter"]
  F --> O["Yaw PID, lateral IMU PID, yaw error test"]
  H --> P["Drive scales and per-wheel servo stop/deadband"]
  I --> Q["Event selection"]
  Q --> R["Event config: shared PID, IMU, speed, front correction settings"]
  R --> S["Event run screen"]
  J --> T["Live heading-relative ultrasonic and PID source"]
```

## Digital Editor Logic

```mermaid
flowchart TD
  A["Menu item selected"] --> B["openDigitEditor"]
  B --> C["Copy target value to tempValue"]
  C --> D["Display padded number and selected digit underline"]
  D --> E{"Encoder moved?"}
  E -->|No| H{"Button event?"}
  E -->|Yes| F{"Digit unlocked?"}
  F -->|No| G["Move selected digit underline"]
  F -->|Yes| I["Increment or decrement selected digit"]
  I --> J["Clamp tempValue to min/max"]
  G --> H
  J --> H
  H -->|Short press| K["Toggle digit edit lock"]
  H -->|Long press| L["Save tempValue to target"]
  L --> M["commitRuntimeConfig"]
  M --> N["Return to previous menu"]
  K --> D
  H -->|None| D
```

## Planner Attempt Logic

```mermaid
flowchart TD
  A["Start confirmation requested"] --> B{"Route mode is Sequence?"}
  B -->|No| C["Use selected mode directly"]
  B -->|Yes| D{"Sequence slot"}
  D -->|0| E["Attempt Right first"]
  D -->|1| F["Attempt Left second"]
  D -->|2| G["Selection attempt"]
  G --> H{"Right and Left records exist?"}
  H -->|No| I["Planner error: Need R+L first"]
  H -->|Yes| J{"Both records had dead-end?"}
  J -->|Yes| K["Planner error: both dead-end"]
  J -->|No| L{"Only right had dead-end?"}
  L -->|Yes| M["Choose Left"]
  L -->|No| N{"Only left had dead-end?"}
  N -->|Yes| O["Choose Right"]
  N -->|No| P{"Right time <= Left time?"}
  P -->|Yes| Q["Choose Right"]
  P -->|No| R["Choose Left"]
  C --> S["beginExecution"]
  E --> S
  F --> S
  M --> S
  O --> S
  Q --> S
  R --> S
```

## Correction Stage Event Priority

```mermaid
flowchart TD
  A["Robot stopped because front < FrontStop or turn-like event confirmed"] --> B{"Was original stop legacy DeadEnd?"}
  B -->|Yes| C["Correction only"]
  C --> D["After correction, go opposite direction"]
  B -->|No| E["Run correction and confirmation together"]
  E --> F["Confirm LeftTurn / RightTurn / TJunction using existing logic"]
  E --> G["Check ultrasonic dead-end only after correction"]
  G --> H{"front, left, and right < WallThreshold?"}
  H -->|Yes| I["Confirmed DeadEnd"]
  H -->|No| J["Use confirmed Left/Right/TJ event"]
  F --> J
  I --> K["Execute event immediately after correction"]
  J --> K
```
