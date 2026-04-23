$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$outFile = Join-Path $root 'test_full_v67_deadend_ultra_start_2wall_zero_ramp_menu_parameters.xlsx'

function New-Row {
  param(
    [string]$Parameter,
    [string]$Default,
    [string]$Units,
    [string]$Type,
    [string]$PrimaryMenu,
    [string]$AlsoIn,
    [string]$BackingField,
    [string]$Description,
    [string]$UsedIn,
    [string]$Notes
  )

  [PSCustomObject]@{
    Parameter    = $Parameter
    Default      = $Default
    Units        = $Units
    Type         = $Type
    PrimaryMenu  = $PrimaryMenu
    AlsoIn       = $AlsoIn
    BackingField = $BackingField
    Description  = $Description
    UsedIn       = $UsedIn
    Notes        = $Notes
  }
}

$rows = @(
  New-Row 'Route mode' 'Sequence' '-' 'cycle' 'Start Settings' 'Start Confirmation / planner displays' 'workingSettings.routeMode' 'Selects the global attempt strategy: Sequence, Right, Left, or Selection.' 'planNextAttempt(), currentRouteMode(), beginExecution()' 'Sequence mode uses stored attempt history to choose later runs.'
  New-Row 'Start heading' 'North' '-' 'cycle' 'Start Settings' 'Preview/Start displays' 'workingSettings.startHeadingIndex' 'Initial logical heading used for runtime, preview, and PID test startup.' 'beginExecution(), startPreviewExecution(), PID test heading seed' 'Rotates the whole heading-relative sensor/control frame.'
  New-Row 'Overall start' '0.20' 'normalized' 'numeric' 'Start Settings' 'Event Test Config (Start case)' 'workingSettings.overallStart_x100' 'Initial overall speed used while Start is active or immediately after Start correction.' 'initializeRuntimeOverallSpeed(), updateOverallSpeedActualForState()' 'Works together with Overall ramp.'
  New-Row 'Overall ramp' '0.50' 'x/s' 'numeric' 'Start Settings' 'Event Test Config (Start case)' 'workingSettings.overallRampK_x100' 'Rate at which overallSpeedActual rises toward Overall speed.' 'updateOverallSpeedActualForState()' 'Stored in x100, so 50 means 0.50 units/s.'

  New-Row 'IMU enable' 'ON' '-' 'toggle' 'Settings' 'IMU PID Menu / Event Test Config' 'workingSettings.imuEnabled' 'Global enable for all IMU-based compensation and tests.' 'imuUsageEnabled(), currentImuRotationCompensation(), currentImuLateralCompensation()' 'If OFF, yaw and lateral IMU outputs are explicitly forced to zero.'
  New-Row 'Overall speed' '0.80' 'normalized' 'numeric' 'Settings' 'Motor Tune / Event Test Config' 'workingSettings.overallSpeedScale_x100' 'Configured ceiling for base travel speed scaling.' 'driveBodyFrameLimited(), setOverallSpeedActual()' 'Corrections stay comparatively strong while base travel is scaled.'
  New-Row 'Left-turn ramp enable' 'ON' '-' 'toggle' 'Settings' 'Event Test Config (LeftTurn/Random)' 'workingSettings.leftTurnRampEnabled' 'Enables ramped release after a left-turn stop event.' 'prepareOverallSpeedAfterStopEvent()' 'If OFF, overall speed snaps back to target after the stop.'
  New-Row 'Right-turn ramp enable' 'ON' '-' 'toggle' 'Settings' 'Event Test Config (RightTurn/Random)' 'workingSettings.rightTurnRampEnabled' 'Enables ramped release after a right-turn stop event.' 'prepareOverallSpeedAfterStopEvent()' 'Same mechanism as left-turn ramp.'
  New-Row 'T-junction ramp enable' 'ON' '-' 'toggle' 'Settings' 'Event Test Config (TJunction/Random)' 'workingSettings.tJunctionRampEnabled' 'Enables ramped release after a T-junction stop event.' 'prepareOverallSpeedAfterStopEvent()' 'Applies after heading choice and correction.'
  New-Row 'Dead-end ramp enable' 'ON' '-' 'toggle' 'Settings' 'Event Test Config (DeadEnd/Random)' 'workingSettings.deadEndRampEnabled' 'Enables ramped release after a dead-end reverse/release event.' 'prepareOverallSpeedAfterStopEvent()' 'Dead-end uses a different handling path than turns.'
  New-Row 'Front stop' '15.00' 'cm' 'numeric' 'Settings' 'Event Test Config' 'workingSettings.frontStopDistance_x100' 'Front distance that triggers the stop/correction stage.' 'updateExecutive(), updatePreviewExecution(), updateEventTestRun()' 'One of the most important navigation thresholds.'
  New-Row 'Front hold reference' '3.50' 'cm' 'numeric' 'Settings' 'Event Test Config' 'workingSettings.frontHoldReference_x100' 'Desired front distance during stopped front-normalization before heading swap.' 'computeFrontNormalizedError(), frontNormalizationComplete()' 'Used for turn-style correction, not every event type.'
  New-Row 'Front PID P' '0.150' 'normalized' 'numeric' 'Settings' 'Event Test Config' 'workingSettings.frontPidP_x1000' 'Proportional gain of the front-distance hold controller.' 'computeFrontPidCorrection()' 'Higher values tighten the front hold but can oscillate.'
  New-Row 'Front PID I' '0.000' 'normalized' 'numeric' 'Settings' 'Event Test Config' 'workingSettings.frontPidI_x1000' 'Integral gain of the front-distance hold controller.' 'computeFrontPidCorrection()' 'Zero by default.'
  New-Row 'Front PID D' '0.040' 'normalized' 'numeric' 'Settings' 'Event Test Config' 'workingSettings.frontPidD_x1000' 'Derivative gain of the front-distance hold controller.' 'computeFrontPidCorrection()' 'Used to damp approach-to-wall settling.'
  New-Row 'Turn yaw tolerance' '3.00' 'deg' 'numeric' 'Settings' 'Event Test Config' 'workingSettings.turnYawTolerance_x100' 'Allowed yaw error before heading swap / stop correction is considered complete.' 'yawNormalizationComplete(), frontNormalizationComplete()' 'Works together with IMU deadband/tolerance logic.'
  New-Row 'Wall threshold' '16.00' 'cm' 'numeric' 'Settings' 'Event Test Config' 'workingSettings.corridorWallThreshold_x100' 'Threshold used to decide whether a side wall exists.' 'classifyCandidateEvent(), choosePidSource(), dead-end override helpers' 'Affects event recognition and PID source selection.'
  New-Row 'Turn detect distance' '26.00' 'cm' 'numeric' 'Settings' 'Event Test Config' 'workingSettings.turnDetectDistance_x100' 'Maximum front distance that still counts as a turn-type close-front condition.' 'classifyCandidateEvent()' 'Separates turn geometry from straight-open corridor.'
  New-Row 'Dead-end distance' '20.00' 'cm' 'numeric' 'Settings' 'Event Test Config / IR Sensor Test' 'workingSettings.deadEndDistance_x100' 'Legacy front-distance threshold for dead-end recognition while driving.' 'classifyCandidateEvent(), legacyDeadEndStopTriggered()' 'Works together with both front IR sensors in the legacy dead-end path.'
  New-Row 'Finish distance' '30.00' 'cm' 'numeric' 'Settings' 'Event Test Config / IR Sensor Test' 'workingSettings.finishDistance_x100' 'Minimum front distance required for finish recognition.' 'classifyCandidateEvent(), tryHandleRuntimeFinish()' 'Finish also depends on IR/open-side geometry.'
  New-Row 'Check gap' '0.01' 's' 'numeric' 'Settings' 'Event Test Config / IR Sensor Test' 'workingSettings.sensingInterval_x100' 'Delay between repeated event-confirmation samples.' 'updateEventConfirmState(), updateCorridorConfirmState()' 'Smaller values confirm faster but reduce noise immunity.'
  New-Row 'Ultrasonic average k' '3' 'samples' 'numeric' 'Settings' 'Displayed in sensor screens' 'workingSettings.ultrasonicAvgCount' 'Number of UART ultrasonic readings averaged into the perception frame.' 'commitRuntimeConfig(), buildPerceptionFrame()' 'Changing it resets the averaging buffers.'
  New-Row 'Match N' '2' 'count' 'numeric' 'Settings' 'Event Test Config / IR Sensor Test' 'workingSettings.eventConfirmCount' 'How many matching detections are required to confirm an event.' 'updateEventConfirmState()' 'A key noise-vs-reactivity tuning knob.'

  New-Row 'Wall PID enable' 'ON' '-' 'toggle' 'PID Menu' 'Event Test Config' 'workingSettings.wallPidEnabled' 'Global enable for wall-following correction.' 'wallPidUsageEnabled(), computeWallCorrection()' 'If OFF, wall PID output is zero even when walls are present.'
  New-Row '1-wall symmetry' 'ON' '-' 'toggle' 'PID Menu' 'Event Test Config' 'workingSettings.pid1WallSymmetryEnabled' 'Chooses whether left and right one-wall cases share one PID or use a separate right-wall PID.' 'computeWallCorrection(), PID menu layout' 'When OFF, extra 1WR fields appear.'
  New-Row '2-wall P' '0.250' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.p_x1000' 'Proportional gain for 2-wall centering control.' 'computePidWithRuntime()' 'This version uses a scaled 2-wall error, not a raw distance difference.'
  New-Row '2-wall I' '0.000' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.i_x1000' 'Integral gain for 2-wall control.' 'computePidWithRuntime()' 'Zero by default.'
  New-Row '2-wall D' '0.020' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.d_x1000' 'Derivative gain for 2-wall control.' 'computePidWithRuntime()' 'Filtered by D alpha.'
  New-Row 'Curve b' '1.00' '-' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.curveB_x100' 'Shapes small errors before they are fed into PID.' 'shapeSignedError(), shapedMagnitude()' 'Values above 1 soften small corrections.'
  New-Row '1-wall target distance' '3.50' 'cm' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pidWallDistance_x100' 'Fallback target distance for one-wall following.' 'computeOneWallScaledError()' 'Can be superseded by remembered wall references during runtime.'
  New-Row '1-wall P' '0.350' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pid1WallP_x1000' 'Proportional gain for symmetric 1-wall mode.' 'computeWallCorrection()' 'Applies to left and right when symmetry is ON.'
  New-Row '1-wall I' '0.000' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pid1WallI_x1000' 'Integral gain for symmetric 1-wall mode.' 'computeWallCorrection()' 'Zero by default.'
  New-Row '1-wall D' '0.040' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pid1WallD_x1000' 'Derivative gain for symmetric 1-wall mode.' 'computeWallCorrection()' 'Used for damping one-wall steering.'
  New-Row '1-wall right P' '0.350' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pid1WallRightP_x1000' 'Proportional gain for right-wall-only mode when symmetry is OFF.' 'computeWallCorrection()' 'Hidden unless 1-wall symmetry is OFF.'
  New-Row '1-wall right I' '0.000' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pid1WallRightI_x1000' 'Integral gain for right-wall-only mode when symmetry is OFF.' 'computeWallCorrection()' 'Hidden unless 1-wall symmetry is OFF.'
  New-Row '1-wall right D' '0.040' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pid1WallRightD_x1000' 'Derivative gain for right-wall-only mode when symmetry is OFF.' 'computeWallCorrection()' 'Hidden unless 1-wall symmetry is OFF.'
  New-Row '1-wall deadband' '0.60' 'cm' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pidDeadband1W_x100' 'Deadband for one-wall distance error.' 'computeWallCorrection()' 'Helps suppress noise when following a single wall.'
  New-Row '2-wall full error' '8.00' 'cm' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pidTwoWallFullError_x100' 'Side-distance mismatch that maps to full 2-wall correction.' 'computeTwoWallScaledError()' 'Central to the scaled 2-wall PID design of this variant.'
  New-Row '2-wall deadband' '0.050' 'normalized' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pidDeadband2W_x1000' 'Deadband on the scaled 2-wall error signal.' 'computeWallCorrection()' 'Stored in x1000.'
  New-Row 'D filter alpha' '0.200' '-' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pidDFilterAlpha_x1000' 'Low-pass smoothing applied to PID derivative estimates.' 'computePidWithRuntime(), computeFrontPidCorrection()' 'Higher alpha follows raw derivative more strongly.'
  New-Row 'Left output scale' '1.00' '-' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pidLeftScale_x100' 'Extra scale applied to negative correction output.' 'computePidWithRuntime()' 'Used for drivetrain asymmetry compensation.'
  New-Row 'Right output scale' '1.00' '-' 'numeric' 'PID Menu' 'Event Test Config' 'workingSettings.pidRightScale_x100' 'Extra scale applied to positive correction output.' 'computePidWithRuntime()' 'Used for drivetrain asymmetry compensation.'

  New-Row 'IMU P' '3.9000' 'normalized' 'numeric' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuP_x10000' 'Yaw-hold proportional gain.' 'applyImuCompensation()' 'Much larger numerically because yaw error is normalized internally.'
  New-Row 'IMU I' '1.7000' 'normalized' 'numeric' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuI_x10000' 'Yaw-hold integral gain.' 'applyImuCompensation()' 'Provides drift rejection.'
  New-Row 'IMU D' '0.0000' 'normalized' 'numeric' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuD_x10000' 'Yaw-hold derivative gain.' 'applyImuCompensation()' 'Zero in this variant.'
  New-Row 'IMU lateral P' '0.2500' 'normalized' 'numeric' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuLatP_x10000' 'Proportional gain for no-wall lateral compensation.' 'currentImuLateralCompensation()' 'Only relevant when no side walls are present.'
  New-Row 'IMU lateral I' '0.0000' 'normalized' 'numeric' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuLatI_x10000' 'Integral gain for no-wall lateral compensation.' 'currentImuLateralCompensation()' 'Zero by default.'
  New-Row 'IMU lateral D' '0.1000' 'normalized' 'numeric' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuLatD_x10000' 'Derivative gain for no-wall lateral compensation.' 'currentImuLateralCompensation()' 'Damps sideways drift.'
  New-Row 'IMU lateral LPF alpha' '0.12' '-' 'numeric' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuLatFilterAlpha_x100' 'Filter alpha for the lateral acceleration signal.' 'currentImuLateralCompensation()' 'Lower values smooth more strongly.'
  New-Row 'IMU threshold' '1.00' 'deg' 'numeric' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuAngleThreshold_x100' 'Extra yaw threshold added above sensor-derived deadband.' 'applyImuCompensation()' 'Controls when yaw-hold starts acting.'
  New-Row 'IMU yaw test error' '3.00' 'deg' 'numeric' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuPidTestYawError_x100' 'Acceptance window for the isolated IMU PID yaw test.' 'IMU PID test screen / logic' 'Used by the dedicated menu test, not by all runtime states.'
  New-Row 'IMU sign' '-1' '-' 'toggle' 'IMU PID Menu' 'Event Test Config' 'workingSettings.imuSign_x100' 'Flips the rotational correction direction.' 'commitRuntimeConfig(), applyImuCompensation()' 'Critical if IMU correction spins the wrong way.'

  New-Row 'Travel speed' '0.35' 'normalized' 'numeric' 'Motor Tune' 'Displayed in run/test screens' 'workingSettings.baseSpeed_x100' 'Nominal forward travel speed in corridor drive.' 'followSmart(), drawMotorTuneScreen()' 'Used in Corridor state.'
  New-Row 'Approach speed' '0.28' 'normalized' 'numeric' 'Motor Tune' 'Displayed in run/test screens' 'workingSettings.approachSpeed_x100' 'Reduced forward speed used near events and stop gates.' 'followSmart(), ApproachWall/Acquire states' 'Used during cautious approach and re-acquire.'
  New-Row 'Left drive scale' '1.00' '-' 'numeric' 'Motor Tune' 'Drive mixer' 'workingSettings.leftDriveScale_x100' 'Calibration scale for FL/RR diagonal drive pair.' 'driveBodyFrameLimited()' 'Affects both translation and rotation on that diagonal.'
  New-Row 'Right drive scale' '1.00' '-' 'numeric' 'Motor Tune' 'Drive mixer' 'workingSettings.rightDriveScale_x100' 'Calibration scale for FR/RL diagonal drive pair.' 'driveBodyFrameLimited()' 'Affects both translation and rotation on that diagonal.'
  New-Row 'FL stop' '90.629' 'deg' 'numeric' 'Motor Tune' 'Servo output layer' 'workingSettings.servoStopFL_x1000' 'Neutral center for front-left continuous-rotation servo.' 'servoStopAngleForWheel(), writeWheelCommand()' 'Used to suppress wheel creep.'
  New-Row 'FR stop' '90.900' 'deg' 'numeric' 'Motor Tune' 'Servo output layer' 'workingSettings.servoStopFR_x1000' 'Neutral center for front-right servo.' 'servoStopAngleForWheel(), writeWheelCommand()' 'Fine neutral tuning.'
  New-Row 'RR stop' '89.800' 'deg' 'numeric' 'Motor Tune' 'Servo output layer' 'workingSettings.servoStopRR_x1000' 'Neutral center for rear-right servo.' 'servoStopAngleForWheel(), writeWheelCommand()' 'Fine neutral tuning.'
  New-Row 'RL stop' '90.000' 'deg' 'numeric' 'Motor Tune' 'Servo output layer' 'workingSettings.servoStopRL_x1000' 'Neutral center for rear-left servo.' 'servoStopAngleForWheel(), writeWheelCommand()' 'Fine neutral tuning.'
  New-Row 'FL deadband' '0.00' 'command' 'numeric' 'Motor Tune' 'Servo output layer' 'workingSettings.servoDeadbandFL_x100' 'Near-zero command zone forced to front-left neutral.' 'servoDeadbandForWheel(), writeWheelCommand()' 'Stored as normalized command x100.'
  New-Row 'FR deadband' '0.00' 'command' 'numeric' 'Motor Tune' 'Servo output layer' 'workingSettings.servoDeadbandFR_x100' 'Near-zero command zone forced to front-right neutral.' 'servoDeadbandForWheel(), writeWheelCommand()' 'Prevents drift around zero.'
  New-Row 'RR deadband' '0.00' 'command' 'numeric' 'Motor Tune' 'Servo output layer' 'workingSettings.servoDeadbandRR_x100' 'Near-zero command zone forced to rear-right neutral.' 'servoDeadbandForWheel(), writeWheelCommand()' 'Prevents drift around zero.'
  New-Row 'RL deadband' '0.00' 'command' 'numeric' 'Motor Tune' 'Servo output layer' 'workingSettings.servoDeadbandRL_x100' 'Near-zero command zone forced to rear-left neutral.' 'servoDeadbandForWheel(), writeWheelCommand()' 'Prevents drift around zero.'

  New-Row 'Event-test heading' 'North' '-' 'cycle' 'Event Test Config' 'Local to selected test case' 'currentEventTestConfig().headingIndex' 'Heading used by the selected event-test case.' 'currentEventTestHeading(), startEventTestRun()' 'This is not a global runtime setting.'
  New-Row 'Event-test T-junction turn choice' 'Left' '-' 'cycle' 'Event Test Config (TJunction only)' 'Local to selected test case' 'currentEventTestConfig().turnChoice' 'Manual branch choice for the T-junction event test.' 'currentEventTestConfiguredTurn(), updateEventTestRun()' 'Only shown for the TJunction test case.'
)

$excel = $null
$workbook = $null
$sheet = $null

try {
  if (Test-Path $outFile) {
    Remove-Item -Force $outFile
  }

  $excel = New-Object -ComObject Excel.Application
  $excel.Visible = $false
  $excel.DisplayAlerts = $false

  $workbook = $excel.Workbooks.Add()
  $sheet = $workbook.Worksheets.Item(1)
  $sheet.Name = 'MenuParameters'

  $headers = @('Parameter', 'Default', 'Units', 'Type', 'PrimaryMenu', 'AlsoIn', 'BackingField', 'Description', 'UsedIn', 'Notes')
  for ($c = 0; $c -lt $headers.Count; $c++) {
    $sheet.Cells.Item(1, $c + 1).Value2 = $headers[$c]
  }

  for ($r = 0; $r -lt $rows.Count; $r++) {
    $rowIndex = $r + 2
    $row = $rows[$r]
    $sheet.Cells.Item($rowIndex, 1).Value2 = $row.Parameter
    $sheet.Cells.Item($rowIndex, 2).Value2 = $row.Default
    $sheet.Cells.Item($rowIndex, 3).Value2 = $row.Units
    $sheet.Cells.Item($rowIndex, 4).Value2 = $row.Type
    $sheet.Cells.Item($rowIndex, 5).Value2 = $row.PrimaryMenu
    $sheet.Cells.Item($rowIndex, 6).Value2 = $row.AlsoIn
    $sheet.Cells.Item($rowIndex, 7).Value2 = $row.BackingField
    $sheet.Cells.Item($rowIndex, 8).Value2 = $row.Description
    $sheet.Cells.Item($rowIndex, 9).Value2 = $row.UsedIn
    $sheet.Cells.Item($rowIndex, 10).Value2 = $row.Notes
  }

  $lastRow = $rows.Count + 1
  $usedRange = $sheet.Range('A1', "J$lastRow")
  $headerRange = $sheet.Range('A1', 'J1')

  $headerRange.Font.Bold = $true
  $headerRange.Interior.Color = 0xD9EAF7
  $headerRange.HorizontalAlignment = -4108

  $usedRange.WrapText = $true
  $usedRange.VerticalAlignment = -4160
  $usedRange.Borders.LineStyle = 1

  $sheet.Columns.Item('A').ColumnWidth = 23
  $sheet.Columns.Item('B').ColumnWidth = 12
  $sheet.Columns.Item('C').ColumnWidth = 12
  $sheet.Columns.Item('D').ColumnWidth = 10
  $sheet.Columns.Item('E').ColumnWidth = 20
  $sheet.Columns.Item('F').ColumnWidth = 22
  $sheet.Columns.Item('G').ColumnWidth = 28
  $sheet.Columns.Item('H').ColumnWidth = 42
  $sheet.Columns.Item('I').ColumnWidth = 34
  $sheet.Columns.Item('J').ColumnWidth = 36

  $sheet.Rows.Item(1).RowHeight = 24
  $sheet.Rows.Item("2:$lastRow").EntireRow.AutoFit() | Out-Null

  $sheet.Activate() | Out-Null
  $sheet.Application.ActiveWindow.SplitRow = 1
  $sheet.Application.ActiveWindow.FreezePanes = $true
  $sheet.Range('A1:J1').AutoFilter() | Out-Null

  $workbook.SaveAs($outFile, 51)
}
finally {
  if ($workbook -ne $null) {
    try { $workbook.Close($false) } catch {}
  }
  if ($excel -ne $null) {
    try { $excel.Quit() } catch {}
  }
  if ($sheet -ne $null) {
    try { [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($sheet) } catch {}
  }
  if ($workbook -ne $null) {
    try { [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($workbook) } catch {}
  }
  if ($excel -ne $null) {
    try { [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($excel) } catch {}
  }
  [GC]::Collect()
  [GC]::WaitForPendingFinalizers()
}

Write-Output $outFile
