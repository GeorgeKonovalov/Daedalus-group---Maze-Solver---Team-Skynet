$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$outFile = Join-Path $root 'test_full_v66_startcorr_parameters.xlsx'

$rows = @(
  [PSCustomObject]@{
    Parameter = 'Parameter'
    Value = 'Current/Default Value'
    Description = 'Description'
    Units = 'Units'
    Menu = 'Menu Location'
    Code = 'Code Location'
    Notes = 'Notes'
  }
  [PSCustomObject]@{
    Parameter = 'Overall Start'
    Value = '0.20'
    Description = 'Start-limited live speed used when execution begins and now again after Start correction completes.'
    Units = 'normalized'
    Menu = 'Start Settings / Event Test Config'
    Code = 'initializeRuntimeOverallSpeed(), updateExecutive(), updatePreviewExecution(), updateEventTestRun()'
    Notes = 'In v66 the start ramp does not begin until Start correction has finished; this value is reloaded immediately before ramped release.'
  }
  [PSCustomObject]@{
    Parameter = 'Overall Ramp K'
    Value = '2.0'
    Description = 'Linear ramp slope that raises overallSpeedActual toward the configured Overall speed.'
    Units = 'speed units/s'
    Menu = 'Start Settings / Event Test Config'
    Code = 'updateOverallSpeedActualForState()'
    Notes = 'v66 keeps the same ramp law but shifts its start point to after the new Start correction stage.'
  }
  [PSCustomObject]@{
    Parameter = 'FrontStop'
    Value = '15.0'
    Description = 'Hard stop threshold. When front drops below this value, the robot enters the correction stage.'
    Units = 'cm'
    Menu = 'Settings / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, updateExecutive(), updateEventTestRun()'
    Notes = 'Unchanged for normal event handling. The new v66 change is only that Start now corrects before the ramp resumes.'
  }
  [PSCustomObject]@{
    Parameter = 'Front Hold Ref'
    Value = '3.5'
    Description = 'Target front distance used by the front PID while stopped in non-dead-end correction.'
    Units = 'cm'
    Menu = 'Settings / Event Test Config'
    Code = 'computeFrontNormalizedError(), computeFrontPidCorrection(), frontNormalizationComplete()'
    Notes = 'Used by turn-style correction. In v66 Start is excluded from front hold and uses wall/yaw correction only.'
  }
  [PSCustomObject]@{
    Parameter = 'WallThreshold'
    Value = '16.0'
    Description = 'Side-wall threshold for wall presence and correction-stage ultrasonic dead-end confirmation.'
    Units = 'cm'
    Menu = 'Settings / Event Test Config'
    Code = 'classifyCandidateEvent(), choosePidSource(), correctionStageUltrasonicDeadEndDetected()'
    Notes = 'Still governs wall presence and the correction-stage all-ultrasonic dead-end rule.'
  }
  [PSCustomObject]@{
    Parameter = 'DeadEndThreshold'
    Value = '20.0'
    Description = 'Legacy dead-end threshold used together with both front IR sensors.'
    Units = 'cm'
    Menu = 'Settings / Event Test Config'
    Code = 'classifyCandidateEvent(), legacyDeadEndStopTriggered()'
    Notes = 'Legacy dead-end handling remains unchanged; v66 only blocks Start correction from being promoted into dead-end.'
  }
  [PSCustomObject]@{
    Parameter = 'Match N'
    Value = '2'
    Description = 'Number of matching observations required to confirm a correction-stage event.'
    Units = 'count'
    Menu = 'IR Sensor Test / Event Test Config'
    Code = 'updateEventConfirmState()'
    Notes = 'Still used for correction-stage confirmation of turn/dead-end events.'
  }
  [PSCustomObject]@{
    Parameter = 'Check gap'
    Value = '0.01'
    Description = 'Delay between confirmation samples.'
    Units = 's'
    Menu = 'IR Sensor Test / Event Test Config'
    Code = 'updateEventConfirmState(), updateCorridorConfirmState()'
    Notes = 'Lower values confirm faster but reduce noise immunity.'
  }
  [PSCustomObject]@{
    Parameter = '1-wall P'
    Value = '0.350'
    Description = 'One-wall PID proportional gain.'
    Units = 'normalized'
    Menu = 'PID / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), computePidWithRuntime()'
    Notes = 'Current renewed default preserved.'
  }
  [PSCustomObject]@{
    Parameter = '1-wall D'
    Value = '0.040'
    Description = 'One-wall PID derivative gain.'
    Units = 'normalized'
    Menu = 'PID / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), computePidWithRuntime()'
    Notes = 'Current renewed default preserved.'
  }
  [PSCustomObject]@{
    Parameter = '2-wall P'
    Value = '0.250'
    Description = 'Two-wall PID proportional gain.'
    Units = 'normalized'
    Menu = 'PID / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), computePidWithRuntime()'
    Notes = 'Renewed default kept from the prior runtime updates.'
  }
  [PSCustomObject]@{
    Parameter = '2-wall D'
    Value = '0.020'
    Description = 'Two-wall PID derivative gain.'
    Units = 'normalized'
    Menu = 'PID / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), computePidWithRuntime()'
    Notes = 'Renewed default kept from the prior runtime updates.'
  }
  [PSCustomObject]@{
    Parameter = 'IMU P'
    Value = '3.9000'
    Description = 'Yaw-hold proportional gain.'
    Units = 'normalized'
    Menu = 'IMU PID Menu / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), currentImuRotationCompensation()'
    Notes = 'Current default preserved.'
  }
  [PSCustomObject]@{
    Parameter = 'IMU I'
    Value = '1.7000'
    Description = 'Yaw-hold integral gain.'
    Units = 'normalized'
    Menu = 'IMU PID Menu / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), currentImuRotationCompensation()'
    Notes = 'Current default preserved.'
  }
  [PSCustomObject]@{
    Parameter = 'Front PID P'
    Value = '0.150'
    Description = 'Front-distance proportional gain used during stopped correction.'
    Units = 'normalized'
    Menu = 'Settings / Event Test Config'
    Code = 'commitRuntimeConfig(), computeFrontPidCorrection()'
    Notes = 'Still used for turn-style correction. Start correction bypasses this in v66.'
  }
  [PSCustomObject]@{
    Parameter = 'Front PID D'
    Value = '0.040'
    Description = 'Front-distance derivative gain used during stopped correction.'
    Units = 'normalized'
    Menu = 'Settings / Event Test Config'
    Code = 'commitRuntimeConfig(), computeFrontPidCorrection()'
    Notes = 'Still used for turn-style correction. Start correction bypasses this in v66.'
  }
  [PSCustomObject]@{
    Parameter = 'correctionConfirmEnabled'
    Value = 'Runtime flag'
    Description = 'Allows stopped correction to confirm and override the pending event.'
    Units = 'flag'
    Menu = 'Internal only'
    Code = 'ExecutiveState / PreviewState / EventTestState, FrontNormalize flows'
    Notes = 'False during Start correction in v66 because Start is already confirmed and only needs settling before release.'
  }
  [PSCustomObject]@{
    Parameter = 'postCorrectionReleaseActive'
    Value = 'Runtime flag'
    Description = 'Release guard that lets the robot depart after correction without being trapped immediately back into FrontStop.'
    Units = 'flag'
    Menu = 'Internal only'
    Code = 'holdPostCorrectionRelease(), AcquireCorridor in preview/runtime/event-test flows'
    Notes = 'Reused in v66 after Start correction so the robot can depart cleanly before normal corridor flow resumes.'
  }
  [PSCustomObject]@{
    Parameter = 'overallSpeedActual'
    Value = '0 in FrontNormalize'
    Description = 'Live execution speed used by the drive path.'
    Units = 'runtime'
    Menu = 'Internal only'
    Code = 'setOverallSpeedActual(), initializeRuntimeOverallSpeed(), updateOverallSpeedActualForState(), driveBodyFrameLimited()'
    Notes = 'Forced to 0 during correction. In v66 it is explicitly reset to Overall Start after Start correction, then ramped upward.'
  }
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
  $sheet.Name = 'Parameters'

  $headers = @('Parameter', 'Current/Default Value', 'Description', 'Units', 'Menu Location', 'Code Location', 'Notes')
  for ($c = 0; $c -lt $headers.Count; $c++) {
    $sheet.Cells.Item(1, $c + 1).Value2 = $headers[$c]
  }

  for ($r = 1; $r -lt $rows.Count; $r++) {
    $row = $rows[$r]
    $sheet.Cells.Item($r + 1, 1).Value2 = $row.Parameter
    $sheet.Cells.Item($r + 1, 2).Value2 = $row.Value
    $sheet.Cells.Item($r + 1, 3).Value2 = $row.Description
    $sheet.Cells.Item($r + 1, 4).Value2 = $row.Units
    $sheet.Cells.Item($r + 1, 5).Value2 = $row.Menu
    $sheet.Cells.Item($r + 1, 6).Value2 = $row.Code
    $sheet.Cells.Item($r + 1, 7).Value2 = $row.Notes
  }

  $lastRow = $rows.Count
  $usedRange = $sheet.Range('A1', "G$lastRow")
  $headerRange = $sheet.Range('A1', 'G1')

  $headerRange.Font.Bold = $true
  $headerRange.Interior.Color = 0xD9EAF7
  $headerRange.HorizontalAlignment = -4108

  $usedRange.WrapText = $true
  $usedRange.VerticalAlignment = -4160
  $usedRange.Borders.LineStyle = 1

  $sheet.Columns.Item('A').ColumnWidth = 22
  $sheet.Columns.Item('B').ColumnWidth = 18
  $sheet.Columns.Item('C').ColumnWidth = 46
  $sheet.Columns.Item('D').ColumnWidth = 12
  $sheet.Columns.Item('E').ColumnWidth = 24
  $sheet.Columns.Item('F').ColumnWidth = 40
  $sheet.Columns.Item('G').ColumnWidth = 44

  $sheet.Rows.Item(1).RowHeight = 24
  $sheet.Rows.Item("2:$lastRow").EntireRow.AutoFit() | Out-Null

  $sheet.Activate() | Out-Null
  $sheet.Application.ActiveWindow.SplitRow = 1
  $sheet.Application.ActiveWindow.FreezePanes = $true
  $sheet.Range('A1:G1').AutoFilter() | Out-Null

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
