$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$outFile = Join-Path $root 'test_full_v64_execresume_parameters.xlsx'

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
    Parameter = 'FrontStop'
    Value = '15.0'
    Description = 'Hard stop threshold. When front drops below this value, the robot enters the correction stage.'
    Units = 'cm'
    Menu = 'Settings / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, updateExecutive(), updateEventTestRun()'
    Notes = 'v64 keeps the strict front < FrontStop rule, then releases immediately after correction using the confirmed event.'
  }
  [PSCustomObject]@{
    Parameter = 'Front Hold Ref'
    Value = '3.5'
    Description = 'Target front distance used by the front PID while stopped in non-dead-end correction.'
    Units = 'cm'
    Menu = 'Settings / Event Test Config'
    Code = 'computeFrontNormalizedError(), computeFrontPidCorrection(), frontNormalizationComplete()'
    Notes = 'Used only during correction; dead-end normalization still excludes front hold.'
  }
  [PSCustomObject]@{
    Parameter = 'WallThreshold'
    Value = '16.0'
    Description = 'Side-wall threshold for wall presence and correction-stage ultrasonic dead-end confirmation.'
    Units = 'cm'
    Menu = 'Settings / Event Test Config'
    Code = 'classifyCandidateEvent(), choosePidSource(), correctionStageUltrasonicDeadEndDetected()'
    Notes = 'Correction-stage dead-end requires front, left, and right all below this threshold.'
  }
  [PSCustomObject]@{
    Parameter = 'DeadEndThreshold'
    Value = '20.0'
    Description = 'Legacy dead-end threshold used together with both front IR sensors.'
    Units = 'cm'
    Menu = 'Settings / Event Test Config'
    Code = 'classifyCandidateEvent(), legacyDeadEndStopTriggered()'
    Notes = 'Legacy dead-end trigger still blocks correction-stage event overrides.'
  }
  [PSCustomObject]@{
    Parameter = 'Match N'
    Value = '2'
    Description = 'Number of matching observations required to confirm a correction-stage event.'
    Units = 'count'
    Menu = 'IR Sensor Test / Event Test Config'
    Code = 'updateEventConfirmState()'
    Notes = 'Used for left, right, T-junction, and correction-stage dead-end confirmation.'
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
    Parameter = '2-wall P'
    Value = '0.250'
    Description = 'Two-wall PID proportional gain.'
    Units = 'normalized'
    Menu = 'PID / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), computePidWithRuntime()'
    Notes = 'Renewed default kept from the prior stop-confirm update.'
  }
  [PSCustomObject]@{
    Parameter = '2-wall D'
    Value = '0.020'
    Description = 'Two-wall PID derivative gain.'
    Units = 'normalized'
    Menu = 'PID / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), computePidWithRuntime()'
    Notes = 'Renewed default kept from the prior stop-confirm update.'
  }
  [PSCustomObject]@{
    Parameter = 'IMU P'
    Value = '3.9000'
    Description = 'Yaw-hold proportional gain.'
    Units = 'normalized'
    Menu = 'IMU PID Menu / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), currentImuRotationCompensation()'
    Notes = 'Current default preserved in v64.'
  }
  [PSCustomObject]@{
    Parameter = 'IMU I'
    Value = '1.7000'
    Description = 'Yaw-hold integral gain.'
    Units = 'normalized'
    Menu = 'IMU PID Menu / Event Test Config'
    Code = 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), currentImuRotationCompensation()'
    Notes = 'Current default preserved in v64.'
  }
  [PSCustomObject]@{
    Parameter = 'Front PID P'
    Value = '0.150'
    Description = 'Front-distance proportional gain used during stopped correction.'
    Units = 'normalized'
    Menu = 'Settings / Event Test Config'
    Code = 'commitRuntimeConfig(), computeFrontPidCorrection()'
    Notes = 'Renewed default preserved in v64.'
  }
  [PSCustomObject]@{
    Parameter = 'Front PID D'
    Value = '0.040'
    Description = 'Front-distance derivative gain used during stopped correction.'
    Units = 'normalized'
    Menu = 'Settings / Event Test Config'
    Code = 'commitRuntimeConfig(), computeFrontPidCorrection()'
    Notes = 'Renewed default preserved in v64.'
  }
  [PSCustomObject]@{
    Parameter = 'correctionConfirmEnabled'
    Value = 'Runtime flag'
    Description = 'Allows stopped correction to confirm and override the pending event.'
    Units = 'flag'
    Menu = 'Internal only'
    Code = 'ExecutiveState / PreviewState / EventTestState, FrontNormalize flows'
    Notes = 'False for legacy dead-end stops, true for turn-triggered or plain FrontStop-triggered correction.'
  }
  [PSCustomObject]@{
    Parameter = 'postCorrectionReleaseActive'
    Value = 'Runtime flag'
    Description = 'Short release guard after correction so the newly confirmed event actually takes over before the next stop gate can re-trigger.'
    Units = 'flag'
    Menu = 'Internal only'
    Code = 'holdPostCorrectionRelease(), AcquireCorridor in preview/runtime/event-test flows'
    Notes = 'Added in v64 to stop the correction-complete -> immediate-stop loop.'
  }
  [PSCustomObject]@{
    Parameter = 'overallSpeedActual'
    Value = '0 in FrontNormalize'
    Description = 'Live execution speed used by the drive path.'
    Units = 'runtime'
    Menu = 'Internal only'
    Code = 'setOverallSpeedActual(), updateOverallSpeedActualForState(), driveBodyFrameLimited()'
    Notes = 'Correction runs with overallSpeedActual forced to zero.'
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
  $sheet.Columns.Item('G').ColumnWidth = 42

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
