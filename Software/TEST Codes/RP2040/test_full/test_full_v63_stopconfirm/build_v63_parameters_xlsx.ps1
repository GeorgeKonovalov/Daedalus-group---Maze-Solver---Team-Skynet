$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$temp = Join-Path $root ("_xlsx_build_" + [Guid]::NewGuid().ToString("N"))
$outFile = Join-Path $root 'test_full_v63_stopconfirm_parameters.xlsx'

if (Test-Path $outFile) {
  Remove-Item -Force $outFile
}

New-Item -ItemType Directory -Path $temp | Out-Null
New-Item -ItemType Directory -Path (Join-Path $temp '_rels') | Out-Null
New-Item -ItemType Directory -Path (Join-Path $temp 'xl') | Out-Null
New-Item -ItemType Directory -Path (Join-Path $temp 'xl\_rels') | Out-Null
New-Item -ItemType Directory -Path (Join-Path $temp 'xl\worksheets') | Out-Null
$zipFile = Join-Path $temp 'workbook.zip'

function Escape-Xml([string]$text) {
  return [System.Security.SecurityElement]::Escape($text)
}

function Get-ColumnName([int]$index) {
  $name = ''
  while ($index -gt 0) {
    $index--
    $name = [char](65 + ($index % 26)) + $name
    $index = [math]::Floor($index / 26)
  }
  return $name
}

$rows = @(
  @('Parameter', 'Current/Default Value', 'Description', 'Units', 'Menu Location', 'Code Location', 'Notes'),
  @('FrontStop', '15.0', 'Hard stop threshold that sends the robot into post-stop correction when front distance drops below the limit.', 'cm', 'Settings / Event Test Config', 'RuntimeSettings, RuntimeConfig, updateExecutive(), updatePreviewExecution(), updateEventTestRun()', 'Stop rule now uses strict front < FrontStop in the main navigation handling paths.'),
  @('WallThreshold', '16.0', 'Side-wall threshold used for wall presence and for the new correction-stage ultrasonic dead-end confirmation.', 'cm', 'Settings / Event Test Config', 'classifyCandidateEvent(), choosePidSource(), correctionStageUltrasonicDeadEndDetected()', 'Correction-stage dead-end requires front, left, and right all below this threshold.'),
  @('DeadEndThreshold', '20.0', 'Legacy dead-end distance threshold that works together with the two front IR sensors.', 'cm', 'Settings / Event Test Config / IR Sensor Test', 'classifyCandidateEvent(), legacyDeadEndStopTriggered()', 'Legacy dead-end path is preserved and excludes correction-stage overrides.'),
  @('Front Hold Ref', '3.5', 'Target front distance held during the non-dead-end correction stage before release.', 'cm', 'Settings / Event Test Config', 'computeFrontNormalizedError(), computeFrontPidCorrection(), frontNormalizationComplete()', 'Used only for correction stages that are not dead-end mode.'),
  @('Front PID P', '0.150', 'Proportional gain for front-distance correction while stopped.', 'normalized', 'Settings / Event Test Config', 'commitRuntimeConfig(), computeFrontPidCorrection()', 'Renewed in this version from the base.'),
  @('Front PID I', '0.000', 'Integral gain for front-distance correction while stopped.', 'normalized', 'Settings / Event Test Config', 'commitRuntimeConfig(), computeFrontPidCorrection()', 'Kept at zero by default.'),
  @('Front PID D', '0.040', 'Derivative gain for front-distance correction while stopped.', 'normalized', 'Settings / Event Test Config', 'commitRuntimeConfig(), computeFrontPidCorrection()', 'Renewed in this version.'),
  @('Turn Yaw Tol', '3.0', 'Yaw acceptance band required before the robot leaves the correction stage.', 'deg', 'Settings / Event Test Config', 'yawNormalizationComplete(), frontNormalizationComplete(), deadEndNormalizationComplete()', 'Works together with IMU deadband/tolerance logic.'),
  @('Match N', '2', 'How many matching observations are needed to confirm a correction-stage event.', 'count', 'IR Sensor Test / Event Test Config', 'updateEventConfirmState()', 'Still used for left/right/T confirmation during correction.'),
  @('Check gap', '0.01', 'Delay between confirmation samples.', 's', 'IR Sensor Test / Event Test Config', 'updateEventConfirmState(), updateCorridorConfirmState()', 'Smaller values confirm faster but are more sensitive to noise.'),
  @('2-wall P', '0.250', 'Shared 2-wall PID proportional gain.', 'normalized', 'PID / Event Test Config', 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), computePidWithRuntime()', 'Renewed in this version.'),
  @('2-wall D', '0.020', 'Shared 2-wall PID derivative gain.', 'normalized', 'PID / Event Test Config', 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), computePidWithRuntime()', 'Renewed in this version.'),
  @('IMU P', '3.9000', 'Yaw-hold proportional gain.', 'normalized', 'IMU PID Menu / Event Test Config', 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), currentImuRotationCompensation()', 'Renewed in this version.'),
  @('IMU I', '1.7000', 'Yaw-hold integral gain.', 'normalized', 'IMU PID Menu / Event Test Config', 'RuntimeSettings, RuntimeConfig, commitRuntimeConfig(), currentImuRotationCompensation()', 'Renewed in this version.'),
  @('overallSpeedActual', '0.0 in FrontNormalize', 'Live execution speed variable forced to zero while the correction stage is active.', 'runtime', 'Internal only', 'enterPhase(), startPreviewPhase(), startEventPhase(), updateOverallSpeedActualForState()', 'This is the condition under which correction plus confirmation runs.'),
  @('correctionConfirmEnabled', 'true except legacy dead-end stops', 'Internal flag that decides whether correction-stage confirmation is allowed to override the current stop result.', 'flag', 'Internal only', 'ExecutiveState / PreviewState / EventTestState, updateExecutive(), updatePreviewExecution(), updateEventTestRun()', 'False for legacy dead-end; true for turn-triggered and plain FrontStop-triggered correction.'),
  @('legacyDeadEndStopTriggered', 'Derived', 'Internal helper for the preserved old dead-end rule: front distance below DeadEndThreshold and both front IR sensors triggered.', 'helper', 'Internal only', 'legacyDeadEndStopTriggered()', 'If true, correction-stage override logic is disabled.'),
  @('correctionStageDeadEnd', 'Derived', 'New correction-stage-only dead-end confirmation using front/left/right ultrasonic block condition.', 'helper', 'Internal only', 'correctionStageUltrasonicDeadEndDetected(), correctionStageCandidate()', 'Only active after stop, never used as the normal running dead-end trigger.')
)

$lastRow = $rows.Count
$lastCol = $rows[0].Count
$lastCell = ('{0}{1}' -f (Get-ColumnName $lastCol), $lastRow)

$widths = @(20, 18, 48, 12, 24, 42, 42)
$colsXml = ''
for ($i = 0; $i -lt $widths.Count; $i++) {
  $col = $i + 1
  $colsXml += "<col min=`"$col`" max=`"$col`" width=`"$($widths[$i])`" customWidth=`"1`"/>"
}

$rowsXml = New-Object System.Text.StringBuilder
for ($r = 0; $r -lt $rows.Count; $r++) {
  $rowNum = $r + 1
  [void]$rowsXml.Append("<row r=`"$rowNum`">")
  for ($c = 0; $c -lt $rows[$r].Count; $c++) {
    $cellRef = ('{0}{1}' -f (Get-ColumnName ($c + 1)), $rowNum)
    $escaped = Escape-Xml ([string]$rows[$r][$c])
    [void]$rowsXml.Append("<c r=`"$cellRef`" t=`"inlineStr`"><is><t xml:space=`"preserve`">$escaped</t></is></c>")
  }
  [void]$rowsXml.Append('</row>')
}

$contentTypes = @'
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
  <Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
</Types>
'@

$rels = @'
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
</Relationships>
'@

$workbook = @'
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <sheets>
    <sheet name="Parameters" sheetId="1" r:id="rId1"/>
  </sheets>
</workbook>
'@

$workbookRels = @'
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
</Relationships>
'@

$sheetXml = @"
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">
  <sheetViews>
    <sheetView workbookViewId="0">
      <pane ySplit="1" topLeftCell="A2" activePane="bottomLeft" state="frozen"/>
      <selection pane="bottomLeft" activeCell="A2" sqref="A2"/>
    </sheetView>
  </sheetViews>
  <dimension ref="A1:$lastCell"/>
  <sheetFormatPr defaultRowHeight="15"/>
  <cols>$colsXml</cols>
  <sheetData>$rowsXml</sheetData>
  <autoFilter ref="A1:$lastCell"/>
</worksheet>
"@

$utf8 = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText((Join-Path $temp '[Content_Types].xml'), $contentTypes, $utf8)
[System.IO.File]::WriteAllText((Join-Path $temp '_rels\.rels'), $rels, $utf8)
[System.IO.File]::WriteAllText((Join-Path $temp 'xl\workbook.xml'), $workbook, $utf8)
[System.IO.File]::WriteAllText((Join-Path $temp 'xl\_rels\workbook.xml.rels'), $workbookRels, $utf8)
[System.IO.File]::WriteAllText((Join-Path $temp 'xl\worksheets\sheet1.xml'), $sheetXml, $utf8)

Compress-Archive -Path (Join-Path $temp '*') -DestinationPath $zipFile -CompressionLevel Optimal
Start-Sleep -Milliseconds 500
[System.IO.File]::Copy($zipFile, $outFile, $true)
try { Remove-Item -Force $zipFile -ErrorAction Stop } catch {}
try { Remove-Item -Recurse -Force $temp -ErrorAction Stop } catch {}
Write-Output $outFile
