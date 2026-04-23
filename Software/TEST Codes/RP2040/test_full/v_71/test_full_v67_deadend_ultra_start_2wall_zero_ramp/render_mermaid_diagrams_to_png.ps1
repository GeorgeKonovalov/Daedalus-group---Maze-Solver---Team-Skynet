$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$markdownPath = Join-Path $root 'test_full_v67_deadend_ultra_start_2wall_zero_ramp_diagrams.md'
$outputDir = Join-Path $root 'png'
$tempDir = Join-Path $root 'render_tmp'
$edgePath = 'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'

if (!(Test-Path $markdownPath)) {
  throw "Diagram markdown not found: $markdownPath"
}

if (!(Test-Path $edgePath)) {
  throw "Microsoft Edge not found at: $edgePath"
}

New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
New-Item -ItemType Directory -Path $tempDir -Force | Out-Null

$content = Get-Content -Path $markdownPath -Raw -Encoding UTF8
$lines = $content -split "`r?`n"

$diagrams = New-Object System.Collections.Generic.List[object]
$currentTitle = $null
$insideMermaid = $false
$buffer = New-Object System.Collections.Generic.List[string]

foreach ($line in $lines) {
  if ($line -match '^##\s+(.*)$') {
    $currentTitle = $Matches[1].Trim()
    continue
  }

  if ($line -eq '```mermaid') {
    $insideMermaid = $true
    $buffer.Clear()
    continue
  }

  if ($insideMermaid -and $line -eq '```') {
    $insideMermaid = $false
    if ($currentTitle) {
      $diagrams.Add([PSCustomObject]@{
        Title = $currentTitle
        Code  = ($buffer -join "`n")
      })
    }
    continue
  }

  if ($insideMermaid) {
    $buffer.Add($line)
  }
}

if ($diagrams.Count -eq 0) {
  throw "No mermaid diagrams found in $markdownPath"
}

function Get-Slug([string]$text) {
  $slug = $text.ToLowerInvariant()
  $slug = [regex]::Replace($slug, '[^a-z0-9]+', '_')
  $slug = $slug.Trim('_')
  if ([string]::IsNullOrWhiteSpace($slug)) {
    return 'diagram'
  }
  return $slug
}

$rendered = @()
$index = 1
foreach ($diagram in $diagrams) {
  $slug = "{0:D2}_{1}" -f $index, (Get-Slug $diagram.Title)
  $htmlPath = Join-Path $tempDir "$slug.html"
  $pngPath = Join-Path $outputDir "$slug.png"

  $escapedCode = [System.Net.WebUtility]::HtmlEncode($diagram.Code)
  $html = @"
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>$($diagram.Title)</title>
  <style>
    html, body {
      margin: 0;
      padding: 0;
      background: #ffffff;
      overflow: hidden;
      font-family: Arial, sans-serif;
    }
    body {
      display: flex;
      align-items: flex-start;
      justify-content: flex-start;
    }
    #wrap {
      padding: 24px;
      display: inline-block;
      background: #ffffff;
    }
    .mermaid svg {
      max-width: none !important;
      height: auto !important;
    }
  </style>
  <script type="module">
    import mermaid from 'https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.esm.min.mjs';
    mermaid.initialize({
      startOnLoad: false,
      securityLevel: 'loose',
      theme: 'default',
      flowchart: { useMaxWidth: false, htmlLabels: true },
      sequence: { useMaxWidth: false },
      gantt: { useMaxWidth: false }
    });

    window.addEventListener('load', async () => {
      const host = document.querySelector('.mermaid');
      const code = host.textContent;
      const id = 'm' + Math.random().toString(36).slice(2);
      const result = await mermaid.render(id, code);
      host.innerHTML = result.svg;

      const svg = host.querySelector('svg');
      if (svg) {
        const bbox = svg.getBBox();
        const width = Math.ceil(bbox.width + 64);
        const height = Math.ceil(bbox.height + 64);
        document.body.style.width = width + 'px';
        document.body.style.height = height + 'px';
      }
      document.body.setAttribute('data-rendered', 'true');
    });
  </script>
</head>
<body>
  <div id="wrap">
    <div class="mermaid">$escapedCode</div>
  </div>
</body>
</html>
"@

  Set-Content -Path $htmlPath -Value $html -Encoding UTF8

  $uri = [System.Uri]::new($htmlPath).AbsoluteUri
  & $edgePath `
    --headless=new `
    --disable-gpu `
    --hide-scrollbars `
    --allow-file-access-from-files `
    --run-all-compositor-stages-before-draw `
    --virtual-time-budget=10000 `
    --window-size=2400,1800 `
    "--screenshot=$pngPath" `
    $uri | Out-Null

  if (!(Test-Path $pngPath)) {
    throw "Failed to render PNG for '$($diagram.Title)'"
  }

  $rendered += [PSCustomObject]@{
    Title = $diagram.Title
    File  = $pngPath
  }

  $index++
}

$rendered | ForEach-Object { $_.File }
