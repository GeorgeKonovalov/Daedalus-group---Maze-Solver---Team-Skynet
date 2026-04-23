$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$markdownPath = Join-Path $root 'test_full_v67_deadend_ultra_start_2wall_zero_ramp_logic_block_schemes.md'
$outputDir = Join-Path $root 'png_logic_schemes'
$tempDir = Join-Path $root 'render_logic_tmp'
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

Add-Type -ReferencedAssemblies System.Drawing -TypeDefinition @'
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;

public static class PngWhitespaceCropper {
  public static void Crop(string path, byte threshold, int padding) {
    using (Bitmap source = new Bitmap(path))
    using (Bitmap bitmap = new Bitmap(source.Width, source.Height, PixelFormat.Format32bppArgb)) {
      using (Graphics graphics = Graphics.FromImage(bitmap)) {
        graphics.DrawImage(source, 0, 0, source.Width, source.Height);
      }

      Rectangle rect = new Rectangle(0, 0, bitmap.Width, bitmap.Height);
      BitmapData data = bitmap.LockBits(rect, ImageLockMode.ReadOnly, PixelFormat.Format32bppArgb);
      int stride = Math.Abs(data.Stride);
      byte[] bytes = new byte[stride * bitmap.Height];
      Marshal.Copy(data.Scan0, bytes, 0, bytes.Length);
      bitmap.UnlockBits(data);

      int minX = bitmap.Width;
      int minY = bitmap.Height;
      int maxX = -1;
      int maxY = -1;

      for (int y = 0; y < bitmap.Height; y++) {
        int row = y * stride;
        for (int x = 0; x < bitmap.Width; x++) {
          int offset = row + (x * 4);
          byte b = bytes[offset];
          byte g = bytes[offset + 1];
          byte r = bytes[offset + 2];
          byte a = bytes[offset + 3];
          if (a > 0 && (r < threshold || g < threshold || b < threshold)) {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
          }
        }
      }

      if (maxX < minX || maxY < minY) return;

      minX = Math.Max(0, minX - padding);
      minY = Math.Max(0, minY - padding);
      maxX = Math.Min(bitmap.Width - 1, maxX + padding);
      maxY = Math.Min(bitmap.Height - 1, maxY + padding);

      Rectangle cropRect = new Rectangle(minX, minY, maxX - minX + 1, maxY - minY + 1);
      using (Bitmap cropped = new Bitmap(cropRect.Width, cropRect.Height, PixelFormat.Format32bppArgb)) {
        using (Graphics cropGraphics = Graphics.FromImage(cropped)) {
          cropGraphics.Clear(Color.White);
          Rectangle destRect = new Rectangle(0, 0, cropRect.Width, cropRect.Height);
          cropGraphics.DrawImage(bitmap, destRect, cropRect, GraphicsUnit.Pixel);
        }

        string tempPath = path + ".tmp.png";
        cropped.Save(tempPath, ImageFormat.Png);
        bitmap.Dispose();
        source.Dispose();
        if (System.IO.File.Exists(path)) System.IO.File.Delete(path);
        System.IO.File.Move(tempPath, path);
      }
    }
  }
}
'@

function Crop-PngWhitespace([string]$path) {
  [PngWhitespaceCropper]::Crop($path, 245, 32)
  return

  Add-Type -AssemblyName System.Drawing

  $source = [System.Drawing.Bitmap]::FromFile($path)
  $bitmap = New-Object System.Drawing.Bitmap $source.Width, $source.Height, ([System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
  $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
  $graphics.DrawImage($source, 0, 0, $source.Width, $source.Height)
  $graphics.Dispose()
  $source.Dispose()

  $rect = New-Object System.Drawing.Rectangle 0, 0, $bitmap.Width, $bitmap.Height
  $data = $bitmap.LockBits($rect, [System.Drawing.Imaging.ImageLockMode]::ReadOnly, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
  $stride = [Math]::Abs($data.Stride)
  $bytes = New-Object byte[] ($stride * $bitmap.Height)
  [System.Runtime.InteropServices.Marshal]::Copy($data.Scan0, $bytes, 0, $bytes.Length)
  $bitmap.UnlockBits($data)

  $minX = $bitmap.Width
  $minY = $bitmap.Height
  $maxX = -1
  $maxY = -1

  for ($y = 0; $y -lt $bitmap.Height; $y++) {
    $row = $y * $stride
    for ($x = 0; $x -lt $bitmap.Width; $x++) {
      $offset = $row + ($x * 4)
      $b = $bytes[$offset]
      $g = $bytes[$offset + 1]
      $r = $bytes[$offset + 2]
      $a = $bytes[$offset + 3]
      if ($a -gt 0 -and ($r -lt 245 -or $g -lt 245 -or $b -lt 245)) {
        if ($x -lt $minX) { $minX = $x }
        if ($y -lt $minY) { $minY = $y }
        if ($x -gt $maxX) { $maxX = $x }
        if ($y -gt $maxY) { $maxY = $y }
      }
    }
  }

  if ($maxX -lt $minX -or $maxY -lt $minY) {
    $bitmap.Dispose()
    return
  }

  $padding = 32
  $minX = [Math]::Max(0, $minX - $padding)
  $minY = [Math]::Max(0, $minY - $padding)
  $maxX = [Math]::Min($bitmap.Width - 1, $maxX + $padding)
  $maxY = [Math]::Min($bitmap.Height - 1, $maxY + $padding)

  $cropRect = New-Object System.Drawing.Rectangle $minX, $minY, ($maxX - $minX + 1), ($maxY - $minY + 1)
  $cropped = New-Object System.Drawing.Bitmap $cropRect.Width, $cropRect.Height, ([System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
  $cropGraphics = [System.Drawing.Graphics]::FromImage($cropped)
  $cropGraphics.Clear([System.Drawing.Color]::White)
  $destRect = New-Object System.Drawing.Rectangle 0, 0, $cropRect.Width, $cropRect.Height
  $cropGraphics.DrawImage($bitmap, $destRect, $cropRect, [System.Drawing.GraphicsUnit]::Pixel)
  $cropGraphics.Dispose()
  $bitmap.Dispose()

  $tempPath = "$path.tmp.png"
  $cropped.Save($tempPath, [System.Drawing.Imaging.ImageFormat]::Png)
  $cropped.Dispose()
  Move-Item -LiteralPath $tempPath -Destination $path -Force
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
    --window-size=5000,5000 `
    "--screenshot=$pngPath" `
    $uri | Out-Null

  if (!(Test-Path $pngPath)) {
    throw "Failed to render PNG for '$($diagram.Title)'"
  }

  Crop-PngWhitespace $pngPath

  $rendered += [PSCustomObject]@{
    Title = $diagram.Title
    File  = $pngPath
  }

  $index++
}

$rendered | ForEach-Object { $_.File }
