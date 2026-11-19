# Rebuild the Arduino firmware and restart ball tracking.
$PlatformIOProject = Join-Path $PSScriptRoot "servo_controller"
$TrackingDirectory = $PSScriptRoot
$Python = "python"

$ErrorActionPreference = "Stop"

# Locate PlatformIO (either on PATH or installed in the standard user location).
$PioCommand = Get-Command pio -ErrorAction SilentlyContinue
if ($PioCommand) {
    $Pio = $PioCommand.Source
} else {
    $Pio = Join-Path $HOME ".platformio\penv\Scripts\platformio.exe"
    if (-not (Test-Path $Pio)) {
        throw "PlatformIO not found. Install it or add 'pio' to PATH."
    }
}

if (-not (Test-Path (Join-Path $PlatformIOProject "platformio.ini"))) {
    throw "No platformio.ini in $PlatformIOProject. Update `$PlatformIOProject."
}
if (-not (Test-Path (Join-Path $TrackingDirectory "track_ball.py"))) {
    throw "No track_ball.py in $TrackingDirectory. Update `$TrackingDirectory."
}

Write-Host "Stopping track_ball.py..."
Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match '^(python|pythonw)(\.exe)?$' -and
    $_.CommandLine -match 'track_ball\.py'
} | ForEach-Object {
    Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
}

Push-Location $PlatformIOProject
try {
    Write-Host "Building PlatformIO project..."
    & $Pio run
    if ($LASTEXITCODE -ne 0) { throw "PlatformIO build failed." }

    Write-Host "Uploading firmware..."
    & $Pio run --target upload
    if ($LASTEXITCODE -ne 0) { throw "PlatformIO upload failed." }
} finally {
    Pop-Location
}

Write-Host "Starting track_ball.py..."
Start-Process -FilePath "powershell.exe" -WorkingDirectory $TrackingDirectory -ArgumentList @(
    "-NoExit", "-Command", "& '$Python' 'track_ball.py'"
)
Write-Host "Done."
