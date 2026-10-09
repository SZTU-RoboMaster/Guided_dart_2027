param(
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
$Root = Split-Path -Parent $PSScriptRoot
$Output = Join-Path $Root "out\groundstation"
$Work = Join-Path $Root "out\build-groundstation"
New-Item -ItemType Directory -Force -Path $Output, $Work | Out-Null
& $Python -m PyInstaller --noconfirm --clean --onefile --windowed `
    --distpath $Output --workpath $Work --specpath $Work `
    --paths $Root --hidden-import lora_profiles --name GuidedDartGroundStation main.py
if ($LASTEXITCODE -ne 0) { throw "Groundstation packaging failed." }
Write-Host "Groundstation generated: $Output\GuidedDartGroundStation.exe"
