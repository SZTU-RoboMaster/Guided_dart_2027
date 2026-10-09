# @file flash_swd.ps1
# @brief 通过 ST-Link/OpenOCD 按指定模式烧录已生成的固件镜像。
# @details Boot、App、BootApp 和 All 模式使用各自的镜像与固定装载地址。
# @author Guided Dart 项目维护者
# @date 2026-10-07
# @version 1.0
# @note 普通 App 升级优先使用上位机 LoRa OTA；本脚本会改写内部 Flash。
param(
    [ValidateSet("Boot", "App", "BootApp", "All")]
    [string]$Mode = "All",
    [string]$Image = "",
    [string]$Python = "python",
    [string]$OpenOcd = "",
    [string]$Interface = "interface/stlink.cfg",
    [string]$Target = "target/stm32g4x.cfg"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$Modes = @{
    Boot = @{ Name = "Guided_dart_bootloader.bin"; Address = "0x08000000"; Size = 0x4000 }
    App  = @{ Name = "Guided_dart_app_swd.bin"; Address = "0x08004000"; Size = 0x1C000 }
    All  = @{ Name = "Guided_dart_initial_full.bin"; Address = "0x08000000"; Size = 0x20000 }
}
$Selected = if ($Mode -eq "BootApp") { $null } else { $Modes[$Mode] }
if ([string]::IsNullOrWhiteSpace($Image)) {
    if ($Mode -eq "BootApp") {
        $Image = ""
    } else {
        $Image = Join-Path $Root (Join-Path "out\firmware" $Selected.Name)
    }
}

function Resolve-Bin([string]$PathValue) {
    if ([string]::IsNullOrWhiteSpace($PathValue)) { return $null }
    if (-not [System.IO.Path]::IsPathRooted($PathValue)) { $PathValue = Join-Path $Root $PathValue }
    if (-not (Test-Path -LiteralPath $PathValue)) { throw "Firmware image not found: $PathValue. Run build_all.ps1 first." }
    $resolved = (Resolve-Path -LiteralPath $PathValue).Path
    if ([System.IO.Path]::GetExtension($resolved) -ne ".bin") { throw "SWD programming requires a BIN image: $resolved" }
    return $resolved
}

function Verify-Bin([string]$PathValue, [int]$ExpectedSize, [bool]$AllowShort) {
    $size = (Get-Item -LiteralPath $PathValue).Length
    if (($AllowShort -and ($size -lt 8 -or $size -gt $ExpectedSize)) -or
        (-not $AllowShort -and $size -ne $ExpectedSize)) {
        throw "镜像大小错误：$PathValue ($size bytes)"
    }
}

if ($Mode -eq "BootApp") {
    $BootImage = Resolve-Bin (Join-Path $Root "out\firmware\Guided_dart_bootloader.bin")
    $AppImage = Resolve-Bin (Join-Path $Root "out\firmware\Guided_dart_app_swd.bin")
    Verify-Bin $BootImage $Modes.Boot.Size $true
    Verify-Bin $AppImage $Modes.App.Size $false
    & $Python (Join-Path $Root "tools\guided_dart_tool.py") verify-app-swd $AppImage
    if ($LASTEXITCODE -ne 0) { throw "App-SWD image verification failed." }
} else {
    $Image = Resolve-Bin $Image
    Verify-Bin $Image $Selected.Size ($Mode -eq "Boot")
    if ($Mode -eq "App") {
        & $Python (Join-Path $Root "tools\guided_dart_tool.py") verify-app-swd $Image
        if ($LASTEXITCODE -ne 0) { throw "App-SWD image verification failed." }
    } elseif ($Mode -eq "All") {
        & $Python (Join-Path $Root "tools\guided_dart_tool.py") verify-image $Image
        if ($LASTEXITCODE -ne 0) { throw "Full image verification failed." }
    }
}

if ([string]::IsNullOrWhiteSpace($OpenOcd)) {
    $command = Get-Command openocd -ErrorAction SilentlyContinue
    if ($command) {
        $OpenOcd = $command.Source
    } else {
        $OpenOcd = "D:\JetBrains\CLion 2023.3.5\OpenOCD-20231002-0.12.0\bin\openocd.exe"
    }
}
if (-not (Test-Path -LiteralPath $OpenOcd)) {
    throw "OpenOCD not found: $OpenOcd"
}

$Commands = @()
if ($Mode -eq "BootApp") {
    $BootForOpenOcd = $BootImage -replace '\\', '/'
    $AppForOpenOcd = $AppImage -replace '\\', '/'
    Write-Host "Programming Boot: $BootImage"
    Write-Host "Programming App:  $AppImage"
    $Commands += "program `"$BootForOpenOcd`" 0x08000000 verify"
    $Commands += "program `"$AppForOpenOcd`" 0x08004000 verify reset exit"
} else {
    $ImageForOpenOcd = $Image -replace '\\', '/'
    Write-Host "Programming $Mode at $($Selected.Address): $Image"
    $Commands += "program `"$ImageForOpenOcd`" $($Selected.Address) verify reset exit"
}
& $OpenOcd -f $Interface -f $Target -c ($Commands -join '; ')
if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD programming failed. Check ST-Link, target voltage and SWD wiring."
}
Write-Host "SWD programming completed."
