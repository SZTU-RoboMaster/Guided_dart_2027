# @file build_all.ps1
# @brief 构建独立 Boot/App、整片首次烧录镜像并运行离线校验。
# @details 生成的独立 App BIN 用于 LoRa OTA；整片 BIN 和 App-SWD BIN 用于有线首次安装或维护。
# @author Guided Dart 项目维护者
# @date 2026-10-07
# @version 1.0
# @note 本脚本不烧录目标板，也不打包上位机安装程序。
param(
    [string]$Configuration = "Release",
    [string]$Generator = "MinGW Makefiles",
    [string]$Toolchain = "",
    [string]$Python = "python",
    [int]$Version = 1
)

$ErrorActionPreference = "Stop"
$Root = $PSScriptRoot
$App = $Root
$Boot = Join-Path $Root "bootloader"
$BuildRoot = Join-Path $Root "out\build"
$ArtifactRoot = Join-Path $Root "out\firmware"
$AppBuild = Join-Path $BuildRoot "app"
$BootBuild = Join-Path $BuildRoot "bootloader"
$FullBin = Join-Path $ArtifactRoot "Guided_dart_initial_full.bin"
$FullElf = Join-Path $ArtifactRoot "Guided_dart_initial_full.elf"
$AppSwdBin = Join-Path $ArtifactRoot "Guided_dart_app_swd.bin"

if ([string]::IsNullOrWhiteSpace($Toolchain)) {
    $candidates = @(
        (Join-Path ${env:ProgramFiles} "GNU Tools ARM Embedded\arm-none-eabi\bin\arm-none-eabi-gcc.exe"),
        "D:\JetBrains\CLion 2023.3.5\arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi\bin\arm-none-eabi-gcc.exe"
    )
    $Toolchain = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
}
if ([string]::IsNullOrWhiteSpace($Toolchain) -or -not (Test-Path $Toolchain)) {
    throw "arm-none-eabi-gcc not found; specify -Toolchain."
}
$ToolchainArgs = @("-DCMAKE_C_COMPILER=$Toolchain", "-DCMAKE_ASM_COMPILER=$Toolchain")
$Objcopy = Join-Path (Split-Path -Parent $Toolchain) "arm-none-eabi-objcopy.exe"
if (-not (Test-Path -LiteralPath $Objcopy)) { throw "arm-none-eabi-objcopy not found: $Objcopy" }

New-Item -ItemType Directory -Force -Path $BuildRoot, $ArtifactRoot | Out-Null

Write-Host "[1/5] Configure and build App"
cmake -S $App -B $AppBuild -G $Generator "-DCMAKE_BUILD_TYPE=$Configuration" @ToolchainArgs
if ($LASTEXITCODE -ne 0) { throw "App CMake configure failed." }
cmake --build $AppBuild --parallel 8
if ($LASTEXITCODE -ne 0) { throw "App build failed." }

Write-Host "[2/5] Configure and build Bootloader"
cmake -S $Boot -B $BootBuild -G $Generator "-DCMAKE_BUILD_TYPE=$Configuration" @ToolchainArgs
if ($LASTEXITCODE -ne 0) { throw "Bootloader CMake configure failed." }
cmake --build $BootBuild --parallel 8
if ($LASTEXITCODE -ne 0) { throw "Bootloader build failed." }

Write-Host "[3/5] Run host-side protocol checks"
& $Python (Join-Path $Root "tools\test_protocol.py")
if ($LASTEXITCODE -ne 0) { throw "Protocol test failed." }
& $Python (Join-Path $Root "tools\test_lora_profiles.py")
if ($LASTEXITCODE -ne 0) { throw "LoRa profile test failed." }
$HostGcc = Get-Command gcc -ErrorAction SilentlyContinue
if ($HostGcc) {
    $LogTest = Join-Path $BuildRoot "imu_log_test.exe"
    & $HostGcc.Source -std=c11 -O0 -Wall -Wextra -Werror `
        '-DIMU_LOG_REGION_SIZE=0x3000UL' `
        "-I$(Join-Path $Root 'tools\test_stubs')" "-I$(Join-Path $App 'component')" `
        (Join-Path $App 'component\imu_log.c') `
        (Join-Path $App 'component\lora_protocol.c') `
        (Join-Path $Root 'tools\test_imu_log.c') -o $LogTest
    if ($LASTEXITCODE -ne 0) { throw "IMU log host test build failed." }
    & $LogTest
    if ($LASTEXITCODE -ne 0) { throw "IMU log host test failed." }
}

Write-Host "[4/5] Export Boot, App and full firmware images"
& $Python (Join-Path $Root "tools\guided_dart_tool.py") make-image `
    --boot (Join-Path $BootBuild "Guided_dart_bootloader.bin") `
    --app (Join-Path $AppBuild "Guided_dart_2027.bin") `
    --output $FullBin `
    --version $Version `
    --full-elf $FullElf `
    --objcopy $Objcopy
if ($LASTEXITCODE -ne 0) { throw "Full image creation failed." }
& $Python (Join-Path $Root "tools\guided_dart_tool.py") make-app-swd $FullBin $AppSwdBin
if ($LASTEXITCODE -ne 0) { throw "App-SWD image creation failed." }
Copy-Item -LiteralPath (Join-Path $BootBuild "Guided_dart_bootloader.bin") -Destination (Join-Path $ArtifactRoot "Guided_dart_bootloader.bin") -Force
Copy-Item -LiteralPath (Join-Path $BootBuild "Guided_dart_bootloader.elf") -Destination (Join-Path $ArtifactRoot "Guided_dart_bootloader.elf") -Force
Copy-Item -LiteralPath (Join-Path $AppBuild "Guided_dart_2027.bin") -Destination (Join-Path $ArtifactRoot "Guided_dart_app.bin") -Force
Copy-Item -LiteralPath (Join-Path $AppBuild "Guided_dart_2027.elf") -Destination (Join-Path $ArtifactRoot "Guided_dart_app.elf") -Force
# 合并 ELF 只有整片 Flash 的装载段，不包含 Boot/App 的源码调试符号。
if (-not (Test-Path -LiteralPath $FullElf)) { throw "Full ELF was not created: $FullElf" }

Write-Host "[5/5] Verify firmware images"
& $Python (Join-Path $Root "tools\guided_dart_tool.py") verify-image $FullBin
if ($LASTEXITCODE -ne 0) { throw "Image verification failed." }
& $Python (Join-Path $Root "tools\guided_dart_tool.py") verify-app-swd $AppSwdBin
if ($LASTEXITCODE -ne 0) { throw "App-SWD image verification failed." }
Write-Host "Build completed: $ArtifactRoot"
