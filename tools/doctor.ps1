param(
    [string]$Port = "",
    [string]$Toolchain = ""
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$App = $Root
$Ioc = Join-Path $App "Guided_Dart.ioc"

if ([string]::IsNullOrWhiteSpace($Toolchain)) {
    $Toolchain = "D:\JetBrains\CLion 2023.3.5\arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi\bin\arm-none-eabi-gcc.exe"
}

Write-Host "=== Guided Dart 工程检查 ==="
Write-Host ("工程根目录: {0}" -f $Root)
Write-Host ("App 联接:   {0} [{1}]" -f $App, (Get-Item $App).LinkType)
Write-Host ("工具链:     {0} [{1}]" -f $Toolchain, (Test-Path $Toolchain))
Write-Host ("CubeMX IOC: {0} [{1}]" -f $Ioc, (Test-Path $Ioc))

if ($Port -and (Get-Command Get-PnpDevice -ErrorAction SilentlyContinue)) {
    $device = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
        Where-Object { $_.FriendlyName -match "\($Port\)" } | Select-Object -First 1
    if ($device) {
        Write-Host ("串口:       {0} ({1})" -f $Port, $device.FriendlyName)
    } else { Write-Warning ("未发现 {0}，请先连接 USB-LoRa/USB-TTL。" -f $Port) }
} elseif (-not $Port) {
    Write-Host "串口:       未指定（上位机启动后可从端口列表选择）"
}

if (Test-Path $Ioc) {
    $iocText = Get-Content $Ioc -Raw
    foreach ($signal in @("PA9.Signal=USART1_TX", "PA10.Signal=USART1_RX", "PB12.Signal=GPIO_Output", "PB13.Signal=SPI2_SCK", "PB14.Signal=SPI2_MISO", "PB15.Signal=SPI2_MOSI")) {
        if ($iocText.Contains($signal)) { Write-Host "IOC 已确认: $signal" }
        else { Write-Warning "IOC 缺少: $signal" }
    }
}

Write-Host "串口约定: 主控 USART1=115200 8N1；电脑端透明模式按模块保存参数设置。"
Write-Host "下一步: LoRa 配置模式使用 tools\guided_dart_tool.py；完整构建使用 build_all.ps1。"
