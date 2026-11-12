<#
.SYNOPSIS
    Build the firmware and flash it to the NUCLEO-G474RE over its on-board ST-LINK.

.DESCRIPTION
    Uses the compiler, CMake, Ninja and STM32CubeProgrammer that the "STM32CubeIDE for VS Code"
    extension installs under %LOCALAPPDATA%\stm32cube\bundles, so nothing has to be on PATH.

.EXAMPLE
    .\tools\build_flash.ps1                  # Release build, then flash
    .\tools\build_flash.ps1 -Config Debug    # Debug build, then flash
    .\tools\build_flash.ps1 -NoFlash         # build only
#>
param(
    [ValidateSet('Release', 'Debug')]
    [string]$Config = 'Release',
    [switch]$NoFlash
)

$ErrorActionPreference = 'Stop'
$root    = Split-Path -Parent $PSScriptRoot
$bundles = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'

# Newest installed version of a bundle, then the given executable inside it.
function Find-BundleTool([string]$bundle, [string]$exe) {
    $dir = Get-ChildItem (Join-Path $bundles $bundle) -Directory -ErrorAction SilentlyContinue |
           Sort-Object Name -Descending | Select-Object -First 1
    if (-not $dir) {
        throw "Bundle '$bundle' not found in $bundles. Install the STM32CubeIDE for VS Code extension."
    }
    $file = Get-ChildItem $dir.FullName -Recurse -Filter $exe -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $file) { throw "'$exe' not found in $($dir.FullName)." }
    return $file.FullName
}

$gcc   = Find-BundleTool 'gnu-tools-for-stm32' 'arm-none-eabi-gcc.exe'
$cmake = Find-BundleTool 'cmake'               'cmake.exe'
$ninja = Find-BundleTool 'ninja'               'ninja.exe'

# The CMake toolchain file expects arm-none-eabi-* and ninja on PATH (this session only).
$env:PATH = "$(Split-Path $gcc);$(Split-Path $ninja);$env:PATH"

Push-Location $root
try {
    Write-Host "== Configure ($Config)" -ForegroundColor Cyan
    & $cmake --preset $Config | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed." }

    Write-Host "== Build ($Config)" -ForegroundColor Cyan
    & $cmake --build --preset $Config
    if ($LASTEXITCODE -ne 0) { throw "Build failed." }

    $elf = Join-Path $root "build\$Config\M3508_Lib.elf"
    if (-not (Test-Path $elf)) { throw "Build finished but $elf is missing." }
    Write-Host "Built $elf" -ForegroundColor Green

    if ($NoFlash) { return }

    $prog = Find-BundleTool 'programmer' 'STM32_Programmer_CLI.exe'
    Write-Host "== Flash" -ForegroundColor Cyan
    # -w: write the .elf (addresses come from the file)  -v: verify  -rst: reset and run
    & $prog -c port=SWD -w $elf -v -rst
    if ($LASTEXITCODE -ne 0) { throw "Flashing failed. Is the board plugged in, and is no debug session open?" }
    Write-Host "Flashed and running." -ForegroundColor Green
}
finally {
    Pop-Location
}
