param(
    [switch] $Force
)

$ErrorActionPreference = "Stop"

function Get-PythonCmd {
    $candidates = @(
        @("py", "-3"),
        @("py"),
        @("python"),
        @("python3")
    )
    foreach ($cand in $candidates) {
        $cmd = Get-Command $cand[0] -ErrorAction SilentlyContinue
        if ($cmd) {
            return ,@($cmd.Path) + $cand[1..($cand.Length - 1)]
        }
    }
    throw "Python 3.8+ not found. Install from https://www.python.org/downloads/ then re-run."
}

$root = Split-Path -Parent $PSScriptRoot  # project root
$venvPath = Join-Path $root ".venv"
$pythonCmd = Get-PythonCmd
$pyBin = $pythonCmd[0]
$pyArgs = @()
if ($pythonCmd.Length -gt 1) { $pyArgs = $pythonCmd[1..($pythonCmd.Length - 1)] }

# Create venv if missing or if -Force is passed
if ($Force -or -not (Test-Path $venvPath)) {
    Write-Host "Creating venv at $venvPath" -ForegroundColor Cyan
    & $pyBin @pyArgs "-m" "venv" $venvPath
}

$venvPy = Join-Path $venvPath "Scripts/python.exe"
if (-not (Test-Path $venvPy)) { throw "Virtual env python not found: $venvPy" }

Write-Host "Upgrading pip and installing requirements..." -ForegroundColor Cyan
& $venvPy "-m" "pip" "install" "--upgrade" "pip"
# Install app deps + PyInstaller and Pillow (PNG -> ICO conversion)
& $venvPy "-m" "pip" "install" "-r" (Join-Path $root "requirements.txt") "pyinstaller" "pillow"

# Clean old build/dist artifacts so we only get one fresh output
$distPath = Join-Path $root "dist"
$buildPath = Join-Path $root "build"
if (Test-Path $distPath) { Remove-Item $distPath -Recurse -Force -ErrorAction SilentlyContinue }
if (Test-Path $buildPath) { Remove-Item $buildPath -Recurse -Force -ErrorAction SilentlyContinue }

Write-Host "Building executable with PyInstaller..." -ForegroundColor Cyan
Push-Location $root
& $venvPy "-m" "PyInstaller" "atticus_terminal.spec" "--noconfirm"
Pop-Location

$builtExe = Join-Path $root "dist/ATTICUS_TERMINAL.exe"
$finalExe = Join-Path $root "ATTICUS_TERMINAL.exe"

if (Test-Path $builtExe) {
    # Move the single exe to the project root
    if (Test-Path $finalExe) { Remove-Item $finalExe -Force -ErrorAction SilentlyContinue }
    Move-Item $builtExe $finalExe -Force
    # Remove the now-empty dist folder
    if (Test-Path $distPath) { Remove-Item $distPath -Recurse -Force -ErrorAction SilentlyContinue }
    Write-Host "Build complete! EXE is at:`n  $finalExe" -ForegroundColor Green
    Write-Host "Launching ATTICUS_TERMINAL..." -ForegroundColor Cyan
    Start-Process $finalExe
} else {
    Write-Warning "Expected EXE not found at $builtExe"
}
