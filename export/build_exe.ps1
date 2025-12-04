param(
    [switch] $Force
)

$ErrorActionPreference = "Stop"

function Get-PythonCmd {
    $py = Get-Command py -ErrorAction SilentlyContinue
    if ($py) { return @("py", "-3") }
    $python = Get-Command python -ErrorAction SilentlyContinue
    if ($python) { return @("python") }
    throw "Python 3.8+ not found. Install from https://www.python.org/downloads/ then re-run."
}

$root = Split-Path -Parent $PSScriptRoot  # project root
$venvPath = Join-Path $root ".venv"
$pythonCmd = Get-PythonCmd

# Create venv if missing or if -Force is passed
if ($Force -or -not (Test-Path $venvPath)) {
    Write-Host "Creating venv at $venvPath" -ForegroundColor Cyan
    & $pythonCmd "-m" "venv" $venvPath
}

$venvPy = Join-Path $venvPath "Scripts/python.exe"
if (-not (Test-Path $venvPy)) { throw "Virtual env python not found: $venvPy" }

Write-Host "Upgrading pip and installing requirements..." -ForegroundColor Cyan
& $venvPy "-m" "pip" "install" "--upgrade" "pip"
& $venvPy "-m" "pip" "install" "-r" (Join-Path $root "requirements.txt") "pyinstaller"

Write-Host "Building executable with PyInstaller..." -ForegroundColor Cyan
Push-Location $root
& $venvPy "-m" "PyInstaller" "atticus_terminal.spec" "--noconfirm"
Pop-Location

$exePath = Join-Path $root "dist/AtticusTerminal/AtticusTerminal.exe"
Write-Host "Build complete!" -ForegroundColor Green
Write-Host "You can run the EXE at:`n  $exePath" -ForegroundColor Green