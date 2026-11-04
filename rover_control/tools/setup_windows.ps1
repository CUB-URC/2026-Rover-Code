# One-shot Windows setup for rover_control.
#
# Run from the rover_control folder (double-click setup_windows.bat, or):
#   powershell -ExecutionPolicy Bypass -File tools\setup_windows.ps1
#
# What it does (each step is skipped if already done):
#   1. Installs Python 3.13 (via winget) if no usable Python is found
#   2. Installs Visual Studio Build Tools with the C++ workload (via winget) if missing
#   3. Installs the VS Code extensions this project recommends (if VS Code is installed)
#   4. Creates .venv (recreates it if it is broken, e.g. copied from another PC)
#   5. Builds the C++ core + Python bindings and installs the gamepad/serial extras
#   6. Runs the C++ and Python tests
#
# Network settings (static IP, firewall) are a separate, optional step:
#   tools\network_setup.bat   (asks for administrator rights)
#
# Optional: install the C++ Build Tools on another drive when C: is short on space:
#   powershell -ExecutionPolicy Bypass -File tools\setup_windows.ps1 -BuildToolsDrive D

param([string]$BuildToolsDrive = "")

# Plain ASCII on purpose: Windows PowerShell 5.1 misreads non-ASCII in scripts.

# "Continue", not "Stop": in Windows PowerShell 5.1, "Stop" turns harmless stderr output
# from native programs (pip, winget) into fatal errors. Each step checks $LASTEXITCODE instead.
$ErrorActionPreference = "Continue"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

function Step($msg) { Write-Host ""; Write-Host "==> $msg" -ForegroundColor Cyan }
function Ok($msg)   { Write-Host "    OK: $msg" -ForegroundColor Green }
function Warn($msg) { Write-Host "    WARNING: $msg" -ForegroundColor Yellow }
function Fail($msg) { Write-Host ""; Write-Host "SETUP STOPPED: $msg" -ForegroundColor Red; exit 1 }

function Refresh-Path {
    # Pick up PATH changes made by installers without reopening the terminal.
    $machine = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $user = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$machine;$user"
}

function Has-Winget { return [bool](Get-Command winget -ErrorAction SilentlyContinue) }

function Find-Python {
    # Returns a command that runs a real Python 3.9+, or $null.
    # (Skips the Microsoft Store "python.exe" stub, which only opens the Store.)
    $candidates = @(
        @("py", "-3"),
        @("python")
    )
    foreach ($c in $candidates) {
        $exe = $c[0]
        if (-not (Get-Command $exe -ErrorAction SilentlyContinue)) { continue }
        $cmdArgs = @()
        if ($c.Count -gt 1) { $cmdArgs = $c[1..($c.Count - 1)] }
        try {
            $ver = & $exe @cmdArgs -c "import sys; print('%d.%d' % sys.version_info[:2])" 2>$null
            if ($LASTEXITCODE -eq 0 -and $ver) {
                $parts = $ver.Trim().Split(".")
                if ([int]$parts[0] -eq 3 -and [int]$parts[1] -ge 9) { return ,$c }
            }
        } catch { }
    }
    return $null
}

function Has-CppBuildTools {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { return $false }
    try {
        $path = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        return [bool]$path
    } catch { return $false }
}

Write-Host "rover_control Windows setup" -ForegroundColor White
Write-Host "Project folder: $Root"
if ($Root -match "OneDrive") {
    Warn "This folder is inside OneDrive. Builds can hit file-lock errors and .venv will sync between PCs. A folder like C:\dev\rover_control is safer."
}

# ---------------------------------------------------------------- 1. Python
Step "Checking for Python 3.9+"
$py = Find-Python
if (-not $py) {
    if (-not (Has-Winget)) { Fail "Python not found and winget is unavailable. Install Python 3.13 from https://www.python.org/downloads/ (tick 'Add python.exe to PATH'), then run this again." }
    Write-Host "    Installing Python 3.13 with winget (a few minutes)..."
    winget install -e --id Python.Python.3.13 --scope user --accept-package-agreements --accept-source-agreements
    Refresh-Path
    $py = Find-Python
    if (-not $py) { Fail "Python was installed but is not on PATH yet. Close this window, open a new terminal, and run setup again." }
}
$pyExe = $py[0]
$pyArgs = @()
if ($py.Count -gt 1) { $pyArgs = $py[1..($py.Count - 1)] }
$pyVer = & $pyExe @pyArgs --version
Ok "$pyVer ($($py -join ' '))"

# ---------------------------------------------------------------- 2. C++ compiler
Step "Checking for the Microsoft C++ compiler (Visual Studio Build Tools)"
if (Has-CppBuildTools) {
    Ok "C++ build tools found"
} else {
    if (-not (Has-Winget)) { Fail "C++ build tools not found and winget is unavailable. Install 'Build Tools for Visual Studio' from https://visualstudio.microsoft.com/downloads/ with 'Desktop development with C++' ticked, then run this again." }
    Write-Host "    Installing Build Tools with the C++ workload (several GB, 10-30 minutes)."
    Write-Host "    Windows will ask for administrator permission - click Yes."
    $vsArgs = "--passive --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
    if ($BuildToolsDrive) {
        $d = $BuildToolsDrive.TrimEnd(":", "\")
        $vsArgs += " --installPath $d`:\BuildTools --path cache=$d`:\VSCache --path shared=$d`:\VSShared"
        Write-Host "    Installing to $d`: (some Windows SDK files still go on C:)"
    }
    $free = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
    Write-Host "    Free space on C: $free GB"
    winget install -e --id Microsoft.VisualStudio.2022.BuildTools --accept-package-agreements --accept-source-agreements --override $vsArgs
    $code = $LASTEXITCODE
    if (-not (Has-CppBuildTools)) {
        if ($code -eq -2147024784 -or $code -eq 2147942512) {
            Fail "Not enough disk space for the Build Tools (error 0x80070070). Free up space on C: (about 8 GB), or re-run with:  powershell -ExecutionPolicy Bypass -File tools\setup_windows.ps1 -BuildToolsDrive D"
        }
        Fail "Build Tools did not finish installing (exit code $code). Re-run this script, or install them manually (see the message above)."
    }
    Ok "C++ build tools installed"
}

# ---------------------------------------------------------------- 3. VS Code extensions
Step "Installing recommended VS Code extensions"
Refresh-Path
if (Get-Command code -ErrorAction SilentlyContinue) {
    foreach ($ext in @("ms-python.python", "ms-vscode.cpptools", "ms-vscode.cmake-tools")) {
        & code --install-extension $ext --force | Out-Null
        Ok $ext
    }
} else {
    Warn "VS Code's 'code' command was not found. Open this folder in VS Code and accept the recommended-extensions prompt instead."
}

# ---------------------------------------------------------------- 4. venv
Step "Setting up the .venv virtual environment"
$venvPy = Join-Path $Root ".venv\Scripts\python.exe"
$venvOk = $false
if (Test-Path $venvPy) {
    try { & $venvPy -c "import sys" 2>$null; $venvOk = ($LASTEXITCODE -eq 0) } catch { $venvOk = $false }
    if (-not $venvOk) {
        Warn "Existing .venv is broken (probably copied from another PC). Recreating it."
        Remove-Item -Recurse -Force (Join-Path $Root ".venv")
    }
}
if (-not $venvOk) {
    & $pyExe @pyArgs -m venv .venv
    if ($LASTEXITCODE -ne 0) { Fail "Could not create .venv" }
}
Ok ".venv ready"

# ---------------------------------------------------------------- 5. build + install
Step "Building the C++ core and Python bindings (1-3 minutes the first time)"
& $venvPy -m pip install --upgrade pip --quiet
& $venvPy -m pip install -e ".[test,gamepad,serial]"
if ($LASTEXITCODE -ne 0) { Fail "Build failed. Scroll up for the first line containing 'error' and send it to whoever is helping you." }
Ok "rover_control installed"

# ---------------------------------------------------------------- 6. tests
Step "Running C++ tests"
& $venvPy tools\run_cpp_tests.py
if ($LASTEXITCODE -ne 0) { Fail "C++ tests failed" }
Step "Running Python tests"
& $venvPy -m pytest -q tests
if ($LASTEXITCODE -ne 0) { Fail "Python tests failed" }

Write-Host ""
Write-Host "ALL SET." -ForegroundColor Green
Write-Host "In each new terminal, activate the environment first:  .venv\Scripts\activate"
Write-Host "Rover side:          rover-node"
Write-Host "Base station side:   rover-base            (gamepad)"
Write-Host "Check the gamepad:   rover-base --print-axes"
Write-Host "Two-PC test over a cable: run tools\network_setup.bat on each PC first."
