$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$venvPath = Join-Path $projectRoot ".venv"

py -3.12 -m venv $venvPath
if ($LASTEXITCODE -ne 0) { throw "Python environment creation failed" }
& (Join-Path $venvPath "Scripts\python.exe") -m pip install -r (Join-Path $projectRoot "requirements-windows-lock.txt")
if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed" }
& (Join-Path $venvPath "Scripts\python.exe") -m pip install --no-build-isolation --no-deps -e $projectRoot
if ($LASTEXITCODE -ne 0) { throw "Application installation failed" }
Write-Host "Installed. Run: $venvPath\Scripts\python.exe -m unfoldar"
