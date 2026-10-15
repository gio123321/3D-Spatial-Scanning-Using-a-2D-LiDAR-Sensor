$ErrorActionPreference = "Stop"
$projectRoot = $PSScriptRoot
$python = Join-Path $projectRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python)) {
    throw "Run setup.ps1 before building the desktop application."
}

Push-Location $projectRoot
try {
    & $python -m PyInstaller --noconfirm --distpath (Join-Path $projectRoot "dist") --workpath (Join-Path $projectRoot "build") (Join-Path $projectRoot "build_assets\UnfoLDAR.spec")
    if ($LASTEXITCODE -ne 0) { throw "Desktop build failed with exit code $LASTEXITCODE" }
} finally {
    Pop-Location
}
