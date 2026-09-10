<#
    tools/build_cli.ps1 — build rcvd-cli without CMake.

    Usage:
        pwsh tools/build_cli.ps1                 # builds .\rcvd-cli.exe
        pwsh tools/build_cli.ps1 -Output C:\bin  # elsewhere
        pwsh tools/build_cli.ps1 -Run            # build, then print the help

    rcvd-cli is header-only apart from app/rcvd_cli.cpp, so this is a single
    translation unit and needs no linker configuration. Spec §4.2 flags are used
    verbatim, including /fp:precise — never /fp:fast, which Rule A2 forbids.
#>
[CmdletBinding()]
param(
    [string]$Output = '.',
    [switch]$Run
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot

function Find-Compiler {
    if (Get-Command cl -ErrorAction SilentlyContinue) { return $null }
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $root = & $vswhere -latest -products * -property installationPath 2>$null | Select-Object -First 1
        if ($root) {
            $vcvars = Join-Path $root 'VC\Auxiliary\Build\vcvars64.bat'
            if (Test-Path $vcvars) { return $vcvars }
        }
    }
    foreach ($ed in 'Community', 'Professional', 'Enterprise', 'BuildTools') {
        $p = "C:\Program Files\Microsoft Visual Studio\2022\$ed\VC\Auxiliary\Build\vcvars64.bat"
        if (Test-Path $p) { return $p }
    }
    throw "No C++ compiler found. Open a 'Developer PowerShell for VS', or install the Visual Studio C++ workload."
}

$vcvars = Find-Compiler
$outDir = (Resolve-Path $Output).Path
$exe = Join-Path $outDir 'rcvd-cli.exe'
$objDir = Join-Path ([System.IO.Path]::GetTempPath()) "rcvd_cli_$PID"
New-Item -ItemType Directory -Path $objDir -Force | Out-Null

$flags = '/nologo /std:c++20 /EHsc /W4 /WX /permissive- /fp:precise'
$cmd = "cl $flags /I `"$repo\include`" /Fe:`"$exe`" /Fo`"$objDir\\`" `"$repo\app\rcvd_cli.cpp`""

Write-Host "==> building rcvd-cli" -ForegroundColor Cyan
if ($vcvars) { cmd /c "`"$vcvars`" >nul 2>&1 && $cmd" } else { cmd /c $cmd }
Remove-Item -Recurse -Force $objDir -ErrorAction SilentlyContinue

if (-not (Test-Path $exe)) { Write-Host "build failed" -ForegroundColor Red; exit 1 }
Write-Host "built $exe" -ForegroundColor Green

if ($Run) {
    Write-Host ""
    & $exe --help
}
exit 0
