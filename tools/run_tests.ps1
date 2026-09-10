<#
    tools/run_tests.ps1 — run everything that can be verified without CMake.

    Builds and runs:
      1. tools/verify_standalone.cpp  — the dependency-free acceptance driver
      2. tools/check_compile_fail.py  — the Rule U6 (§3.1) compile-failure gate

    Neither needs CMake, Catch2, Qt or Eigen. A C++20 compiler is the only
    requirement; Python is needed for the second gate only.

    Usage:
        pwsh tools/run_tests.ps1
        pwsh tools/run_tests.ps1 -KeepBinary     # leave verify.exe in place

    The authoritative suite is tests/ under Catch2 v3 (spec §4.2) and needs
    CMake — see README. This script covers what a bare toolchain can check.
#>
[CmdletBinding()]
param(
    [switch]$KeepBinary
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$out  = Join-Path ([System.IO.Path]::GetTempPath()) "rcvd_tests_$PID"
New-Item -ItemType Directory -Path $out -Force | Out-Null

function Find-Compiler {
    # Already in a developer shell?
    if (Get-Command cl -ErrorAction SilentlyContinue) { return $null }
    # Otherwise locate vcvars64 via vswhere.
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
$exe = Join-Path $out 'verify.exe'

# NOTE the two include paths. tools/verify_standalone.cpp pulls in the shared
# fixtures under tests/support, so `-I include` alone does not compile.
$flags = '/nologo /std:c++20 /EHsc /W4 /WX /permissive- /fp:precise'
$cmd = "cl $flags /I `"$repo\include`" /I `"$repo\tests`" /Fe:`"$exe`" /Fo`"$out\\`" `"$repo\tools\verify_standalone.cpp`""

Write-Host "==> building the standalone acceptance driver" -ForegroundColor Cyan
if ($vcvars) {
    cmd /c "`"$vcvars`" >nul 2>&1 && $cmd"
} else {
    cmd /c $cmd
}
if (-not (Test-Path $exe)) { throw "build failed" }

Write-Host "==> running it" -ForegroundColor Cyan
& $exe
$driverOk = ($LASTEXITCODE -eq 0)

$u6Ok = $true
if (Get-Command python -ErrorAction SilentlyContinue) {
    Write-Host "`n==> Rule U6 compile-failure gate (spec 3.1)" -ForegroundColor Cyan
    $u6 = "python `"$repo\tools\check_compile_fail.py`" --compiler cl --include `"$repo\include`" --dir `"$repo\tests\compile_fail`""
    if ($vcvars) { cmd /c "`"$vcvars`" >nul 2>&1 && cd /d `"$repo`" && $u6" }
    else { cmd /c "cd /d `"$repo`" && $u6" }
    $u6Ok = ($LASTEXITCODE -eq 0)
} else {
    Write-Host "`n==> skipping the Rule U6 gate (python not found)" -ForegroundColor Yellow
}

if ($KeepBinary) {
    Write-Host "`nbinary left at $exe"
} else {
    Remove-Item -Recurse -Force $out -ErrorAction SilentlyContinue
}

Write-Host ""
if ($driverOk -and $u6Ok) {
    Write-Host "ALL GREEN" -ForegroundColor Green
    exit 0
}
Write-Host "FAILURES ABOVE" -ForegroundColor Red
exit 1
