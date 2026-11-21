<#
    tools/build_gui.ps1 — build and run the Qt GUI (spec §16).

    Usage:
        pwsh tools/build_gui.ps1              # configure, build, report the path
        pwsh tools/build_gui.ps1 -Run         # ...then launch the shell
        pwsh tools/build_gui.ps1 -Run P3      # ...and open one program directly
        pwsh tools/build_gui.ps1 -Selftest    # headless: print every result
        pwsh tools/build_gui.ps1 -Deploy out  # standalone folder via windeployqt

    Unlike rcvd-cli, the GUI cannot be built with a bare `cl` invocation: its
    Q_OBJECT widgets need moc, so CMake drives it. Qt is located automatically
    under C:\Qt; pass -QtPrefix to override.

    §4.2 flags are unchanged — /W4 /WX /fp:precise, never /fp:fast. The GUI is
    built at BOTH /Od and /O2 by run_tests.ps1 because those are different static
    analyses; this script builds one configuration, RelWithDebInfo by default.
#>
[CmdletBinding()]
param(
    [string]$QtPrefix = '',
    [string]$Config = 'RelWithDebInfo',
    [string]$BuildDir = '',
    [switch]$Run,
    [string]$Program = '',
    [switch]$Selftest,
    [string]$Deploy = ''
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot

function Find-CMake {
    if (Get-Command cmake -ErrorAction SilentlyContinue) { return 'cmake' }
    $p = 'C:\Program Files\CMake\bin\cmake.exe'
    if (Test-Path $p) { return $p }
    throw "CMake not found. Install it with: winget install Kitware.CMake"
}

function Find-Qt {
    if ($QtPrefix) {
        if (-not (Test-Path (Join-Path $QtPrefix 'lib\cmake\Qt6'))) {
            throw "No Qt6 CMake package under $QtPrefix"
        }
        return $QtPrefix
    }
    # Newest version first, so a machine with several Qt installs picks the latest.
    $candidates = Get-ChildItem 'C:\Qt' -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -match '^\d+\.\d+' } |
        Sort-Object { [version]$_.Name } -Descending
    foreach ($v in $candidates) {
        foreach ($kit in Get-ChildItem $v.FullName -Directory -ErrorAction SilentlyContinue) {
            if (Test-Path (Join-Path $kit.FullName 'lib\cmake\Qt6')) { return $kit.FullName }
        }
    }
    throw @"
Qt 6 not found under C:\Qt.

Install just what the GUI needs (about 260 MB, no account required):
    pip install aqtinstall
    python -m aqt install-qt windows desktop 6.8.3 win64_msvc2022_64 --archives qtbase --outputdir C:\Qt

Then re-run this script, or pass -QtPrefix <path-to>\msvc2022_64.
"@
}

$cmake = Find-CMake
$qt = Find-Qt
if (-not $BuildDir) { $BuildDir = Join-Path $repo 'build-gui' }

Write-Host "==> Qt      $qt" -ForegroundColor Cyan
Write-Host "==> build   $BuildDir ($Config)" -ForegroundColor Cyan

& $cmake -S $repo -B $BuildDir -DCMAKE_PREFIX_PATH="$qt" -DRCVD_BUILD_TESTS=OFF
if ($LASTEXITCODE -ne 0) { throw 'configure failed' }
& $cmake --build $BuildDir --config $Config --target rcvd-gui
if ($LASTEXITCODE -ne 0) { throw 'build failed' }

$exe = Join-Path $BuildDir "$Config\rcvd-gui.exe"
if (-not (Test-Path $exe)) { $exe = Join-Path $BuildDir 'rcvd-gui.exe' }
if (-not (Test-Path $exe)) { throw "built, but rcvd-gui.exe is not where expected" }
Write-Host "built $exe" -ForegroundColor Green

# The Qt DLLs are not beside the exe in a plain build tree.
$env:Path = (Join-Path $qt 'bin') + ';' + $env:Path

if ($Deploy) {
    $out = (New-Item -ItemType Directory -Path $Deploy -Force).FullName
    Copy-Item $exe $out -Force
    & (Join-Path $qt 'bin\windeployqt.exe') --no-translations (Join-Path $out 'rcvd-gui.exe')
    Write-Host "standalone build in $out" -ForegroundColor Green
}

if ($Selftest) {
    # rcvd-gui is a WIN32-subsystem binary, so it has no console of its own;
    # stdout only arrives somewhere if the parent redirects it to a handle.
    $tmp = [System.IO.Path]::GetTempFileName()
    cmd /c "`"$exe`" --selftest > `"$tmp`" 2>&1"
    Get-Content $tmp
    Remove-Item $tmp -ErrorAction SilentlyContinue
    exit 0
}

if ($Run) {
    if ($Program) { & $exe $Program } else { & $exe }
}
exit 0
