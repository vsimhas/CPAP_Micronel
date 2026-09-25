param(
    [string]$BuildDir = (Join-Path ([System.IO.Path]::GetTempPath()) 'cpap-maskoff-host-tests')
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) { throw 'Visual Studio C++ build tools are required.' }
$vsRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsRoot) { throw 'No Visual Studio C++ toolchain found.' }
$vcvars = Join-Path $vsRoot 'VC/Auxiliary/Build/vcvars64.bat'
$null = New-Item -ItemType Directory -Force -Path $BuildDir
$BuildDir = (Resolve-Path -LiteralPath $BuildDir).Path
Push-Location $repoRoot
try {
    $cases = @(
        @{ Name = 'test_mask_off'; Sources = 'tests/host/test_mask_off.c' },
        @{ Name = 'test_flow_quality'; Sources = 'tests/host/test_flow_quality.c CM7/Core/Src/flow_sensor.c CM7/Core/Src/leak_btps.c' },
        @{ Name = 'test_tube_comp'; Sources = 'tests/host/test_tube_comp.c CM7/Core/Src/tube_comp.c' }
    )
    foreach ($case in $cases) {
        $exe = Join-Path $BuildDir ($case.Name + '.exe')
        $command = 'call "{0}" >nul && cl /nologo /std:c11 /W3 /O1 /Gy /utf-8 /DCORE_CM7 /DDBG_ENABLED=0 /DTELEM_ENABLED=0 /Itests/host/include /ICM7/Core/Inc /ICommon/Inc {1} /Fo"{2}\\" /Fe"{3}" /link /OPT:REF' -f $vcvars, $case.Sources, $BuildDir, $exe
        & $env:ComSpec /d /c $command
        if ($LASTEXITCODE -ne 0) { throw "Build failed: $($case.Name)" }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw "Tests failed: $($case.Name)" }
    }
}
finally { Pop-Location }
