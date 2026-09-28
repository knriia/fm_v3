[CmdletBinding()]
param(
    [string]$BuildDirectory
)

$ErrorActionPreference = 'Stop'

$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if ([string]::IsNullOrWhiteSpace($BuildDirectory)) {
    $buildRoot = [System.IO.Path]::GetFullPath((Join-Path $projectRoot 'cmake-build-coverage'))
} elseif ([System.IO.Path]::IsPathRooted($BuildDirectory)) {
    $buildRoot = [System.IO.Path]::GetFullPath($BuildDirectory)
} else {
    $buildRoot = [System.IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
}

$cmake = (Get-Command cmake -ErrorAction Stop).Source
$ctest = (Get-Command ctest -ErrorAction Stop).Source
$null = Get-Command ninja -ErrorAction Stop
$clang = (Get-Command clang -ErrorAction Stop).Source
$llvmProfdata = (Get-Command llvm-profdata -ErrorAction Stop).Source
$llvmCov = (Get-Command llvm-cov -ErrorAction Stop).Source

$configureArguments = @(
    '-S', $projectRoot,
    '-B', $buildRoot,
    '-G', 'Ninja',
    '-DBUILD_TESTING=ON',
    '-DFM_V3_ENABLE_LLVM_COVERAGE=ON',
    '-DCMAKE_BUILD_TYPE=Debug',
    "-DCMAKE_C_COMPILER=$clang"
)

& $cmake @configureArguments
if ($LASTEXITCODE -ne 0) {
    throw "CMake configuration failed with exit code $LASTEXITCODE"
}

& $cmake --build $buildRoot
if ($LASTEXITCODE -ne 0) {
    throw "Host-test build failed with exit code $LASTEXITCODE"
}

$runId = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$runDirectory = Join-Path $buildRoot "coverage-$runId"
$profileDirectory = Join-Path $runDirectory 'profiles'
$htmlDirectory = Join-Path $runDirectory 'html'
New-Item -ItemType Directory -Path $profileDirectory -Force | Out-Null

$previousProfileFile = $env:LLVM_PROFILE_FILE
$env:LLVM_PROFILE_FILE = Join-Path $profileDirectory '%p.profraw'
try {
    & $ctest --test-dir $buildRoot --output-on-failure
    $testExitCode = $LASTEXITCODE
} finally {
    if ($null -eq $previousProfileFile) {
        Remove-Item Env:LLVM_PROFILE_FILE -ErrorAction SilentlyContinue
    } else {
        $env:LLVM_PROFILE_FILE = $previousProfileFile
    }
}

$profileFiles = @(
    Get-ChildItem -LiteralPath $profileDirectory -Filter '*.profraw' -File |
        Select-Object -ExpandProperty FullName
)
if ($profileFiles.Count -eq 0) {
    throw 'No LLVM profiles were produced. Check that the instrumented host tests ran.'
}

$profileData = Join-Path $runDirectory 'coverage.profdata'
$mergeArguments = @('merge', '-sparse') + $profileFiles + @('-o', $profileData)
& $llvmProfdata @mergeArguments
if ($LASTEXITCODE -ne 0) {
    throw "llvm-profdata failed with exit code $LASTEXITCODE"
}

$testTargetNames = @(
    'diagnostic_protocol_tests',
    'diagnostic_task_unit_tests',
    'startup_task_unit_tests',
    'telemetry_protocol_tests',
    'telemetry_task_unit_tests',
    'command_protocol_tests',
    'command_task_unit_tests',
    'command_decoder_task_unit_tests'
)
$testDirectory = Join-Path $buildRoot 'tests'
$testBinaries = @(
    foreach ($targetName in $testTargetNames) {
        $binary = Get-ChildItem -LiteralPath $testDirectory -Filter "$targetName.exe" -File -Recurse |
            Where-Object { $_.BaseName -eq $targetName } |
            Select-Object -First 1
        if ($null -eq $binary) {
            throw "Could not find the built test executable '$targetName'"
        }
        $binary.FullName
    }
)

$objectArguments = @(
    foreach ($binary in ($testBinaries | Select-Object -Skip 1)) {
        "--object=$binary"
    }
)
$sourceFilter = '[/\\](tests|Drivers|Middlewares|Core)[/\\]|[/\\]LWIP[/\\]App[/\\]'
$reportPath = Join-Path $runDirectory 'coverage-summary.txt'
$reportArguments = @(
    'report',
    $testBinaries[0],
    "-instr-profile=$profileData",
    '--show-branch-summary',
    "--ignore-filename-regex=$sourceFilter"
) + $objectArguments
$report = & $llvmCov @reportArguments
if ($LASTEXITCODE -ne 0) {
    throw "llvm-cov report failed with exit code $LASTEXITCODE"
}
$report | Set-Content -LiteralPath $reportPath -Encoding UTF8
$report

New-Item -ItemType Directory -Path $htmlDirectory -Force | Out-Null
$htmlArguments = @(
    'show',
    $testBinaries[0],
    "-instr-profile=$profileData",
    '--format=html',
    "--output-dir=$htmlDirectory",
    "--ignore-filename-regex=$sourceFilter"
) + $objectArguments
& $llvmCov @htmlArguments
if ($LASTEXITCODE -ne 0) {
    throw "llvm-cov HTML report failed with exit code $LASTEXITCODE"
}

Write-Host "Text report: $reportPath"
Write-Host "HTML report: $(Join-Path $htmlDirectory 'index.html')"
if ($testExitCode -ne 0) {
    throw "CTest failed with exit code $testExitCode; coverage reports were still generated."
}
