<#
.SYNOPSIS
Builds and runs the C++ core benchmark and writes a Markdown report.

.PARAMETER Output
Path of the Markdown report. If omitted, the report is only printed.

.PARAMETER Reps
Number of timed repetitions per benchmark.

.PARAMETER Quick
Run 10x fewer iterations.

.PARAMETER NoPin
Do not pin the benchmark thread to a performance core.

.PARAMETER Note
Free-text note added to the report, e.g. vendor performance profile.

.EXAMPLE
.\scripts\run_cpp_benchmark.ps1 -Output docs\results\phase1_speed.md -Note "Vendor profile: Performance"
#>
param(
    [string]$Output = "",
    [int]$Reps = 5,
    [switch]$Quick,
    [switch]$NoPin,
    [string]$Note = ""
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $root "build\bench"

cmake -S $root -B $buildDir -DROBOTSIM_BUILD_BENCHMARKS=ON -DBUILD_TESTING=OFF | Out-Host
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed" }
cmake --build $buildDir --config Release --target robotsim_bench | Out-Host
if ($LASTEXITCODE -ne 0) { throw "Benchmark build failed" }

$exe = Join-Path $buildDir "Release\robotsim_bench.exe"
$benchArgs = @("--reps", "$Reps")
if ($Quick) { $benchArgs += "--quick" }
if (-not $NoPin) { $benchArgs += "--pin" }
$result = & $exe @benchArgs
if ($LASTEXITCODE -ne 0) { throw "Benchmark run failed" }

# ---------------------------------------------------------------------------
# Machine information
# ---------------------------------------------------------------------------
$cpu = Get-CimInstance Win32_Processor | Select-Object -First 1
$ramGb = [math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory / 1GB, 1)
$os = (Get-CimInstance Win32_OperatingSystem).Caption
$plan = ((powercfg /getactivescheme) -replace '^.*\((.*)\)\s*$', '$1').Trim()

# Windows 11 power mode is an overlay on top of the active power plan.
$overlayNames = @{
    "00000000-0000-0000-0000-000000000000" = "Balanced"
    "961cc777-2547-4f9d-8174-7d86181b8a7a" = "Best power efficiency"
    "ded574b5-45a0-4f42-8737-46345c09c238" = "Best performance"
}
$overlay = "unknown"
try {
    $key = Get-ItemProperty "HKLM:\SYSTEM\CurrentControlSet\Control\Power\User\PowerSchemes" -ErrorAction Stop
    $guid = "$($key.ActiveOverlayAcPowerScheme)"
    $overlay = if ($overlayNames.ContainsKey($guid)) { $overlayNames[$guid] } else { "unknown ($guid)" }
} catch { }

$acStatus = "unknown"
try {
    Add-Type -AssemblyName System.Windows.Forms
    $acStatus = "$([System.Windows.Forms.SystemInformation]::PowerStatus.PowerLineStatus)"
} catch { }

$commit = (git -C $root rev-parse --short HEAD).Trim()
if (git -C $root status --porcelain) { $commit += " (with uncommitted changes)" }

# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
$lines = @()
$lines += $result
$lines += ""
$lines += "## Machine"
$lines += ""
$lines += "- CPU: $($cpu.Name.Trim())"
$lines += "- Cores / logical processors: $($cpu.NumberOfCores) / $($cpu.NumberOfLogicalProcessors)"
$lines += "- RAM: $ramGb GB"
$lines += "- OS: $os"
$lines += "- Power plan: $plan; power mode (AC): $overlay; AC power: $acStatus"
if ($Note) { $lines += "- Note: $Note" }
$lines += "- Commit: $commit"
$lines += "- Date: $(Get-Date -Format 'yyyy-MM-dd')"
$lines += ""
$lines += "Reproduce with ``.\scripts\run_cpp_benchmark.ps1``."

$report = ($lines -join "`n") + "`n"
Write-Host ""
Write-Host $report

if ($Output) {
    $outPath = if ([IO.Path]::IsPathRooted($Output)) { $Output } else { Join-Path $root $Output }
    $outDir = Split-Path -Parent $outPath
    if (-not (Test-Path $outDir)) { New-Item -ItemType Directory -Path $outDir -Force | Out-Null }
    [IO.File]::WriteAllText($outPath, $report, (New-Object System.Text.UTF8Encoding $false))
    Write-Host "Report written to $outPath"
}