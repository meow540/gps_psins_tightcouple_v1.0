param(
    [ValidateSet('proof', 'online', 'online_loose', 'online_ins', 'online_prop', 'online_base')]
    [string]$Mode = 'online_loose',
    [int]$EndSec = 400,
    [double]$StartSec = 176.0,
    [string]$MatlabExe = 'matlab',
    [string]$PrefDirPrefix = '.matlab_prefs_batch'
)

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $projectRoot

$env:DS5_HEAP_SAFE_ANCHOR_MODE = $Mode.ToLowerInvariant()
$env:DS5_HEAP_SAFE_END_SEC = [string]$EndSec
$env:DS5_HEAP_SAFE_START_SEC = [string]::Format([System.Globalization.CultureInfo]::InvariantCulture, '{0:0.0}', $StartSec)
$env:OMP_NUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'
$env:OPENBLAS_NUM_THREADS = '1'

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$prefDirName = '{0}_{1}_{2}s_{3}' -f $PrefDirPrefix.Trim(), $env:DS5_HEAP_SAFE_ANCHOR_MODE, $EndSec, $stamp
$env:MATLAB_PREFDIR = Join-Path $projectRoot $prefDirName
New-Item -ItemType Directory -Force -Path $env:MATLAB_PREFDIR | Out-Null

$outPrefix = 'rt_deep_goal2_ds5_heap_safe_true_tracking_goal_{0:000}s' -f $EndSec
if ($env:DS5_HEAP_SAFE_ANCHOR_MODE -ne 'proof') {
    $outPrefix = '{0}_{1}' -f $outPrefix, $env:DS5_HEAP_SAFE_ANCHOR_MODE
}
$doneFile = Join-Path $projectRoot ($outPrefix + '_summary_done.txt')
if (Test-Path $doneFile) {
    Remove-Item $doneFile -Force
}

Write-Host ("[HEAP-SAFE] projectRoot={0}" -f $projectRoot)
Write-Host ("[HEAP-SAFE] mode={0} endSec={1} startSec={2}" -f $env:DS5_HEAP_SAFE_ANCHOR_MODE, $EndSec, $env:DS5_HEAP_SAFE_START_SEC)
Write-Host ("[HEAP-SAFE] MATLAB_PREFDIR={0}" -f $env:MATLAB_PREFDIR)

& $MatlabExe -singleCompThread -batch "run('run_ds5_400s_heap_safe_true_tracking_goal.m')"
$matlabExit = $LASTEXITCODE

$doneOk = Test-Path $doneFile
$resultOk = $false
$resultPath = $null
if ($doneOk) {
    $doneMap = @{}
    foreach ($line in Get-Content $doneFile) {
        if ($line -match '^\s*([^=]+)=(.*)\s*$') {
            $doneMap[$matches[1]] = $matches[2]
        }
    }
    if ($doneMap.ContainsKey('resultFile')) {
        $candidate = Join-Path $projectRoot $doneMap['resultFile']
        if (Test-Path $candidate) {
            $resultOk = $true
            $resultPath = $candidate
        }
    }
}

if ($matlabExit -ne 0 -and $doneOk -and $resultOk) {
    Write-Warning ("MATLAB exited with code 0x{0:X8}, but summary_done and result MAT exist. Treating run as successful. result={1}" -f ($matlabExit -band 0xffffffff), $resultPath)
    exit 0
}

if ($matlabExit -ne 0) {
    Write-Error ("MATLAB exited with code 0x{0:X8}. summary_done={1} resultOk={2}" -f ($matlabExit -band 0xffffffff), $doneOk, $resultOk)
}

exit $matlabExit
