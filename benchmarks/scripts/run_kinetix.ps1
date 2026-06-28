param(
    [int] $Iterations = 50000,
    [int] $Warmup = 5000,
    [switch] $NoInit,
    [switch] $Correctness
)

$ErrorActionPreference = "Stop"

$Root = Resolve-Path "$PSScriptRoot\..\.."
$OutDir = Join-Path $Root "benchmarks\results"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$ArgsList = @(
    "run", "-p", "kinetix-benchmarks", "--release", "--"
)

if ($Correctness) {
    $ArgsList += "--correctness"
    $OutputFile = Join-Path $OutDir "kinetix_correctness.csv"
}
else {
    $ArgsList += @("--iterations", "$Iterations", "--warmup", "$Warmup")
    if ($NoInit) {
        $ArgsList += "--no-init"
    }
    $OutputFile = Join-Path $OutDir "kinetix.csv"
}

Push-Location $Root
try {
    cargo @ArgsList | Tee-Object -FilePath $OutputFile
}
finally {
    Pop-Location
}
