param(
    [int] $Iterations = 50000,
    [int] $Warmup = 5000,
    [switch] $NoInit,
    [string] $CmakePrefixPath = "",
    [switch] $UseLocalRef,
    [string] $PinocchioSourceDir = "",
    [string] $VcpkgRoot = "",
    [string] $VcpkgTriplet = "x64-windows",
    [switch] $UsePinocchioCMake,
    [switch] $Correctness
)

$ErrorActionPreference = "Stop"

$Root = Resolve-Path "$PSScriptRoot\..\.."
$BenchDir = Join-Path $Root "benchmarks\pinocchio"
$BuildDir = Join-Path $Root "benchmarks\pinocchio\build"
$OutDir = Join-Path $Root "benchmarks\results"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$ConfigureArgs = @("-S", $BenchDir, "-B", $BuildDir, "-DCMAKE_BUILD_TYPE=Release")
if ($VcpkgRoot -eq "") {
    if ($env:VCPKG_ROOT -ne "") {
        $VcpkgRoot = $env:VCPKG_ROOT
    }
}
if ($VcpkgRoot -ne "") {
    $Toolchain = Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake"
    if (Test-Path $Toolchain) {
        $ConfigureArgs += "-DCMAKE_TOOLCHAIN_FILE=$Toolchain"
        $ConfigureArgs += "-DVCPKG_TARGET_TRIPLET=$VcpkgTriplet"
    }
}
if ($CmakePrefixPath -ne "") {
    $ConfigureArgs += "-DCMAKE_PREFIX_PATH=$CmakePrefixPath"
}
if ($UseLocalRef) {
    if ($PinocchioSourceDir -eq "") {
        $PinocchioSourceDir = Join-Path $Root "ref\pinocchio"
    }
    $ConfigureArgs += "-DKINETIX_PINOCCHIO_SOURCE_DIR=$PinocchioSourceDir"
    if ($UsePinocchioCMake) {
        $ConfigureArgs += "-DKINETIX_PINOCCHIO_HEADER_ONLY=OFF"
    }
}

cmake @ConfigureArgs
if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed with exit code $LASTEXITCODE"
}
cmake --build $BuildDir --config Release
if ($LASTEXITCODE -ne 0) {
    throw "CMake build failed with exit code $LASTEXITCODE"
}

$Exe = Join-Path $BuildDir "pinocchio_bench.exe"
if (!(Test-Path $Exe)) {
    $FoundExe = Get-ChildItem $BuildDir -Recurse -File -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -eq "pinocchio_bench.exe" -or $_.Name -eq "pinocchio_bench" } |
        Select-Object -First 1
    if ($null -eq $FoundExe) {
        throw "Could not find pinocchio_bench executable under $BuildDir"
    }
    $Exe = $FoundExe.FullName
}

$RunArgs = @()
if ($Correctness) {
    $RunArgs += "--correctness"
    $OutputFile = Join-Path $OutDir "pinocchio_correctness.csv"
}
else {
    $RunArgs += @("--iterations", "$Iterations", "--warmup", "$Warmup")
    if ($NoInit) {
        $RunArgs += "--no-init"
    }
    $OutputFile = Join-Path $OutDir "pinocchio.csv"
}

& $Exe @RunArgs | Tee-Object -FilePath $OutputFile
