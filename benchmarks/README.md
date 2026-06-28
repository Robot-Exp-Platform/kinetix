# Kinetix / Pinocchio Benchmarks

This directory contains reproducible performance runners for comparing the
current static Kinetix core against upstream Pinocchio.

The comparison is intentionally split into three groups:

- `spatial`: 6D spatial algebra microbenchmarks.
- `dynamics`: hot-path `rnea`, `crba`, and `aba`.
- `init`: non-realtime initialization, currently model construction and Kinetix
  URDF loading.

The hot-path runners construct `Model` and `Data` outside the timed loop and
reuse them for every iteration.

## Fairness Boundary

Kinetix is currently a static-size Rust implementation with a small joint set
(`RevoluteZ`, `PrismaticZ`, `Fixed`). Pinocchio is a much more general C++
library with a runtime model and a broad joint/geometry ecosystem.

Treat the first comparison as:

```text
Kinetix static-specialized hot path
vs
Pinocchio generic runtime hot path
```

It is useful for realtime-control latency investigation, but it is not yet a
feature-complete library comparison.

## Run Kinetix

```powershell
.\benchmarks\scripts\run_kinetix.ps1 -Iterations 50000 -Warmup 5000
```

Output is written to:

```text
benchmarks/results/kinetix.csv
```

For a quick smoke run:

```powershell
.\benchmarks\scripts\run_kinetix.ps1 -Iterations 1000 -Warmup 100 -NoInit
```

## Run Pinocchio

The Pinocchio runner expects an installed CMake package:

```powershell
.\benchmarks\scripts\run_pinocchio.ps1 -Iterations 50000 -Warmup 5000
```

If Pinocchio is installed in a custom prefix:

```powershell
.\benchmarks\scripts\run_pinocchio.ps1 -CmakePrefixPath C:\path\to\pinocchio
```

To build against the local `ref/pinocchio` source tree instead of an installed
CMake package:

```powershell
.\benchmarks\scripts\run_pinocchio.ps1 -UseLocalRef
```

This mode uses `ref/pinocchio/include` directly and generates a tiny
benchmark-local `pinocchio/config.hpp` shim inside the CMake build directory.
It avoids building the full Pinocchio project and does not require the
`ref/pinocchio/cmake` nested submodule.

The script automatically uses `VCPKG_ROOT` when it is set. You can also pass it
explicitly:

```powershell
.\benchmarks\scripts\run_pinocchio.ps1 `
  -UseLocalRef `
  -VcpkgRoot D:\environment\vcpkg\vcpkg `
  -VcpkgTriplet x64-windows
```

If you want to build Pinocchio through its own CMake project instead of the
header-only shim, initialize its nested submodules and pass
`-UsePinocchioCMake`:

```powershell
git -C ref/pinocchio submodule update --init --recursive
.\benchmarks\scripts\run_pinocchio.ps1 -UseLocalRef -UsePinocchioCMake
```

Output is written to:

```text
benchmarks/results/pinocchio.csv
```

The local header-only mode still requires C++ dependencies such as Eigen and
Boost to be discoverable by CMake. URDF/collision/python support is disabled for
this benchmark runner.

## Compare

```powershell
python .\benchmarks\scripts\compare.py `
  .\benchmarks\results\kinetix.csv `
  .\benchmarks\results\pinocchio.csv
```

The `kinetix_speedup` column is:

```text
pinocchio_ns_per_iter / kinetix_ns_per_iter
```

Values above `1.0` mean Kinetix is faster for that row.

## Correctness Gate

Before reading timing numbers, run the numerical behavior check. It exports the
same deterministic spatial and dynamics workloads from both implementations and
compares every scalar output.

```powershell
.\benchmarks\scripts\run_kinetix.ps1 -Correctness
.\benchmarks\scripts\run_pinocchio.ps1 -UseLocalRef -Correctness

python .\benchmarks\scripts\compare_correctness.py `
  .\benchmarks\results\kinetix_correctness.csv `
  .\benchmarks\results\pinocchio_correctness.csv `
  1e-9
```

The current gate covers:

- `spatial`: motion cross, force cross, motion transform, force transform,
  inertia application.
- `dynamics`: `rnea`, `crba`, and `aba` on serial revolute chains with
  `1, 2, 6, 7, 12, 18, 30` DoF.

## CSV Format

Both runners emit:

```text
implementation,group,model,algorithm,dof,iterations,total_ns,ns_per_iter,checksum
```

Rows are matched by:

```text
group, model, algorithm, dof
```

`checksum` exists only to discourage dead-code elimination and to make gross
semantic drift visible. It is not a robust numerical equivalence proof. Use the
correctness gate above as the behavior oracle, then use the speedup columns as
hot-path timing data.
