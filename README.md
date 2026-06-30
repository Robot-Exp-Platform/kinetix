# Kinetix

Kinetix is an experimental Rust rigid-body dynamics workspace inspired by
[Pinocchio](https://github.com/stack-of-tasks/pinocchio). Its goal is not to
clone Pinocchio feature-for-feature, but to explore what a robotics dynamics
library can look like when the core ideas of spatial algebra, model/data
separation, and recursive rigid-body algorithms are expressed with Rust's type
system and zero-cost abstractions.

The project is currently early-stage. The implemented surface focuses on
typed spatial algebra, fixed-size allocation-free dynamics, URDF loading,
forward-mode inverse-dynamics derivatives, examples, and Pinocchio-aligned
correctness benchmarks.

## Motivation

Robotics dynamics code is full of bugs that are hard to see in review:
mixing frames, applying a force transform where a motion transform was needed,
reallocating inside a realtime loop, or sharing mutable algorithm buffers across
threads. Kinetix is built around a few design constraints meant to make those
mistakes harder:

- **Type-driven frame safety**: spatial vectors carry their expressed frame in
  the Rust type. Adding motions from different frames should fail at compile
  time.
- **No allocation in the hot path**: `kinetix-spatial` and `kinetix-core` are
  `#![no_std]` and use static `nalgebra` matrices/vectors.
- **Model/Data split**: immutable robot structure lives in `Model`; temporary
  algorithm state lives in reusable `Data`, following the Pinocchio pattern.
- **Static DOF specialization**: joint vectors and matrices are sized by
  `const DOF: usize`, giving the compiler more room to optimize.
- **Generic scalar path**: dynamics code can run with `f64` or a dual number
  scalar for automatic differentiation.

## Pinocchio Reference

Pinocchio is the primary design and numerical reference for this repository. It
is a mature C++ library for efficient rigid-body dynamics and analytical
derivatives, built on the classic algorithms of Roy Featherstone and used
widely in robotics research and industry.

Kinetix borrows several architectural lessons from Pinocchio:

- spatial motion and force algebra,
- recursive Newton-Euler inverse dynamics,
- composite rigid body mass matrix computation,
- articulated body forward dynamics,
- strict separation between immutable `Model` and reusable `Data`,
- offline model construction from URDF.

This repository includes Pinocchio as a local reference under `ref/pinocchio`.
The benchmark harness can compile against that local source tree and compare
Kinetix outputs against Pinocchio outputs. Kinetix is not a drop-in replacement
for Pinocchio; it is a Rust-first experiment with a narrower current feature
set.

## Performance Snapshot

The latest local benchmark snapshot was generated on 2026-06-30 with:

```powershell
.\benchmarks\scripts\run_kinetix.ps1 -Iterations 50000 -Warmup 5000 -NoInit
.\benchmarks\scripts\run_pinocchio.ps1 -UseLocalRef -Iterations 50000 -Warmup 5000 -NoInit
python .\benchmarks\scripts\compare.py `
  .\benchmarks\results\kinetix.csv `
  .\benchmarks\results\pinocchio.csv
```

The comparison uses the local `ref/pinocchio` source tree. Correctness is checked
separately by the correctness gate before timing is interpreted. Values below
are nanoseconds per iteration; `speedup = pinocchio_ns / kinetix_ns`, so values
above `1.0` mean Kinetix is faster.

| Workload | Kinetix vs Pinocchio |
| --- | --- |
| Spatial algebra micro-ops | Kinetix was `1.47x` to `2.30x` faster |
| RNEA on serial chains | Kinetix was `1.11x` to `1.23x` faster |
| ABA on serial chains | Kinetix was `1.30x` to `1.62x` faster |
| CRBA on serial chains | Pinocchio was faster; Kinetix ran at `0.54x` to `0.61x` |

Selected raw rows:

| Algorithm | DoF | Kinetix ns/iter | Pinocchio ns/iter | Speedup |
| --- | ---: | ---: | ---: | ---: |
| RNEA | 2 | `156.114` | `190.772` | `1.222x` |
| RNEA | 12 | `1062.394` | `1174.760` | `1.106x` |
| RNEA | 30 | `2544.262` | `3008.250` | `1.182x` |
| ABA | 2 | `268.400` | `435.544` | `1.623x` |
| ABA | 12 | `2222.222` | `3030.868` | `1.364x` |
| ABA | 30 | `6025.446` | `7828.530` | `1.299x` |
| CRBA | 2 | `165.264` | `97.306` | `0.589x` |
| CRBA | 12 | `3056.106` | `1644.726` | `0.538x` |
| CRBA | 30 | `15448.452` | `9464.182` | `0.613x` |

This is an implementation snapshot, not a general claim about all robots or all
machines. It also shows the next obvious optimization target: CRBA still spends
too much time in generic spatial inertia and force transforms.

## Workspace Layout

```text
kinetix/
|-- kinetix/             # Umbrella crate and examples
|-- kinetix-spatial/     # no_std frame-safe spatial algebra
|-- kinetix-core/        # no_std RNEA, CRBA, ABA, Model/Data
|-- kinetix-urdf/        # std URDF parser and topology flattener
|-- kinetix-autodiff/    # no_std forward-mode derivative helpers
|-- kinetix-py/          # placeholder for future Python bindings
|-- benchmarks/          # Kinetix vs local Pinocchio checks and timings
`-- ref/pinocchio/       # Local reference copy of Pinocchio
```

The top-level `kinetix` crate re-exports the lower crates:

```rust
use kinetix::prelude::*;
```

For lower-level work, each subcrate can still be used directly.

## Quick Start

Build and test the workspace:

```powershell
cargo test --workspace --offline
```

Run a first inverse-dynamics example:

```powershell
cargo run -p kinetix --example 02_inverse_dynamics --offline
```

Run the examples in release mode when looking at realtime-style behavior:

```powershell
cargo run -p kinetix --example 07_realtime_data_reuse --offline --release
```

## Teaching Examples

The examples live in `kinetix/examples`. They are intentionally small and
ordered as a learning path:

| Example | Topic | What it teaches |
| --- | --- | --- |
| `01_spatial_algebra` | Spatial vectors and transforms | Angular-first 6D motion/force vectors, cross products, frame transforms |
| `02_inverse_dynamics` | RNEA | Compute joint torques from `q`, `q_dot`, and `q_ddot` |
| `03_forward_dynamics_roundtrip` | RNEA + ABA | Use RNEA torque as ABA input and recover the target acceleration |
| `04_mass_matrix` | CRBA | Build the generalized inertia matrix `M(q)` |
| `05_urdf_loading` | URDF | Load a small double pendulum from a URDF string |
| `06_rnea_derivatives` | Autodiff | Compute `d tau / d q` and `d tau / d q_dot` with the same RNEA implementation |
| `07_realtime_data_reuse` | Realtime pattern | Allocate `Data` once and reuse it across control ticks |

Run them from the workspace root:

```powershell
cargo run -p kinetix --example 01_spatial_algebra --offline
cargo run -p kinetix --example 02_inverse_dynamics --offline
cargo run -p kinetix --example 03_forward_dynamics_roundtrip --offline
cargo run -p kinetix --example 04_mass_matrix --offline
cargo run -p kinetix --example 05_urdf_loading --offline
cargo run -p kinetix --example 06_rnea_derivatives --offline
cargo run -p kinetix --example 07_realtime_data_reuse --offline --release
```

Pinocchio has examples for visualization, collision, contact, inverse
kinematics, and reduced models. Kinetix does not include those examples yet
because the corresponding modules do not exist yet.

## Read-Only Examples

The repository examples are the canonical runnable versions, but the snippets
below show the basic shape of the API without requiring the reader to open the
source tree.

### Spatial algebra with frame tags

```rust
use kinetix::prelude::*;

let motion_w = SpatialMotion::<WorldFrame>::from_vector(Vector6::new(
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
));

let x_w_l1 = SpatialTransform::<WorldFrame, LinkFrame<1>>::new(
    Matrix3::identity(),
    Vector3::new(0.4, -0.2, 0.7),
);

let motion_l1 = x_w_l1.apply_motion(&motion_w);
println!("motion in link frame = {}", motion_l1.to_vector().transpose());
```

The type parameter is part of the safety story: a
`SpatialMotion<WorldFrame>` cannot be accidentally added to a
`SpatialMotion<LinkFrame<1>>`.

### A tiny inverse-dynamics call

```rust
use kinetix::prelude::*;

fn rigid_inertia(mass: f64, com: Vector3) -> SpatialInertia<WorldFrame> {
    SpatialInertia::from_mass_com_inertia(mass, com, Matrix3::identity() * 0.01)
}

let model = Model::<2>::new(
    [0, 0],
    [
        SpatialTransform::identity(),
        SpatialTransform::new(Matrix3::identity(), Vector3::new(1.0, 0.0, 0.0)),
    ],
    [
        rigid_inertia(1.7, Vector3::new(0.45, 0.0, 0.0)),
        rigid_inertia(1.1, Vector3::new(0.35, 0.0, 0.0)),
    ],
    [JointType::RevoluteZ, JointType::RevoluteZ],
    Vector3::new(0.0, -9.81, 0.0),
);

let mut data = Data::<2>::default();
data.q = JointVector::<2>::new(0.3, -0.8);
data.q_dot = JointVector::<2>::new(0.7, -0.2);
data.q_ddot = JointVector::<2>::new(1.1, -0.4);

rnea(&model, &mut data, None);
println!("tau = {}", data.tau.transpose());
```

### Reusing `Data` in a control loop

```rust
use kinetix::prelude::*;

// Your initialization-time model builder or URDF loader.
let model: Model<2> = build_model_once();
let mut data = Data::<2>::default();

loop {
    // Update q, q_dot, q_ddot or tau from sensors/controllers.
    rnea(&model, &mut data, None);

    // Send data.tau to the actuator layer.
    break; // only here to keep the README snippet finite
}
```

`Model` is immutable and can be shared. `Data` is the mutable scratchpad and is
meant to be reused by one realtime thread.

### URDF loading at initialization time

```rust
use kinetix::prelude::*;

let urdf = r#"
<robot name="one_joint">
  <link name="base"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

let model = load_from_str::<1>(urdf)?;
```

### RNEA derivatives with the same dynamics implementation

```rust
use kinetix::prelude::*;

// Your initialization-time model builder or URDF loader.
let model: Model<2> = build_model_once();
let q = JointVector::<2>::new(0.31, -0.43);
let q_dot = JointVector::<2>::new(0.37, -0.22);
let q_ddot = JointVector::<2>::new(0.91, -0.34);

let (dtau_dq, dtau_dq_dot) = compute_rnea_derivatives(&model, &q, &q_dot, &q_ddot);
println!("d tau / d q =\n{dtau_dq}");
println!("d tau / d q_dot =\n{dtau_dq_dot}");
```

## Core Design

### Spatial Algebra

`kinetix-spatial` defines:

- `SpatialMotion<F, T>` for `[omega; v]`,
- `SpatialForce<F, T>` for `[torque; force]`,
- `SpatialTransform<From, To, T>`,
- `SpatialInertia<F, T>`,
- frame tags such as `WorldFrame` and `LinkFrame<ID>`.

The frame parameter is a compile-time guard. A value expressed in
`WorldFrame` cannot be accidentally added to one expressed in `LinkFrame<1>`.

### Dynamics Core

`kinetix-core` defines:

- `Model<DOF, T = f64>`: immutable robot topology, transforms, inertias,
  joint types, and gravity,
- `Data<DOF, T = f64>`: reusable algorithm workspace,
- `rnea`: inverse dynamics,
- `crba`: generalized inertia matrix,
- `aba`: forward dynamics.

The hot path is allocation-free. `Model` is shareable read-only state, while
`Data` is owned mutably by the thread running the algorithm.

### URDF Loading

`kinetix-urdf` is allowed to use `std`, because XML parsing and graph traversal
happen during initialization. It reads URDF from a file or string, topologically
sorts active joints, and flattens the robot into a `Model<DOF>`.

### Automatic Differentiation

`kinetix-autodiff` provides a first-order dual number and derivative helpers.
The key idea is that RNEA is not rewritten for derivatives; instead, the scalar
type changes from `f64` to `Dual`, and the same generic dynamics code propagates
tangents.

## Correctness Against Pinocchio

The benchmark folder contains a behavior gate that exports deterministic
results from both Kinetix and Pinocchio and compares every scalar output.

```powershell
.\benchmarks\scripts\run_kinetix.ps1 -Correctness
.\benchmarks\scripts\run_pinocchio.ps1 -UseLocalRef -Correctness

python .\benchmarks\scripts\compare_correctness.py `
  .\benchmarks\results\kinetix_correctness.csv `
  .\benchmarks\results\pinocchio_correctness.csv `
  1e-9
```

The current gate covers spatial algebra and `rnea`, `crba`, `aba` on serial
revolute chains with `1, 2, 6, 7, 12, 18, 30` DoF.

## Performance Benchmarks

After the correctness gate passes, timing comparisons can be run with:

```powershell
.\benchmarks\scripts\run_kinetix.ps1 -Iterations 50000 -Warmup 5000 -NoInit
.\benchmarks\scripts\run_pinocchio.ps1 -UseLocalRef -Iterations 50000 -Warmup 5000 -NoInit

python .\benchmarks\scripts\compare.py `
  .\benchmarks\results\kinetix.csv `
  .\benchmarks\results\pinocchio.csv
```

Interpret the numbers carefully. Kinetix is a static-size, narrow-joint-set
implementation; Pinocchio is a much broader runtime model library. The timing
comparison is useful for realtime hot-path investigation, not as a complete
library comparison.

## Current Limitations

- Only `RevoluteZ`, `PrismaticZ`, and `Fixed` joints are implemented.
- No geometry, collision, contact dynamics, closed-loop constraints, or inverse
  kinematics yet.
- URDF support is intentionally small and currently maps only supported joint
  and inertia patterns.
- The project is still experimental and API stability is not promised.

## Roadmap

- Expand the joint model set.
- Remove remaining generic 6x6 operations from hot loops where specialized 3D
  formulas are available.
- Add richer URDF coverage and model diagnostics.
- Add kinematics and Jacobian APIs.
- Add geometry/collision modules.
- Explore generated robot-specific kernels for fixed robots.
- Grow Python bindings once the Rust API settles.

## Citation

If you use Kinetix in academic work, please cite the software repository:

```bibtex
@software{kinetix2026,
   author = {{Robot-Exp Platform contributors}},
   title = {Kinetix: A Rust-first rigid-body dynamics library with type-driven frame safety},
   url = {https://github.com/Robot-Exp-Platform/kinetix},
   year = {2026},
   note = {Experimental software}
}
```

Kinetix is inspired by Pinocchio and uses Pinocchio as a numerical reference in
its benchmarks. If your work relies on that comparison or on the Pinocchio
design lineage, please also cite Pinocchio.

To cite **Pinocchio** in your academic research, please consider citing the
[software paper](https://laas.hal.science/hal-01866228v2/file/19-sii-pinocchio.pdf)
and use the following BibTeX entry:

```bibtex
@inproceedings{carpentier2019pinocchio,
   title={The Pinocchio C++ library -- A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives},
   author={Carpentier, Justin and Saurel, Guilhem and Buondonno, Gabriele and Mirabel, Joseph and Lamiraux, Florent and Stasse, Olivier and Mansard, Nicolas},
   booktitle={IEEE International Symposium on System Integrations (SII)},
   year={2019}
}
```

And the following one for the link to the GitHub codebase:

```bibtex
@misc{pinocchioweb,
   author = {Justin Carpentier and Florian Valenza and Nicolas Mansard and others},
   title = {Pinocchio: fast forward and inverse dynamics for poly-articulated systems},
   howpublished = {https://stack-of-tasks.github.io/pinocchio},
   year = {2015--2021}
}
```

## Acknowledgment

Kinetix exists because Pinocchio has already shown how powerful a carefully
designed rigid-body dynamics library can be. This project uses Pinocchio as a
conceptual reference, a numerical oracle, and a benchmark counterpart while
trying a Rust-native path for type-driven safety and allocation-free realtime
control.
