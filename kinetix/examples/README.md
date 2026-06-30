# Kinetix Examples

Run examples from the workspace root:

```powershell
cargo run -p kinetix --example 01_spatial_algebra --offline
cargo run -p kinetix --example 02_inverse_dynamics --offline
cargo run -p kinetix --example 03_forward_dynamics_roundtrip --offline
cargo run -p kinetix --example 04_mass_matrix --offline
cargo run -p kinetix --example 05_urdf_loading --offline
cargo run -p kinetix --example 06_rnea_derivatives --offline
cargo run -p kinetix --example 07_realtime_data_reuse --offline --release
```

The set mirrors the Pinocchio examples that are relevant to Kinetix today:

- spatial algebra basics,
- inverse dynamics,
- forward dynamics,
- mass matrix computation,
- URDF model loading,
- inverse dynamics derivatives,
- allocation-free data reuse in a control loop.

Viewer, collision, contact, and inverse-kinematics examples will make sense
once those modules exist in Kinetix.
