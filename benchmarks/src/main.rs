use std::env;
use std::hint::black_box;
use std::time::Instant;

use kinetix_core::{Data, JointType, Model, aba, crba, rnea};
use kinetix_spatial::{
    Matrix3, SpatialForce, SpatialInertia, SpatialMotion, SpatialTransform, Vector3, Vector6,
    WorldFrame,
};

#[derive(Clone, Copy)]
struct Config {
    iterations: u64,
    warmup: u64,
    include_init: bool,
    correctness: bool,
}

impl Config {
    fn from_args() -> Self {
        let mut config = Self {
            iterations: 50_000,
            warmup: 5_000,
            include_init: true,
            correctness: false,
        };

        let mut args = env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--iterations" => {
                    config.iterations = args
                        .next()
                        .expect("--iterations requires a value")
                        .parse()
                        .expect("--iterations must be an integer");
                }
                "--warmup" => {
                    config.warmup = args
                        .next()
                        .expect("--warmup requires a value")
                        .parse()
                        .expect("--warmup must be an integer");
                }
                "--no-init" => config.include_init = false,
                "--correctness" => config.correctness = true,
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other => panic!("unknown argument: {other}"),
            }
        }

        config
    }
}

fn main() {
    let config = Config::from_args();
    if config.correctness {
        println!("implementation,group,model,quantity,dof,row,col,value");
        run_correctness();
        return;
    }

    println!("implementation,group,model,algorithm,dof,iterations,total_ns,ns_per_iter,checksum");

    run_spatial(&config);
    run_dynamics::<2>("chain_2dof", &config);
    run_dynamics::<6>("chain_6dof", &config);
    run_dynamics::<7>("chain_7dof", &config);
    run_dynamics::<12>("chain_12dof", &config);
    run_dynamics::<18>("chain_18dof", &config);
    run_dynamics::<30>("chain_30dof", &config);

    if config.include_init {
        run_initialization::<2>("chain_2dof", &config);
        run_initialization::<6>("chain_6dof", &config);
        run_initialization::<7>("chain_7dof", &config);
        run_initialization::<12>("chain_12dof", &config);
        run_initialization::<18>("chain_18dof", &config);
        run_initialization::<30>("chain_30dof", &config);
    }
}

fn print_help() {
    println!(
        "\
Usage: cargo run -p kinetix-benchmarks --release -- [options]

Options:
  --iterations N  Timed iterations per benchmark, default 50000
  --warmup N      Warmup iterations per benchmark, default 5000
  --no-init       Skip non-realtime initialization benchmarks
  --correctness   Emit deterministic numeric outputs instead of timings
  -h, --help      Show this help
"
    );
}

fn run_correctness() {
    emit_spatial_correctness();
    emit_dynamics_correctness::<1>("chain_1dof");
    emit_dynamics_correctness::<2>("chain_2dof");
    emit_dynamics_correctness::<6>("chain_6dof");
    emit_dynamics_correctness::<7>("chain_7dof");
    emit_dynamics_correctness::<12>("chain_12dof");
    emit_dynamics_correctness::<18>("chain_18dof");
    emit_dynamics_correctness::<30>("chain_30dof");
}

fn emit_spatial_correctness() {
    let motion_a =
        SpatialMotion::<WorldFrame>::from_vector(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
    let motion_b =
        SpatialMotion::<WorldFrame>::from_vector(Vector6::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0));
    let force = SpatialForce::<WorldFrame>::from_vector(Vector6::new(0.7, 0.8, 0.9, 1.0, 1.1, 1.2));
    let transform =
        SpatialTransform::<WorldFrame, WorldFrame>::new(rz(0.31), Vector3::new(0.4, -0.2, 0.7));
    let inertia = rigid_inertia(1.7, Vector3::new(0.25, -0.1, 0.05));

    emit_vector(
        "spatial",
        "spatial_ops",
        "cross_motion",
        0,
        motion_a.cross_motion(&motion_b).to_vector().as_slice(),
    );
    emit_vector(
        "spatial",
        "spatial_ops",
        "cross_force",
        0,
        motion_a.cross_force(&force).to_vector().as_slice(),
    );
    emit_vector(
        "spatial",
        "spatial_ops",
        "apply_motion",
        0,
        transform.apply_motion(&motion_a).to_vector().as_slice(),
    );
    emit_vector(
        "spatial",
        "spatial_ops",
        "apply_force",
        0,
        transform.apply_force(&force).to_vector().as_slice(),
    );
    emit_vector(
        "spatial",
        "spatial_ops",
        "inertia_apply_motion",
        0,
        inertia.apply_motion(&motion_a).to_vector().as_slice(),
    );
}

fn emit_dynamics_correctness<const DOF: usize>(model_name: &str) {
    let model = serial_revolute_model::<DOF>();
    let mut data = seeded_data::<DOF>();

    rnea(&model, &mut data, None);
    let tau = data.tau;
    emit_vector("dynamics", model_name, "rnea_tau", DOF, tau.as_slice());

    crba(&model, &mut data);
    for row in 0..DOF {
        for col in 0..DOF {
            emit_scalar(
                "dynamics",
                model_name,
                "crba_m",
                DOF,
                row,
                col,
                data.m_matrix[(row, col)],
            );
        }
    }

    data.tau = deterministic_tau::<DOF>();
    aba(&model, &mut data, None);
    emit_vector(
        "dynamics",
        model_name,
        "aba_qddot",
        DOF,
        data.q_ddot.as_slice(),
    );
}

fn emit_vector(group: &str, model: &str, quantity: &str, dof: usize, values: &[f64]) {
    for (row, value) in values.iter().enumerate() {
        emit_scalar(group, model, quantity, dof, row, 0, *value);
    }
}

fn emit_scalar(
    group: &str,
    model: &str,
    quantity: &str,
    dof: usize,
    row: usize,
    col: usize,
    value: f64,
) {
    println!("kinetix,{group},{model},{quantity},{dof},{row},{col},{value:.17e}");
}

fn run_spatial(config: &Config) {
    let motion_a =
        SpatialMotion::<WorldFrame>::from_vector(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
    let motion_b =
        SpatialMotion::<WorldFrame>::from_vector(Vector6::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0));
    let force = SpatialForce::<WorldFrame>::from_vector(Vector6::new(0.7, 0.8, 0.9, 1.0, 1.1, 1.2));
    let transform =
        SpatialTransform::<WorldFrame, WorldFrame>::new(rz(0.31), Vector3::new(0.4, -0.2, 0.7));
    let inertia = rigid_inertia(1.7, Vector3::new(0.25, -0.1, 0.05));

    emit("spatial", "spatial_ops", "cross_motion", 0, config, || {
        black_box(motion_a.cross_motion(black_box(&motion_b))).to_vector()[0]
    });
    emit("spatial", "spatial_ops", "cross_force", 0, config, || {
        black_box(motion_a.cross_force(black_box(&force))).to_vector()[0]
    });
    emit("spatial", "spatial_ops", "apply_motion", 0, config, || {
        black_box(transform.apply_motion(black_box(&motion_a))).to_vector()[0]
    });
    emit("spatial", "spatial_ops", "apply_force", 0, config, || {
        black_box(transform.apply_force(black_box(&force))).to_vector()[0]
    });
    emit(
        "spatial",
        "spatial_ops",
        "inertia_apply_motion",
        0,
        config,
        || black_box(inertia.apply_motion(black_box(&motion_a))).to_vector()[0],
    );
}

fn run_dynamics<const DOF: usize>(model_name: &str, config: &Config) {
    let model = serial_revolute_model::<DOF>();

    let mut rnea_data = seeded_data::<DOF>();
    emit("dynamics", model_name, "rnea", DOF, config, || {
        perturb_state(&mut rnea_data);
        rnea(black_box(&model), black_box(&mut rnea_data), None);
        black_box(rnea_data.tau[0])
    });

    let mut crba_data = seeded_data::<DOF>();
    emit("dynamics", model_name, "crba", DOF, config, || {
        perturb_state(&mut crba_data);
        crba(black_box(&model), black_box(&mut crba_data));
        black_box(crba_data.m_matrix[(0, 0)])
    });

    let mut aba_data = seeded_data::<DOF>();
    let tau_seed = tau_for_target_acceleration(&model, &aba_data);
    aba_data.tau = tau_seed;
    emit("dynamics", model_name, "aba", DOF, config, || {
        perturb_state(&mut aba_data);
        aba(black_box(&model), black_box(&mut aba_data), None);
        black_box(aba_data.q_ddot[0])
    });
}

fn run_initialization<const DOF: usize>(model_name: &str, config: &Config) {
    emit("init", model_name, "build_model", DOF, config, || {
        let model = black_box(serial_revolute_model::<DOF>());
        black_box(model.gravity[1])
    });

    let urdf = serial_revolute_urdf(DOF);
    emit("init", model_name, "load_urdf", DOF, config, || {
        let model = kinetix_urdf::load_from_str::<DOF>(black_box(&urdf)).expect("valid URDF");
        black_box(model.gravity[0])
    });
}

fn emit<F>(group: &str, model: &str, algorithm: &str, dof: usize, config: &Config, mut operation: F)
where
    F: FnMut() -> f64,
{
    let (total_ns, checksum) = measure(config.iterations, config.warmup, &mut operation);
    let ns_per_iter = total_ns as f64 / config.iterations as f64;
    println!(
        "kinetix,{group},{model},{algorithm},{dof},{},{total_ns},{ns_per_iter:.3},{checksum:.12e}",
        config.iterations
    );
}

fn measure<F>(iterations: u64, warmup: u64, operation: &mut F) -> (u128, f64)
where
    F: FnMut() -> f64,
{
    let mut checksum = 0.0;
    for _ in 0..warmup {
        checksum += black_box(operation());
    }

    let start = Instant::now();
    for _ in 0..iterations {
        checksum += black_box(operation());
    }
    (start.elapsed().as_nanos(), black_box(checksum))
}

fn serial_revolute_model<const DOF: usize>() -> Model<DOF> {
    let parents = std::array::from_fn(|i| if i == 0 { 0 } else { i - 1 });
    let transforms = std::array::from_fn(|i| {
        if i == 0 {
            SpatialTransform::identity()
        } else {
            SpatialTransform::new(Matrix3::identity(), Vector3::new(-1.0, 0.0, 0.0))
        }
    });
    let inertias = std::array::from_fn(|i| {
        rigid_inertia(
            1.0 + 0.05 * i as f64,
            Vector3::new(0.35 + 0.002 * i as f64, 0.01, 0.0),
        )
    });
    Model::new(
        parents,
        transforms,
        inertias,
        [JointType::RevoluteZ; DOF],
        Vector3::new(0.0, -9.81, 0.0),
    )
}

fn seeded_data<const DOF: usize>() -> Data<DOF> {
    let mut data = Data::<DOF>::default();
    let tau = deterministic_tau::<DOF>();
    for i in 0..DOF {
        let k = i as f64 + 1.0;
        data.q[i] = 0.03 * k - 0.02 * (i % 3) as f64;
        data.q_dot[i] = 0.04 * k - 0.01 * (i % 5) as f64;
        data.q_ddot[i] = 0.02 * k + 0.005 * (i % 7) as f64;
        data.tau[i] = tau[i];
    }
    data
}

fn deterministic_tau<const DOF: usize>() -> nalgebra::SVector<f64, DOF> {
    nalgebra::SVector::from_fn(|i, _| 0.1 + 0.03 * (i as f64 + 1.0))
}

fn perturb_state<const DOF: usize>(data: &mut Data<DOF>) {
    if DOF == 0 {
        return;
    }
    data.q[0] += 1.0e-12;
    if data.q[0] > 1.0 {
        data.q[0] = 0.03;
    }
}

fn tau_for_target_acceleration<const DOF: usize>(
    model: &Model<DOF>,
    source: &Data<DOF>,
) -> nalgebra::SVector<f64, DOF> {
    let mut data = Data::<DOF>::default();
    data.q = source.q;
    data.q_dot = source.q_dot;
    data.q_ddot = source.q_ddot;
    rnea(model, &mut data, None);
    data.tau
}

fn rigid_inertia(mass: f64, com: Vector3) -> SpatialInertia<WorldFrame> {
    SpatialInertia::from_mass_com_inertia(mass, com, Matrix3::identity() * 0.01)
}

fn rz(theta: f64) -> Matrix3 {
    let (sin, cos) = theta.sin_cos();
    Matrix3::new(cos, -sin, 0.0, sin, cos, 0.0, 0.0, 0.0, 1.0)
}

fn serial_revolute_urdf(dof: usize) -> String {
    let mut urdf = String::from(r#"<robot name="kinetix_bench">"#);
    urdf.push_str(r#"<link name="base"/>"#);

    for i in 0..dof {
        urdf.push_str(&format!(
            r#"
<link name="link{i}">
  <inertial>
    <origin xyz="0.35 0.01 0" rpy="0 0 0"/>
    <mass value="{}"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>
"#,
            1.0 + 0.05 * i as f64
        ));
    }

    for i in 0..dof {
        let parent = if i == 0 {
            "base".to_string()
        } else {
            format!("link{}", i - 1)
        };
        let xyz = if i == 0 { "0 0 0" } else { "1 0 0" };
        urdf.push_str(&format!(
            r#"
<joint name="joint{i}" type="revolute">
  <parent link="{parent}"/>
  <child link="link{i}"/>
  <origin xyz="{xyz}" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
</joint>
"#
        ));
    }

    urdf.push_str("</robot>");
    urdf
}
