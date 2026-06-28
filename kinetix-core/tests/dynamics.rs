use kinetix_core::{Data, JointType, Model, aba, crba, rnea};
use kinetix_spatial::{Matrix3, SpatialInertia, SpatialTransform, Vector3, WorldFrame};

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual={actual}, expected={expected}, tolerance={tolerance}"
    );
}

fn rigid_inertia(mass: f64, com: Vector3) -> SpatialInertia<WorldFrame> {
    SpatialInertia::from_mass_com_inertia(mass, com, Matrix3::identity() * 0.01)
}

fn prismatic_gravity_model() -> Model<2> {
    Model::new(
        [0, 0],
        [SpatialTransform::identity(), SpatialTransform::identity()],
        [
            rigid_inertia(2.0, Vector3::zeros()),
            rigid_inertia(3.0, Vector3::zeros()),
        ],
        [JointType::PrismaticZ, JointType::PrismaticZ],
        Vector3::new(0.0, 0.0, -9.81),
    )
}

fn two_link_revolute_model() -> Model<2> {
    Model::new(
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
    )
}

#[test]
fn rnea_static_gravity_compensation_for_prismatic_chain() {
    let model = prismatic_gravity_model();
    let mut data = Data::<2>::default();

    rnea(&model, &mut data, None);

    assert_close(data.tau[0], (2.0 + 3.0) * 9.81, 1.0e-12);
    assert_close(data.tau[1], 3.0 * 9.81, 1.0e-12);
}

#[test]
fn crba_is_symmetric_positive_definite_for_two_link_arm() {
    let model = two_link_revolute_model();
    let mut data = Data::<2>::default();
    data.q[0] = 0.4;
    data.q[1] = -0.7;

    crba(&model, &mut data);

    assert_close(data.m_matrix[(0, 1)], data.m_matrix[(1, 0)], 1.0e-12);

    let leading_minor = data.m_matrix[(0, 0)];
    let determinant = data.m_matrix[(0, 0)] * data.m_matrix[(1, 1)]
        - data.m_matrix[(0, 1)] * data.m_matrix[(1, 0)];

    assert!(leading_minor > 1.0e-12, "leading minor={leading_minor}");
    assert!(determinant > 1.0e-12, "determinant={determinant}");
}

#[test]
fn rnea_and_aba_round_trip_for_two_link_arm() {
    let model = two_link_revolute_model();
    let mut data = Data::<2>::default();

    data.q[0] = 0.3;
    data.q[1] = -0.8;
    data.q_dot[0] = 0.7;
    data.q_dot[1] = -0.2;
    data.q_ddot[0] = 1.1;
    data.q_ddot[1] = -0.4;

    let target_q_ddot = data.q_ddot;

    rnea(&model, &mut data, None);
    data.q_ddot.fill(0.0);

    aba(&model, &mut data, None);

    assert_close(data.q_ddot[0], target_q_ddot[0], 1.0e-10);
    assert_close(data.q_ddot[1], target_q_ddot[1], 1.0e-10);
}
