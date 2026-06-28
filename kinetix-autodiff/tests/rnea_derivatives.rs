use kinetix_autodiff::compute_rnea_derivatives;
use kinetix_core::{Data, JointType, Model, rnea};
use kinetix_spatial::{Matrix3, SpatialInertia, SpatialTransform, Vector3, WorldFrame};
use nalgebra::SVector;

fn rigid_inertia(mass: f64, com: Vector3) -> SpatialInertia<WorldFrame> {
    SpatialInertia::from_mass_com_inertia(mass, com, Matrix3::identity() * 0.01)
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

fn tau(
    model: &Model<2>,
    q: &SVector<f64, 2>,
    q_dot: &SVector<f64, 2>,
    q_ddot: &SVector<f64, 2>,
) -> SVector<f64, 2> {
    let mut data = Data::<2>::default();
    data.q = *q;
    data.q_dot = *q_dot;
    data.q_ddot = *q_ddot;
    rnea(model, &mut data, None);
    data.tau
}

#[test]
fn rnea_derivatives_match_finite_difference() {
    let model = two_link_revolute_model();
    let q = SVector::<f64, 2>::new(0.31, -0.43);
    let q_dot = SVector::<f64, 2>::new(0.37, -0.22);
    let q_ddot = SVector::<f64, 2>::new(0.91, -0.34);

    let (dtau_dq, dtau_dq_dot) = compute_rnea_derivatives(&model, &q, &q_dot, &q_ddot);
    let base_tau = tau(&model, &q, &q_dot, &q_ddot);
    let eps = 1.0e-6;

    for col in 0..2 {
        let mut shifted_q = q;
        shifted_q[col] += eps;
        let finite_difference = (tau(&model, &shifted_q, &q_dot, &q_ddot) - base_tau) / eps;
        for row in 0..2 {
            let error = (dtau_dq[(row, col)] - finite_difference[row]).abs();
            assert!(
                error < 1.0e-5,
                "dtau/dq[{row},{col}] ad={} fd={} error={error}",
                dtau_dq[(row, col)],
                finite_difference[row],
            );
        }
    }

    for col in 0..2 {
        let mut shifted_q_dot = q_dot;
        shifted_q_dot[col] += eps;
        let finite_difference = (tau(&model, &q, &shifted_q_dot, &q_ddot) - base_tau) / eps;
        for row in 0..2 {
            let error = (dtau_dq_dot[(row, col)] - finite_difference[row]).abs();
            assert!(
                error < 1.0e-5,
                "dtau/dq_dot[{row},{col}] ad={} fd={} error={error}",
                dtau_dq_dot[(row, col)],
                finite_difference[row],
            );
        }
    }
}
