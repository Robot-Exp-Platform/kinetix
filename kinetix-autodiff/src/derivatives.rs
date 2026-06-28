use core::array;

use kinetix_core::{Model, TypedData, TypedModel, rnea};
use kinetix_spatial::{Matrix3, Matrix6, SpatialInertia, SpatialTransform, Vector3};
use nalgebra::{SMatrix, SVector};

use crate::Dual;

/// Lifts an `f64` model into the `Dual` scalar domain with zero tangents.
///
/// The model is immutable algorithm input, so all of its physical parameters
/// are constants for the derivative propagation.
pub fn lift_model<const DOF: usize>(model: &Model<DOF>) -> Model<DOF, Dual> {
    TypedModel::new(
        model.parents,
        array::from_fn(|i| {
            SpatialTransform::new(
                lift_matrix3(model.inertia_transforms[i].rotation()),
                lift_vector3(model.inertia_transforms[i].translation()),
            )
        }),
        array::from_fn(|i| SpatialInertia::from_matrix(lift_matrix6(model.inertias[i].matrix()))),
        model.joint_types,
        lift_vector3(&model.gravity),
    )
}

/// Computes inverse-dynamics Jacobians with respect to `q` and `q_dot`.
///
/// The same generic RNEA implementation used by `kinetix-core` is executed
/// once per seeded input direction. No heap allocation is needed; all matrices
/// are statically sized by `DOF`.
pub fn compute_rnea_derivatives<const DOF: usize>(
    model: &Model<DOF>,
    q: &SVector<f64, DOF>,
    q_dot: &SVector<f64, DOF>,
    q_ddot: &SVector<f64, DOF>,
) -> (SMatrix<f64, DOF, DOF>, SMatrix<f64, DOF, DOF>) {
    let dual_model = lift_model(model);
    let mut dtau_dq = SMatrix::<f64, DOF, DOF>::zeros();
    let mut dtau_dq_dot = SMatrix::<f64, DOF, DOF>::zeros();

    for seed in 0..DOF {
        let mut data = seeded_data(q, q_dot, q_ddot, Some(seed), None);
        rnea(&dual_model, &mut data, None);
        for row in 0..DOF {
            dtau_dq[(row, seed)] = data.tau[row].derivative();
        }
    }

    for seed in 0..DOF {
        let mut data = seeded_data(q, q_dot, q_ddot, None, Some(seed));
        rnea(&dual_model, &mut data, None);
        for row in 0..DOF {
            dtau_dq_dot[(row, seed)] = data.tau[row].derivative();
        }
    }

    (dtau_dq, dtau_dq_dot)
}

fn seeded_data<const DOF: usize>(
    q: &SVector<f64, DOF>,
    q_dot: &SVector<f64, DOF>,
    q_ddot: &SVector<f64, DOF>,
    q_seed: Option<usize>,
    q_dot_seed: Option<usize>,
) -> TypedData<Dual, DOF> {
    let mut data = TypedData::<Dual, DOF>::default();
    for i in 0..DOF {
        data.q[i] = Dual::variable(q[i], seed_value(q_seed, i));
        data.q_dot[i] = Dual::variable(q_dot[i], seed_value(q_dot_seed, i));
        data.q_ddot[i] = Dual::constant(q_ddot[i]);
    }
    data
}

#[inline]
fn seed_value(seed: Option<usize>, index: usize) -> f64 {
    if seed == Some(index) { 1.0 } else { 0.0 }
}

fn lift_vector3(vector: &Vector3) -> Vector3<Dual> {
    Vector3::new(
        Dual::constant(vector[0]),
        Dual::constant(vector[1]),
        Dual::constant(vector[2]),
    )
}

fn lift_matrix3(matrix: &Matrix3) -> Matrix3<Dual> {
    Matrix3::from_fn(|row, col| Dual::constant(matrix[(row, col)]))
}

fn lift_matrix6(matrix: &Matrix6) -> Matrix6<Dual> {
    Matrix6::from_fn(|row, col| Dual::constant(matrix[(row, col)]))
}
