use kinetix_spatial::{
    KScalar, Matrix6, SpatialForce, SpatialInertia, SpatialMotion, SpatialTransform, Vector6,
    WorldFrame,
};
use nalgebra::{SMatrix, SVector};

/// Zero-allocation algorithm workspace.
///
/// `Data` is owned by one control or planning thread and reused across calls.
/// All arrays are fixed-size and live wherever the user stores `Data`.
pub struct TypedData<T: KScalar, const DOF: usize> {
    /// Joint configuration.
    pub q: SVector<T, DOF>,
    /// Joint velocity.
    pub q_dot: SVector<T, DOF>,
    /// Joint acceleration.
    pub q_ddot: SVector<T, DOF>,
    /// Joint torque or force.
    pub tau: SVector<T, DOF>,
    /// Generalized inertia matrix filled by CRBA.
    pub m_matrix: SMatrix<T, DOF, DOF>,

    /// Body spatial velocities.
    pub v: [SpatialMotion<WorldFrame, T>; DOF],
    /// Body spatial accelerations.
    pub a: [SpatialMotion<WorldFrame, T>; DOF],
    /// Body spatial forces.
    pub f: [SpatialForce<WorldFrame, T>; DOF],
    /// Runtime parent-to-body transforms.
    pub li_x_p: [SpatialTransform<WorldFrame, WorldFrame, T>; DOF],

    /// ABA articulated inertias.
    pub i_a: [SpatialInertia<WorldFrame, T>; DOF],
    /// ABA bias forces.
    pub p_a: [SpatialForce<WorldFrame, T>; DOF],

    pub(crate) c: [SpatialMotion<WorldFrame, T>; DOF],
    pub(crate) u: [SpatialForce<WorldFrame, T>; DOF],
    pub(crate) d: SVector<T, DOF>,
    pub(crate) joint_u: SVector<T, DOF>,
}

impl<T: KScalar, const DOF: usize> Default for TypedData<T, DOF> {
    fn default() -> Self {
        Self {
            q: SVector::from_element(T::zero()),
            q_dot: SVector::from_element(T::zero()),
            q_ddot: SVector::from_element(T::zero()),
            tau: SVector::from_element(T::zero()),
            m_matrix: SMatrix::from_element(T::zero()),
            v: [SpatialMotion::zeros(); DOF],
            a: [SpatialMotion::zeros(); DOF],
            f: [SpatialForce::zeros(); DOF],
            li_x_p: [SpatialTransform::identity(); DOF],
            i_a: [SpatialInertia::zeros(); DOF],
            p_a: [SpatialForce::zeros(); DOF],
            c: [SpatialMotion::zeros(); DOF],
            u: [SpatialForce::zeros(); DOF],
            d: SVector::from_element(T::zero()),
            joint_u: SVector::from_element(T::zero()),
        }
    }
}

#[inline]
pub(crate) fn motion_dot_force<T: KScalar>(
    motion: &SpatialMotion<WorldFrame, T>,
    force: &SpatialForce<WorldFrame, T>,
) -> T {
    motion.angular()[0] * force.torque()[0]
        + motion.angular()[1] * force.torque()[1]
        + motion.angular()[2] * force.torque()[2]
        + motion.linear()[0] * force.force()[0]
        + motion.linear()[1] * force.force()[1]
        + motion.linear()[2] * force.force()[2]
}

#[inline]
pub(crate) fn inertia_from_matrix<T: KScalar>(matrix: Matrix6<T>) -> SpatialInertia<WorldFrame, T> {
    SpatialInertia::from_matrix(matrix)
}

#[inline]
pub(crate) fn force_from_vector<T: KScalar>(vector: Vector6<T>) -> SpatialForce<WorldFrame, T> {
    SpatialForce::from_vector(vector)
}
