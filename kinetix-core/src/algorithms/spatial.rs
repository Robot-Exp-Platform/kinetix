use kinetix_spatial::{
    KScalar, Matrix3, SpatialForce, SpatialInertia, SpatialMotion, SpatialTransform, Vector3,
    WorldFrame,
};

use crate::data::{force_from_vector, inertia_from_matrix};
use crate::joints::JointType;
use crate::model::rz;

#[inline]
pub(crate) fn joint_transform<T: KScalar>(
    joint_type: JointType,
    q: T,
) -> SpatialTransform<WorldFrame, WorldFrame, T> {
    // Hot-path recursion stores the parent motion expressed in the child frame,
    // so the joint placement is used through its inverse action.
    let q_inv = -q;
    match joint_type {
        JointType::RevoluteZ => SpatialTransform::new(rz(q_inv), Vector3::from_element(T::zero())),
        JointType::PrismaticZ => SpatialTransform::new(
            Matrix3::identity(),
            Vector3::new(T::zero(), T::zero(), q_inv),
        ),
        JointType::Fixed => SpatialTransform::identity(),
    }
}

#[inline]
pub(crate) fn joint_motion<T: KScalar>(
    joint_type: JointType,
    value: T,
) -> SpatialMotion<WorldFrame, T> {
    joint_type.motion_subspace() * value
}

#[inline]
pub(crate) fn transform_force_to_parent<T: KScalar>(
    x_parent_to_child: &SpatialTransform<WorldFrame, WorldFrame, T>,
    child_force: &SpatialForce<WorldFrame, T>,
) -> SpatialForce<WorldFrame, T> {
    force_from_vector(x_parent_to_child.motion_matrix().transpose() * child_force.to_vector())
}

#[inline]
pub(crate) fn transform_inertia_to_parent<T: KScalar>(
    x_parent_to_child: &SpatialTransform<WorldFrame, WorldFrame, T>,
    child_inertia: &SpatialInertia<WorldFrame, T>,
) -> SpatialInertia<WorldFrame, T> {
    let x = x_parent_to_child.motion_matrix();
    inertia_from_matrix(x.transpose() * child_inertia.matrix() * x)
}

#[inline]
pub(crate) fn subtract_rank_one<T: KScalar>(
    inertia: &SpatialInertia<WorldFrame, T>,
    u: &SpatialForce<WorldFrame, T>,
    d: T,
) -> SpatialInertia<WorldFrame, T> {
    let uv = u.to_vector();
    inertia_from_matrix(inertia.matrix() - (uv * uv.transpose()) * (T::one() / d))
}

#[inline]
pub(crate) fn add_inertia<T: KScalar>(
    lhs: SpatialInertia<WorldFrame, T>,
    rhs: SpatialInertia<WorldFrame, T>,
) -> SpatialInertia<WorldFrame, T> {
    inertia_from_matrix(lhs.matrix() + rhs.matrix())
}

#[inline]
pub(crate) fn inertia_mul_motion<T: KScalar>(
    inertia: &SpatialInertia<WorldFrame, T>,
    motion: &SpatialMotion<WorldFrame, T>,
) -> SpatialForce<WorldFrame, T> {
    inertia.apply_motion(motion)
}
