use kinetix_spatial::{
    KScalar, Matrix3, Matrix6, SpatialForce, SpatialInertia, SpatialMotion, SpatialTransform,
    Vector3, WorldFrame,
};

use crate::data::inertia_from_matrix;
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
    let rotation_t = x_parent_to_child.rotation().transpose();
    let translation = x_parent_to_child.translation();
    let parent_force = rotation_t * child_force.force();
    let parent_torque =
        rotation_t * (child_force.torque() - cross3(translation, child_force.force()));

    SpatialForce::new(parent_torque, parent_force)
}

#[inline]
pub(crate) fn transform_inertia_to_parent<T: KScalar>(
    x_parent_to_child: &SpatialTransform<WorldFrame, WorldFrame, T>,
    child_inertia: &SpatialInertia<WorldFrame, T>,
) -> SpatialInertia<WorldFrame, T> {
    let rotation = *x_parent_to_child.rotation();
    let rotation_t = rotation.transpose();
    let e = skew(x_parent_to_child.translation()) * rotation;
    let e_t = e.transpose();
    let inertia = child_inertia.matrix();

    let angular_angular = block3(inertia, 0, 0);
    let angular_linear = block3(inertia, 0, 3);
    let linear_angular = block3(inertia, 3, 0);
    let linear_linear = block3(inertia, 3, 3);

    let angular_columns = angular_angular * rotation + angular_linear * e;
    let linear_columns = linear_angular * rotation + linear_linear * e;
    let angular_linear_columns = angular_linear * rotation;
    let linear_linear_columns = linear_linear * rotation;

    let mut parent_inertia = Matrix6::zeros();
    parent_inertia
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(rotation_t * angular_columns + e_t * linear_columns));
    parent_inertia
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(rotation_t * angular_linear_columns + e_t * linear_linear_columns));
    parent_inertia
        .fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&(rotation_t * linear_columns));
    parent_inertia
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&(rotation_t * linear_linear_columns));

    inertia_from_matrix(parent_inertia)
}

#[inline]
pub(crate) fn joint_project_force<T: KScalar>(
    joint_type: JointType,
    force: &SpatialForce<WorldFrame, T>,
) -> T {
    match joint_type {
        JointType::RevoluteZ => force.torque()[2],
        JointType::PrismaticZ => force.force()[2],
        JointType::Fixed => T::zero(),
    }
}

#[inline]
pub(crate) fn joint_inertia_column<T: KScalar>(
    joint_type: JointType,
    inertia: &SpatialInertia<WorldFrame, T>,
) -> SpatialForce<WorldFrame, T> {
    let column = match joint_type {
        JointType::RevoluteZ => 2,
        JointType::PrismaticZ => 5,
        JointType::Fixed => return SpatialForce::zeros(),
    };
    let matrix = inertia.matrix();

    SpatialForce::new(
        Vector3::new(
            matrix[(0, column)],
            matrix[(1, column)],
            matrix[(2, column)],
        ),
        Vector3::new(
            matrix[(3, column)],
            matrix[(4, column)],
            matrix[(5, column)],
        ),
    )
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

#[inline]
fn cross3<T: KScalar>(lhs: &Vector3<T>, rhs: &Vector3<T>) -> Vector3<T> {
    Vector3::new(
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    )
}

#[inline]
fn skew<T: KScalar>(vector: &Vector3<T>) -> Matrix3<T> {
    Matrix3::new(
        T::zero(),
        -vector[2],
        vector[1],
        vector[2],
        T::zero(),
        -vector[0],
        -vector[1],
        vector[0],
        T::zero(),
    )
}

#[inline]
fn block3<T: KScalar>(matrix: &Matrix6<T>, row: usize, col: usize) -> Matrix3<T> {
    Matrix3::new(
        matrix[(row, col)],
        matrix[(row, col + 1)],
        matrix[(row, col + 2)],
        matrix[(row + 1, col)],
        matrix[(row + 1, col + 1)],
        matrix[(row + 1, col + 2)],
        matrix[(row + 2, col)],
        matrix[(row + 2, col + 1)],
        matrix[(row + 2, col + 2)],
    )
}
