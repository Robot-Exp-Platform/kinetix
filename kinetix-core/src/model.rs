use kinetix_spatial::{KScalar, Matrix3, SpatialInertia, SpatialTransform, Vector3, WorldFrame};

use crate::joints::JointType;

/// Immutable robot model stored in arena-like flat arrays.
///
/// The arena order is topological: for every non-root body, `parents[i] < i`.
/// Slot `0` is the root body and uses `parents[0] == 0`.
pub struct TypedModel<T: KScalar, const DOF: usize> {
    /// Parent body index. `parents[0]` is ignored and conventionally equals `0`.
    pub parents: [usize; DOF],
    /// Fixed transform from parent to body at `q = 0`.
    pub inertia_transforms: [SpatialTransform<WorldFrame, WorldFrame, T>; DOF],
    /// Spatial rigid-body inertia for each body.
    pub inertias: [SpatialInertia<WorldFrame, T>; DOF],
    /// Joint type connecting each body to its parent.
    pub joint_types: [JointType; DOF],
    /// Linear gravity acceleration expressed in the base frame.
    pub gravity: Vector3<T>,
}

impl<T: KScalar, const DOF: usize> TypedModel<T, DOF> {
    /// Creates a model with explicit gravity.
    #[inline]
    pub fn new(
        parents: [usize; DOF],
        inertia_transforms: [SpatialTransform<WorldFrame, WorldFrame, T>; DOF],
        inertias: [SpatialInertia<WorldFrame, T>; DOF],
        joint_types: [JointType; DOF],
        gravity: Vector3<T>,
    ) -> Self {
        Self {
            parents,
            inertia_transforms,
            inertias,
            joint_types,
            gravity,
        }
    }

    /// Creates a model with zero gravity.
    #[inline]
    pub fn without_gravity(
        parents: [usize; DOF],
        inertia_transforms: [SpatialTransform<WorldFrame, WorldFrame, T>; DOF],
        inertias: [SpatialInertia<WorldFrame, T>; DOF],
        joint_types: [JointType; DOF],
    ) -> Self {
        Self::new(
            parents,
            inertia_transforms,
            inertias,
            joint_types,
            Vector3::from_element(T::zero()),
        )
    }

    /// Checks the arena topology invariant used by the recursive algorithms.
    #[inline]
    pub fn has_valid_topology(&self) -> bool {
        if DOF == 0 {
            return true;
        }

        self.parents[0] == 0 && (1..DOF).all(|i| self.parents[i] < i)
    }
}

impl<T: KScalar, const DOF: usize> Default for TypedModel<T, DOF> {
    fn default() -> Self {
        Self {
            parents: [0; DOF],
            inertia_transforms: [SpatialTransform::identity(); DOF],
            inertias: [SpatialInertia::zeros(); DOF],
            joint_types: [JointType::Fixed; DOF],
            gravity: Vector3::from_element(T::zero()),
        }
    }
}

/// Creates a rotation around Z.
#[inline]
pub(crate) fn rz<T: KScalar>(theta: T) -> Matrix3<T> {
    let (sin, cos) = theta.sin_cos();
    let zero = T::zero();
    Matrix3::new(cos, -sin, zero, sin, cos, zero, zero, zero, T::one())
}
