use kinetix_spatial::{KScalar, SpatialMotion, Vector3, WorldFrame};

/// Joint motion model specialized for the first core algorithms.
///
/// The first implementation intentionally supports only Z-axis one-DoF
/// primitives plus fixed joints. Their motion subspace is hard-coded, avoiding
/// heap-backed Jacobian columns or dynamic matrix products in the hot path.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum JointType {
    /// Rotation around the local Z axis.
    RevoluteZ,
    /// Translation along the local Z axis.
    PrismaticZ,
    /// No relative motion.
    Fixed,
}

impl JointType {
    /// Returns the joint motion subspace as an angular-first spatial vector.
    #[inline]
    pub fn motion_subspace<T: KScalar>(self) -> SpatialMotion<WorldFrame, T> {
        let zero = T::zero();
        let one = T::one();

        match self {
            Self::RevoluteZ => SpatialMotion::new(
                Vector3::new(zero, zero, one),
                Vector3::from_element(T::zero()),
            ),
            Self::PrismaticZ => SpatialMotion::new(
                Vector3::from_element(T::zero()),
                Vector3::new(zero, zero, one),
            ),
            Self::Fixed => SpatialMotion::zeros(),
        }
    }

    /// Returns whether this joint contributes an active scalar coordinate.
    #[inline]
    pub fn is_active(self) -> bool {
        !matches!(self, Self::Fixed)
    }
}
