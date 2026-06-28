use crate::force::SpatialForce;
use crate::frames::{Frame, FrameMarker};
use crate::motion::SpatialMotion;
use crate::scalar::KScalar;
use crate::transform::skew;
use crate::{Matrix3, Matrix6, Vector3};

/// Spatial rigid-body inertia expressed in frame `F`.
///
/// The matrix maps an angular-first [`SpatialMotion`] to a torque-first
/// [`SpatialForce`].
#[derive(Debug, PartialEq)]
pub struct SpatialInertia<F: Frame, T: KScalar = f64> {
    matrix: Matrix6<T>,
    frame: FrameMarker<F>,
}

impl<F: Frame, T: KScalar> Clone for SpatialInertia<F, T> {
    #[inline]
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame, T: KScalar> Copy for SpatialInertia<F, T> {}

impl<F: Frame, T: KScalar> SpatialInertia<F, T> {
    /// Creates an inertia from a full 6x6 spatial inertia matrix.
    #[inline]
    pub fn from_matrix(matrix: Matrix6<T>) -> Self {
        Self {
            matrix,
            frame: FrameMarker::default(),
        }
    }

    /// Creates a zero inertia.
    #[inline]
    pub fn zeros() -> Self {
        Self::from_matrix(Matrix6::zeros())
    }

    /// Creates a rigid-body spatial inertia from mass, center of mass, and
    /// rotational inertia about the center of mass.
    pub fn from_mass_com_inertia(
        mass: T,
        center_of_mass: Vector3<T>,
        inertia_at_com: Matrix3<T>,
    ) -> Self {
        let c = skew(center_of_mass);
        let mut matrix = Matrix6::zeros();

        matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(inertia_at_com + (c * c.transpose()) * mass));
        matrix.fixed_view_mut::<3, 3>(0, 3).copy_from(&(c * mass));
        matrix
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-(c * mass)));
        matrix
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(Matrix3::identity() * mass));

        Self::from_matrix(matrix)
    }

    /// Returns the underlying 6x6 matrix.
    #[inline]
    pub fn matrix(&self) -> &Matrix6<T> {
        &self.matrix
    }

    /// Applies the inertia to a spatial motion.
    #[inline]
    pub fn apply_motion(&self, motion: &SpatialMotion<F, T>) -> SpatialForce<F, T> {
        SpatialForce::from_vector(self.matrix * motion.to_vector())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WorldFrame;

    #[test]
    fn inertia_maps_motion_to_force_in_same_frame() {
        let inertia = SpatialInertia::<WorldFrame>::from_matrix(Matrix6::identity());
        let motion = SpatialMotion::<WorldFrame>::from_vector(crate::Vector6::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
        ));

        assert_eq!(
            inertia.apply_motion(&motion).to_vector(),
            motion.to_vector()
        );
    }
}
