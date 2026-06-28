use core::marker::PhantomData;
use core::ops::Mul;

use crate::force::SpatialForce;
use crate::frames::Frame;
use crate::motion::SpatialMotion;
use crate::motion::cross3;
use crate::scalar::KScalar;
use crate::{Matrix3, Matrix6, Vector3};

/// SE(3) spatial transform from `From` to `To`.
///
/// `rotation` maps 3D vectors expressed in `From` into `To`.
/// `translation` is the displacement term used by the spatial adjoint formula,
/// expressed in `To`.
#[derive(Debug, PartialEq)]
pub struct SpatialTransform<From: Frame, To: Frame, T: KScalar = f64> {
    rotation: Matrix3<T>,
    translation: Vector3<T>,
    frames: PhantomData<fn(From) -> To>,
}

impl<From: Frame, To: Frame, T: KScalar> Clone for SpatialTransform<From, To, T> {
    #[inline]
    fn clone(&self) -> Self {
        *self
    }
}

impl<From: Frame, To: Frame, T: KScalar> Copy for SpatialTransform<From, To, T> {}

impl<From: Frame, To: Frame, T: KScalar> SpatialTransform<From, To, T> {
    /// Creates a spatial transform from a rotation and translation.
    #[inline]
    pub fn new(rotation: Matrix3<T>, translation: Vector3<T>) -> Self {
        Self {
            rotation,
            translation,
            frames: PhantomData,
        }
    }

    /// Creates a pure identity transform. This is most useful when `From = To`.
    #[inline]
    pub fn identity() -> Self {
        Self::new(Matrix3::identity(), Vector3::from_element(T::zero()))
    }

    /// Returns the 3D rotation block.
    #[inline]
    pub fn rotation(&self) -> &Matrix3<T> {
        &self.rotation
    }

    /// Returns the translation vector.
    #[inline]
    pub fn translation(&self) -> &Vector3<T> {
        &self.translation
    }

    /// Applies the motion adjoint transform.
    ///
    /// For `m = [omega; v]`, this computes:
    /// `omega_to = R * omega_from`
    /// `v_to = R * v_from + p x omega_to`.
    #[inline]
    pub fn apply_motion(&self, motion: &SpatialMotion<From, T>) -> SpatialMotion<To, T> {
        let angular = self.rotation * motion.angular();
        let linear = self.rotation * motion.linear() + cross3(&self.translation, &angular);
        SpatialMotion::new(angular, linear)
    }

    /// Applies the force dual-adjoint transform.
    ///
    /// For `force = [n; f]`, this computes:
    /// `f_to = R * f_from`
    /// `n_to = R * n_from + p x f_to`.
    #[inline]
    pub fn apply_force(&self, force: &SpatialForce<From, T>) -> SpatialForce<To, T> {
        let linear_force = self.rotation * force.force();
        let torque = self.rotation * force.torque() + cross3(&self.translation, &linear_force);
        SpatialForce::new(torque, linear_force)
    }

    /// Returns the 6x6 motion adjoint matrix for diagnostics and tests.
    ///
    /// Hot-path algorithms should prefer `apply_motion`, which expands the 3D
    /// operations directly.
    pub fn motion_matrix(&self) -> Matrix6<T> {
        let mut matrix = Matrix6::zeros();
        matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&self.rotation);
        matrix
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(skew(self.translation) * self.rotation));
        matrix
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&self.rotation);
        matrix
    }

    /// Returns the 6x6 force dual-adjoint matrix for diagnostics and tests.
    ///
    /// Hot-path algorithms should prefer `apply_force`, which expands the 3D
    /// operations directly.
    pub fn force_matrix(&self) -> Matrix6<T> {
        let mut matrix = Matrix6::zeros();
        matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&self.rotation);
        matrix
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(skew(self.translation) * self.rotation));
        matrix
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&self.rotation);
        matrix
    }
}

impl<From: Frame, Mid: Frame, To: Frame, T: KScalar> Mul<SpatialTransform<From, Mid, T>>
    for SpatialTransform<Mid, To, T>
{
    type Output = SpatialTransform<From, To, T>;

    /// Composes transforms so that `(x2 * x1).apply_motion(m)` is equivalent to
    /// `x2.apply_motion(&x1.apply_motion(m))`.
    #[inline]
    fn mul(self, rhs: SpatialTransform<From, Mid, T>) -> Self::Output {
        SpatialTransform::new(
            self.rotation * rhs.rotation,
            self.rotation * rhs.translation + self.translation,
        )
    }
}

#[inline]
pub(crate) fn skew<T: KScalar>(vector: Vector3<T>) -> Matrix3<T> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{LinkFrame, Vector6, WorldFrame};

    fn assert_vector6_close(actual: Vector6, expected: Vector6) {
        for i in 0..6 {
            assert!(
                (actual[i] - expected[i]).abs() < 1.0e-12,
                "index {i}: actual={}, expected={}",
                actual[i],
                expected[i]
            );
        }
    }

    fn rot_z_90() -> Matrix3 {
        Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    }

    #[test]
    fn apply_motion_matches_expanded_formula() {
        let x = SpatialTransform::<WorldFrame, LinkFrame<1>>::new(
            rot_z_90(),
            Vector3::new(1.0, 0.0, 0.0),
        );
        let motion = SpatialMotion::<WorldFrame>::new(
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
        );

        let result = x.apply_motion(&motion);

        assert_vector6_close(
            result.to_vector(),
            Vector6::new(-2.0, 1.0, 3.0, -5.0, 1.0, 7.0),
        );
    }

    #[test]
    fn transform_composition_matches_sequential_application() {
        let x1 = SpatialTransform::<WorldFrame, LinkFrame<1>>::new(
            rot_z_90(),
            Vector3::new(1.0, 0.0, 0.0),
        );
        let x2 = SpatialTransform::<LinkFrame<1>, LinkFrame<2>>::new(
            Matrix3::identity(),
            Vector3::new(0.0, 2.0, 0.0),
        );
        let motion = SpatialMotion::<WorldFrame>::new(
            Vector3::new(0.5, -1.0, 2.0),
            Vector3::new(3.0, 4.0, -2.0),
        );

        let sequential = x2.apply_motion(&x1.apply_motion(&motion));
        let composed = (x2 * x1).apply_motion(&motion);

        assert_vector6_close(sequential.to_vector(), composed.to_vector());
    }
}
