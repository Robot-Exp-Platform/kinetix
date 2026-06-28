use core::ops::{Add, Mul, Neg, Sub};

use crate::force::SpatialForce;
use crate::frames::{Frame, FrameMarker};
use crate::scalar::KScalar;
use crate::{Vector3, Vector6};

/// Spatial motion vector expressed in frame `F`.
///
/// The component order is Featherstone-style angular-first:
/// `[omega_x, omega_y, omega_z, v_x, v_y, v_z]`.
///
/// Frame safety example:
///
/// ```compile_fail
/// use kinetix_spatial::{LinkFrame, SpatialMotion, WorldFrame};
///
/// let world = SpatialMotion::<WorldFrame>::zeros();
/// let link = SpatialMotion::<LinkFrame<1>>::zeros();
///
/// let _invalid = world + link;
/// ```
#[derive(Debug, PartialEq)]
pub struct SpatialMotion<F: Frame, T: KScalar = f64> {
    angular: Vector3<T>,
    linear: Vector3<T>,
    frame: FrameMarker<F>,
}

impl<F: Frame, T: KScalar> Clone for SpatialMotion<F, T> {
    #[inline]
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame, T: KScalar> Copy for SpatialMotion<F, T> {}

impl<F: Frame, T: KScalar> SpatialMotion<F, T> {
    /// Creates a spatial motion from angular and linear components.
    #[inline]
    pub fn new(angular: Vector3<T>, linear: Vector3<T>) -> Self {
        Self {
            angular,
            linear,
            frame: FrameMarker::default(),
        }
    }

    /// Creates a zero spatial motion.
    #[inline]
    pub fn zeros() -> Self {
        Self::new(
            Vector3::from_element(T::zero()),
            Vector3::from_element(T::zero()),
        )
    }

    /// Creates a spatial motion from an angular-first six-vector.
    #[inline]
    pub fn from_vector(vector: Vector6<T>) -> Self {
        Self::new(
            Vector3::new(vector[0], vector[1], vector[2]),
            Vector3::new(vector[3], vector[4], vector[5]),
        )
    }

    /// Returns the angular component.
    #[inline]
    pub fn angular(&self) -> &Vector3<T> {
        &self.angular
    }

    /// Returns the linear component.
    #[inline]
    pub fn linear(&self) -> &Vector3<T> {
        &self.linear
    }

    /// Returns an angular-first six-vector copy.
    #[inline]
    pub fn to_vector(&self) -> Vector6<T> {
        Vector6::new(
            self.angular[0],
            self.angular[1],
            self.angular[2],
            self.linear[0],
            self.linear[1],
            self.linear[2],
        )
    }

    /// Spatial motion cross product `self x rhs`.
    ///
    /// For `self = [omega1; v1]` and `rhs = [omega2; v2]`, this computes:
    /// `[omega1 x omega2; omega1 x v2 + v1 x omega2]`.
    #[inline]
    pub fn cross_motion(&self, rhs: &SpatialMotion<F, T>) -> SpatialMotion<F, T> {
        SpatialMotion::new(
            cross3(&self.angular, &rhs.angular),
            cross3(&self.angular, &rhs.linear) + cross3(&self.linear, &rhs.angular),
        )
    }

    /// Spatial force cross product `self x* rhs`.
    ///
    /// For `self = [omega; v]` and `rhs = [n; f]`, this computes:
    /// `[omega x n + v x f; omega x f]`.
    #[inline]
    pub fn cross_force(&self, rhs: &SpatialForce<F, T>) -> SpatialForce<F, T> {
        SpatialForce::new(
            cross3(&self.angular, rhs.torque()) + cross3(&self.linear, rhs.force()),
            cross3(&self.angular, rhs.force()),
        )
    }
}

impl<F: Frame, T: KScalar> Add for SpatialMotion<F, T> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.angular + rhs.angular, self.linear + rhs.linear)
    }
}

impl<F: Frame, T: KScalar> Add<&Self> for SpatialMotion<F, T> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: &Self) -> Self::Output {
        self + *rhs
    }
}

impl<F: Frame, T: KScalar> Add<SpatialMotion<F, T>> for &SpatialMotion<F, T> {
    type Output = SpatialMotion<F, T>;

    #[inline]
    fn add(self, rhs: SpatialMotion<F, T>) -> Self::Output {
        *self + rhs
    }
}

impl<F: Frame, T: KScalar> Add for &SpatialMotion<F, T> {
    type Output = SpatialMotion<F, T>;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        *self + *rhs
    }
}

impl<F: Frame, T: KScalar> Sub for SpatialMotion<F, T> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.angular - rhs.angular, self.linear - rhs.linear)
    }
}

impl<F: Frame, T: KScalar> Sub<&Self> for SpatialMotion<F, T> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: &Self) -> Self::Output {
        self - *rhs
    }
}

impl<F: Frame, T: KScalar> Sub<SpatialMotion<F, T>> for &SpatialMotion<F, T> {
    type Output = SpatialMotion<F, T>;

    #[inline]
    fn sub(self, rhs: SpatialMotion<F, T>) -> Self::Output {
        *self - rhs
    }
}

impl<F: Frame, T: KScalar> Sub for &SpatialMotion<F, T> {
    type Output = SpatialMotion<F, T>;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        *self - *rhs
    }
}

impl<F: Frame, T: KScalar> Neg for SpatialMotion<F, T> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self::new(-self.angular, -self.linear)
    }
}

impl<F: Frame, T: KScalar> Mul<T> for SpatialMotion<F, T> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: T) -> Self::Output {
        Self::new(self.angular * rhs, self.linear * rhs)
    }
}

#[inline]
pub(crate) fn cross3<T: KScalar>(lhs: &Vector3<T>, rhs: &Vector3<T>) -> Vector3<T> {
    Vector3::new(
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WorldFrame;

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

    #[test]
    fn cross_motion_matches_featherstone_formula() {
        let v1 =
            SpatialMotion::<WorldFrame>::from_vector(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        let v2 =
            SpatialMotion::<WorldFrame>::from_vector(Vector6::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0));

        let result = v1.cross_motion(&v2).to_vector();

        assert_vector6_close(result, Vector6::new(-6.0, 12.0, -6.0, -12.0, 24.0, -12.0));
    }

    #[test]
    fn cross_force_matches_dual_formula() {
        let motion =
            SpatialMotion::<WorldFrame>::from_vector(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        let force =
            SpatialForce::<WorldFrame>::from_vector(Vector6::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0));

        let result = motion.cross_force(&force).to_vector();

        assert_vector6_close(result, Vector6::new(-12.0, 24.0, -12.0, -9.0, 18.0, -9.0));
    }
}
