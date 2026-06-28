use core::ops::{Add, Mul, Neg, Sub};

use crate::frames::{Frame, FrameMarker};
use crate::scalar::KScalar;
use crate::{Vector3, Vector6};

/// Spatial force vector expressed in frame `F`.
///
/// The component order is angular-first:
/// `[n_x, n_y, n_z, f_x, f_y, f_z]`, where `n` is torque and `f` is
/// linear force.
#[derive(Debug, PartialEq)]
pub struct SpatialForce<F: Frame, T: KScalar = f64> {
    torque: Vector3<T>,
    force: Vector3<T>,
    frame: FrameMarker<F>,
}

impl<F: Frame, T: KScalar> Clone for SpatialForce<F, T> {
    #[inline]
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame, T: KScalar> Copy for SpatialForce<F, T> {}

impl<F: Frame, T: KScalar> SpatialForce<F, T> {
    /// Creates a spatial force from torque and linear force components.
    #[inline]
    pub fn new(torque: Vector3<T>, force: Vector3<T>) -> Self {
        Self {
            torque,
            force,
            frame: FrameMarker::default(),
        }
    }

    /// Creates a zero spatial force.
    #[inline]
    pub fn zeros() -> Self {
        Self::new(
            Vector3::from_element(T::zero()),
            Vector3::from_element(T::zero()),
        )
    }

    /// Creates a spatial force from a torque-first six-vector.
    #[inline]
    pub fn from_vector(vector: Vector6<T>) -> Self {
        Self::new(
            Vector3::new(vector[0], vector[1], vector[2]),
            Vector3::new(vector[3], vector[4], vector[5]),
        )
    }

    /// Returns the torque component.
    #[inline]
    pub fn torque(&self) -> &Vector3<T> {
        &self.torque
    }

    /// Returns the linear force component.
    #[inline]
    pub fn force(&self) -> &Vector3<T> {
        &self.force
    }

    /// Returns a torque-first six-vector copy.
    #[inline]
    pub fn to_vector(&self) -> Vector6<T> {
        Vector6::new(
            self.torque[0],
            self.torque[1],
            self.torque[2],
            self.force[0],
            self.force[1],
            self.force[2],
        )
    }
}

impl<F: Frame, T: KScalar> Add for SpatialForce<F, T> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.torque + rhs.torque, self.force + rhs.force)
    }
}

impl<F: Frame, T: KScalar> Add<&Self> for SpatialForce<F, T> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: &Self) -> Self::Output {
        self + *rhs
    }
}

impl<F: Frame, T: KScalar> Add<SpatialForce<F, T>> for &SpatialForce<F, T> {
    type Output = SpatialForce<F, T>;

    #[inline]
    fn add(self, rhs: SpatialForce<F, T>) -> Self::Output {
        *self + rhs
    }
}

impl<F: Frame, T: KScalar> Add for &SpatialForce<F, T> {
    type Output = SpatialForce<F, T>;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        *self + *rhs
    }
}

impl<F: Frame, T: KScalar> Sub for SpatialForce<F, T> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.torque - rhs.torque, self.force - rhs.force)
    }
}

impl<F: Frame, T: KScalar> Sub<&Self> for SpatialForce<F, T> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: &Self) -> Self::Output {
        self - *rhs
    }
}

impl<F: Frame, T: KScalar> Sub<SpatialForce<F, T>> for &SpatialForce<F, T> {
    type Output = SpatialForce<F, T>;

    #[inline]
    fn sub(self, rhs: SpatialForce<F, T>) -> Self::Output {
        *self - rhs
    }
}

impl<F: Frame, T: KScalar> Sub for &SpatialForce<F, T> {
    type Output = SpatialForce<F, T>;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        *self - *rhs
    }
}

impl<F: Frame, T: KScalar> Neg for SpatialForce<F, T> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self::new(-self.torque, -self.force)
    }
}

impl<F: Frame, T: KScalar> Mul<T> for SpatialForce<F, T> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: T) -> Self::Output {
        Self::new(self.torque * rhs, self.force * rhs)
    }
}
