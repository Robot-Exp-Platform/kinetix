use core::fmt::Debug;
use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use num_traits::{One, Zero};

/// Minimal scalar contract used by Kinetix spatial algebra.
///
/// This is intentionally much smaller than `nalgebra::RealField`, so lightweight
/// forward-mode AD scalars can run the same dynamics code without implementing
/// a large numerical-traits surface.
pub trait KScalar:
    nalgebra::Scalar
    + Copy
    + Default
    + Debug
    + PartialEq
    + Zero
    + One
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
    + AddAssign
    + SubAssign
    + MulAssign
    + DivAssign
{
    /// Builds a scalar from an `f64` literal.
    fn from_f64(value: f64) -> Self;

    /// Computes sine and cosine together.
    fn sin_cos(self) -> (Self, Self);

    /// Returns the primal numeric value for pivot checks and diagnostics.
    fn value(self) -> f64;
}

impl KScalar for f64 {
    #[inline]
    fn from_f64(value: f64) -> Self {
        value
    }

    #[inline]
    fn sin_cos(self) -> (Self, Self) {
        libm_sin_cos(self)
    }

    #[inline]
    fn value(self) -> f64 {
        self
    }
}

fn libm_sin_cos(mut x: f64) -> (f64, f64) {
    const PI: f64 = core::f64::consts::PI;
    const TWO_PI: f64 = core::f64::consts::TAU;

    while x > PI {
        x -= TWO_PI;
    }
    while x < -PI {
        x += TWO_PI;
    }

    let x2 = x * x;
    let x4 = x2 * x2;
    let x6 = x4 * x2;
    let x8 = x4 * x4;
    let x10 = x8 * x2;
    let x12 = x10 * x2;
    let x14 = x12 * x2;
    let x16 = x14 * x2;

    let sin = x
        * (1.0 - x2 / 6.0 + x4 / 120.0 - x6 / 5_040.0 + x8 / 362_880.0 - x10 / 39_916_800.0
            + x12 / 6_227_020_800.0
            - x14 / 1_307_674_368_000.0
            + x16 / 355_687_428_096_000.0);
    let cos = 1.0 - x2 / 2.0 + x4 / 24.0 - x6 / 720.0 + x8 / 40_320.0 - x10 / 3_628_800.0
        + x12 / 479_001_600.0
        - x14 / 87_178_291_200.0
        + x16 / 20_922_789_888_000.0;

    (sin, cos)
}
