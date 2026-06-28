use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use kinetix_spatial::KScalar;
use num_traits::{One, Zero};

/// First-order forward automatic differentiation scalar.
///
/// `val` is the primal value and `der` carries one seeded tangent direction.
/// Full Jacobians are assembled by running the same dynamics once per seed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Dual {
    pub val: f64,
    pub der: f64,
}

impl Dual {
    #[inline]
    pub const fn constant(value: f64) -> Self {
        Self {
            val: value,
            der: 0.0,
        }
    }

    #[inline]
    pub const fn variable(value: f64, derivative: f64) -> Self {
        Self {
            val: value,
            der: derivative,
        }
    }

    #[inline]
    pub const fn value(self) -> f64 {
        self.val
    }

    #[inline]
    pub const fn derivative(self) -> f64 {
        self.der
    }
}

impl Add for Dual {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::variable(self.val + rhs.val, self.der + rhs.der)
    }
}

impl AddAssign for Dual {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.val += rhs.val;
        self.der += rhs.der;
    }
}

impl Sub for Dual {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::variable(self.val - rhs.val, self.der - rhs.der)
    }
}

impl SubAssign for Dual {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.val -= rhs.val;
        self.der -= rhs.der;
    }
}

impl Mul for Dual {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self::variable(self.val * rhs.val, self.der * rhs.val + self.val * rhs.der)
    }
}

impl MulAssign for Dual {
    #[inline]
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

impl Div for Dual {
    type Output = Self;

    #[inline]
    fn div(self, rhs: Self) -> Self::Output {
        let denominator = rhs.val * rhs.val;
        Self::variable(
            self.val / rhs.val,
            (self.der * rhs.val - self.val * rhs.der) / denominator,
        )
    }
}

impl DivAssign for Dual {
    #[inline]
    fn div_assign(&mut self, rhs: Self) {
        *self = *self / rhs;
    }
}

impl Neg for Dual {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self::variable(-self.val, -self.der)
    }
}

impl Zero for Dual {
    #[inline]
    fn zero() -> Self {
        Self::constant(0.0)
    }

    #[inline]
    fn is_zero(&self) -> bool {
        self.val == 0.0 && self.der == 0.0
    }
}

impl One for Dual {
    #[inline]
    fn one() -> Self {
        Self::constant(1.0)
    }
}

impl KScalar for Dual {
    #[inline]
    fn from_f64(value: f64) -> Self {
        Self::constant(value)
    }

    #[inline]
    fn sin_cos(self) -> (Self, Self) {
        let (sin, cos) = sin_cos(self.val);
        (
            Self::variable(sin, cos * self.der),
            Self::variable(cos, -sin * self.der),
        )
    }

    #[inline]
    fn value(self) -> f64 {
        self.val
    }
}

fn sin_cos(mut x: f64) -> (f64, f64) {
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
