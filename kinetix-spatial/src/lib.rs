#![no_std]
#![doc = "Typed, allocation-free spatial algebra for Kinetix."]

pub mod force;
pub mod frames;
pub mod inertia;
pub mod motion;
pub mod scalar;
pub mod transform;

pub use force::SpatialForce;
pub use frames::{Frame, LinkFrame, WorldFrame};
pub use inertia::SpatialInertia;
pub use motion::SpatialMotion;
pub use scalar::KScalar;
pub use transform::SpatialTransform;

pub type Vector3<T = f64> = nalgebra::SVector<T, 3>;
pub type Vector6<T = f64> = nalgebra::SVector<T, 6>;
pub type Matrix3<T = f64> = nalgebra::SMatrix<T, 3, 3>;
pub type Matrix6<T = f64> = nalgebra::SMatrix<T, 6, 6>;
