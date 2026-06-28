#![no_std]
#![doc = "Allocation-free rigid-body dynamics core for Kinetix."]

#[cfg(test)]
extern crate std;

pub mod algorithms;
pub mod data;
pub mod joints;
pub mod model;

pub use algorithms::{aba, crba, rnea};
pub use data::TypedData;
pub use joints::JointType;
pub use model::TypedModel;

/// Backward-compatible numeric model alias.
pub type Model<const DOF: usize, T = f64> = TypedModel<T, DOF>;

/// Backward-compatible numeric data alias.
pub type Data<const DOF: usize, T = f64> = TypedData<T, DOF>;

/// Fixed-size vector in joint space.
pub type JointVector<const DOF: usize, T = f64> = nalgebra::SVector<T, DOF>;

/// Fixed-size square matrix in joint space.
pub type JointMatrix<const DOF: usize, T = f64> = nalgebra::SMatrix<T, DOF, DOF>;
