#![no_std]
#![doc = "Forward-mode automatic differentiation helpers for Kinetix dynamics."]

pub mod derivatives;
pub mod scalar;

pub use derivatives::{compute_rnea_derivatives, lift_model};
pub use scalar::Dual;
