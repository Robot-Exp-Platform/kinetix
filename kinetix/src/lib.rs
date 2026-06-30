//! Public entry point for the Kinetix workspace.
//!
//! The lower crates stay independently usable, while this crate provides a
//! compact import surface for examples and applications.

pub mod autodiff {
    pub use kinetix_autodiff::*;
}

pub mod core {
    pub use kinetix_core::*;
}

pub mod spatial {
    pub use kinetix_spatial::*;
}

pub mod urdf {
    pub use kinetix_urdf::*;
}

pub mod prelude {
    pub use crate::autodiff::compute_rnea_derivatives;
    pub use crate::core::{Data, JointMatrix, JointType, JointVector, Model, aba, crba, rnea};
    pub use crate::spatial::{
        LinkFrame, Matrix3, Matrix6, SpatialForce, SpatialInertia, SpatialMotion, SpatialTransform,
        Vector3, Vector6, WorldFrame,
    };
    pub use crate::urdf::{KinetixUrdfError, load_from_file, load_from_str};
}
