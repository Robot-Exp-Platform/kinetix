//! URDF loader for Kinetix models.
//!
//! This crate is intentionally `std`-enabled. URDF parsing, string handling and
//! graph traversal are initialization-time tasks and do not run in the real-time
//! control loop.

use std::error::Error;
use std::fmt;
use std::path::Path;

use kinetix_core::Model;

pub mod converter;
pub mod topology;

/// Error type returned by URDF-to-Kinetix conversion.
#[derive(Debug)]
pub enum KinetixUrdfError {
    /// Error reported by `urdf-rs`.
    Urdf(urdf_rs::UrdfError),
    /// The URDF graph has no unique root link.
    RootLink { roots: Vec<String> },
    /// A referenced link does not exist.
    MissingLink { name: String },
    /// A referenced parent link does not exist.
    MissingParentLink { name: String },
    /// A child link is reached more than once.
    DuplicateChildLink { name: String },
    /// The URDF contains a joint kind that Step 3 does not support yet.
    UnsupportedJoint { name: String, reason: String },
    /// The number of active joints does not match the requested const generic.
    DofMismatch { expected: usize, found: usize },
}

impl fmt::Display for KinetixUrdfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Urdf(err) => write!(f, "{err}"),
            Self::RootLink { roots } => write!(f, "expected one URDF root link, found {roots:?}"),
            Self::MissingLink { name } => write!(f, "URDF references missing link `{name}`"),
            Self::MissingParentLink { name } => {
                write!(f, "URDF joint references missing parent link `{name}`")
            }
            Self::DuplicateChildLink { name } => {
                write!(f, "link `{name}` is reached more than once")
            }
            Self::UnsupportedJoint { name, reason } => {
                write!(f, "unsupported joint `{name}`: {reason}")
            }
            Self::DofMismatch { expected, found } => {
                write!(
                    f,
                    "requested DOF={expected}, but URDF contains {found} active joints"
                )
            }
        }
    }
}

impl Error for KinetixUrdfError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Urdf(err) => Some(err),
            _ => None,
        }
    }
}

impl From<urdf_rs::UrdfError> for KinetixUrdfError {
    fn from(value: urdf_rs::UrdfError) -> Self {
        Self::Urdf(value)
    }
}

/// Loads a Kinetix model from a URDF file path.
pub fn load_from_file<P: AsRef<Path>, const DOF: usize>(
    path: P,
) -> Result<Model<DOF>, KinetixUrdfError> {
    let robot = urdf_rs::read_file(path)?;
    build_kinetix_model(robot)
}

/// Loads a Kinetix model from an in-memory URDF string.
pub fn load_from_str<const DOF: usize>(urdf_str: &str) -> Result<Model<DOF>, KinetixUrdfError> {
    let robot = urdf_rs::read_from_string(urdf_str)?;
    build_kinetix_model(robot)
}

/// Converts an already parsed URDF robot into a Kinetix model.
pub fn build_kinetix_model<const DOF: usize>(
    robot: urdf_rs::Robot,
) -> Result<Model<DOF>, KinetixUrdfError> {
    let ordered = topology::sort_active_joints(&robot)?;
    converter::convert_robot::<DOF>(&robot, &ordered)
}
