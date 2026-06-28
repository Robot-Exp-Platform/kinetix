use std::array;

use kinetix_core::{JointType, Model};
use kinetix_spatial::{Matrix3, SpatialInertia, SpatialTransform, Vector3, WorldFrame};
use urdf_rs::{Inertial, Joint, JointType as UrdfJointType, Link, Pose, Robot, Vec3 as UrdfVec3};

use crate::KinetixUrdfError;
use crate::topology::OrderedJoint;

const AXIS_TOLERANCE: f64 = 1.0e-12;

/// Converts an ordered URDF robot into a fixed-size Kinetix model.
pub fn convert_robot<const DOF: usize>(
    robot: &Robot,
    ordered: &[OrderedJoint],
) -> Result<Model<DOF>, KinetixUrdfError> {
    if ordered.len() != DOF {
        return Err(KinetixUrdfError::DofMismatch {
            expected: DOF,
            found: ordered.len(),
        });
    }

    let parents = array::from_fn(|i| ordered[i].parent_index);
    let inertia_transforms = array::from_fn(|i| ordered[i].fixed_transform);
    let mut joint_types_result = [JointType::Fixed; DOF];
    for i in 0..DOF {
        joint_types_result[i] = convert_joint_type(&robot.joints[ordered[i].joint_index])?;
    }

    let inertias = {
        let mut result = [SpatialInertia::zeros(); DOF];
        for i in 0..DOF {
            let joint = &robot.joints[ordered[i].joint_index];
            let child_link = find_link(robot, joint.child.link.as_str())?;
            result[i] = inertia_from_link(child_link);
        }
        result
    };

    Ok(Model::without_gravity(
        parents,
        inertia_transforms,
        inertias,
        joint_types_result,
    ))
}

/// Converts a URDF pose into a Kinetix spatial transform.
pub fn transform_from_pose(pose: &Pose) -> SpatialTransform<WorldFrame, WorldFrame> {
    SpatialTransform::new(rotation_from_rpy(pose.rpy), vector_from_urdf(pose.xyz))
}

/// Converts a URDF link inertial block into angular-first spatial inertia.
pub fn inertia_from_link(link: &Link) -> SpatialInertia<WorldFrame> {
    inertia_from_inertial(&link.inertial)
}

/// Converts a URDF inertial block into angular-first spatial inertia.
pub fn inertia_from_inertial(inertial: &Inertial) -> SpatialInertia<WorldFrame> {
    let mass = inertial.mass.value;
    let com = vector_from_urdf(inertial.origin.xyz);
    let rotation = rotation_from_rpy(inertial.origin.rpy);
    let inertia = &inertial.inertia;
    let inertia_in_inertial_frame = Matrix3::new(
        inertia.ixx,
        inertia.ixy,
        inertia.ixz,
        inertia.ixy,
        inertia.iyy,
        inertia.iyz,
        inertia.ixz,
        inertia.iyz,
        inertia.izz,
    );
    let inertia_in_link_frame = rotation * inertia_in_inertial_frame * rotation.transpose();

    SpatialInertia::from_mass_com_inertia(mass, com, inertia_in_link_frame)
}

fn find_link<'a>(robot: &'a Robot, name: &str) -> Result<&'a Link, KinetixUrdfError> {
    robot
        .links
        .iter()
        .find(|link| link.name == name)
        .ok_or_else(|| KinetixUrdfError::MissingLink {
            name: name.to_owned(),
        })
}

fn convert_joint_type(joint: &Joint) -> Result<JointType, KinetixUrdfError> {
    match joint.joint_type {
        UrdfJointType::Revolute | UrdfJointType::Continuous => {
            require_positive_z_axis(joint)?;
            Ok(JointType::RevoluteZ)
        }
        UrdfJointType::Prismatic => {
            require_positive_z_axis(joint)?;
            Ok(JointType::PrismaticZ)
        }
        UrdfJointType::Fixed => Ok(JointType::Fixed),
        UrdfJointType::Floating | UrdfJointType::Planar | UrdfJointType::Spherical => {
            Err(KinetixUrdfError::UnsupportedJoint {
                name: joint.name.clone(),
                reason: "only fixed, revolute/continuous Z, and prismatic Z joints are supported"
                    .to_owned(),
            })
        }
    }
}

fn require_positive_z_axis(joint: &Joint) -> Result<(), KinetixUrdfError> {
    let axis = joint.axis.xyz;
    let is_z = axis[0].abs() <= AXIS_TOLERANCE
        && axis[1].abs() <= AXIS_TOLERANCE
        && (axis[2] - 1.0).abs() <= AXIS_TOLERANCE;

    if is_z {
        Ok(())
    } else {
        Err(KinetixUrdfError::UnsupportedJoint {
            name: joint.name.clone(),
            reason: format!(
                "axis [{}, {}, {}] is not the supported positive Z axis",
                axis[0], axis[1], axis[2]
            ),
        })
    }
}

fn vector_from_urdf(vector: UrdfVec3) -> Vector3 {
    Vector3::new(vector[0], vector[1], vector[2])
}

fn rotation_from_rpy(rpy: UrdfVec3) -> Matrix3 {
    let (roll_sin, roll_cos) = rpy[0].sin_cos();
    let (pitch_sin, pitch_cos) = rpy[1].sin_cos();
    let (yaw_sin, yaw_cos) = rpy[2].sin_cos();

    let rx = Matrix3::new(
        1.0, 0.0, 0.0, 0.0, roll_cos, -roll_sin, 0.0, roll_sin, roll_cos,
    );
    let ry = Matrix3::new(
        pitch_cos, 0.0, pitch_sin, 0.0, 1.0, 0.0, -pitch_sin, 0.0, pitch_cos,
    );
    let rz = Matrix3::new(yaw_cos, -yaw_sin, 0.0, yaw_sin, yaw_cos, 0.0, 0.0, 0.0, 1.0);

    rz * ry * rx
}
