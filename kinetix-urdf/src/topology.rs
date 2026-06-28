use std::collections::{HashMap, HashSet};

use kinetix_spatial::{SpatialTransform, WorldFrame};
use urdf_rs::{JointType as UrdfJointType, Robot};

use crate::KinetixUrdfError;
use crate::converter::transform_from_pose;

/// One active joint in topological order.
#[derive(Clone, Debug)]
pub struct OrderedJoint {
    /// Index into `Robot::joints`.
    pub joint_index: usize,
    /// Parent active joint index in the Kinetix arena.
    pub parent_index: usize,
    /// Fixed transform accumulated through skipped fixed joints.
    pub fixed_transform: SpatialTransform<WorldFrame, WorldFrame>,
}

/// Sorts active URDF joints into the arena order required by `kinetix-core`.
pub fn sort_active_joints(robot: &Robot) -> Result<Vec<OrderedJoint>, KinetixUrdfError> {
    let root = root_link(robot)?;
    let children_by_parent = children_by_parent(robot)?;
    let mut visited_links = HashSet::new();
    let mut ordered = Vec::new();

    visit_link(
        root.as_str(),
        None,
        SpatialTransform::identity(),
        &children_by_parent,
        robot,
        &mut visited_links,
        &mut ordered,
    )?;

    Ok(ordered)
}

fn root_link(robot: &Robot) -> Result<String, KinetixUrdfError> {
    let mut child_links = HashSet::new();
    for joint in &robot.joints {
        child_links.insert(joint.child.link.as_str());
    }

    let roots: Vec<String> = robot
        .links
        .iter()
        .filter(|link| !child_links.contains(link.name.as_str()))
        .map(|link| link.name.clone())
        .collect();

    match roots.as_slice() {
        [root] => Ok(root.clone()),
        _ => Err(KinetixUrdfError::RootLink { roots }),
    }
}

fn children_by_parent(robot: &Robot) -> Result<HashMap<&str, Vec<usize>>, KinetixUrdfError> {
    let link_names: HashSet<&str> = robot.links.iter().map(|link| link.name.as_str()).collect();
    let mut map: HashMap<&str, Vec<usize>> = HashMap::new();

    for (index, joint) in robot.joints.iter().enumerate() {
        if !link_names.contains(joint.parent.link.as_str()) {
            return Err(KinetixUrdfError::MissingParentLink {
                name: joint.parent.link.clone(),
            });
        }
        if !link_names.contains(joint.child.link.as_str()) {
            return Err(KinetixUrdfError::MissingLink {
                name: joint.child.link.clone(),
            });
        }
        map.entry(joint.parent.link.as_str())
            .or_default()
            .push(index);
    }

    Ok(map)
}

fn visit_link(
    link_name: &str,
    active_parent: Option<usize>,
    accumulated_transform: SpatialTransform<WorldFrame, WorldFrame>,
    children_by_parent: &HashMap<&str, Vec<usize>>,
    robot: &Robot,
    visited_links: &mut HashSet<String>,
    ordered: &mut Vec<OrderedJoint>,
) -> Result<(), KinetixUrdfError> {
    if !visited_links.insert(link_name.to_owned()) {
        return Err(KinetixUrdfError::DuplicateChildLink {
            name: link_name.to_owned(),
        });
    }

    let Some(children) = children_by_parent.get(link_name) else {
        return Ok(());
    };

    for &joint_index in children {
        let joint = &robot.joints[joint_index];
        let joint_origin = transform_from_pose(&joint.origin);
        let combined_transform = joint_origin * accumulated_transform;

        if is_active_joint(&joint.joint_type) {
            let arena_index = ordered.len();
            ordered.push(OrderedJoint {
                joint_index,
                parent_index: active_parent.unwrap_or(0),
                fixed_transform: combined_transform,
            });

            visit_link(
                joint.child.link.as_str(),
                Some(arena_index),
                SpatialTransform::identity(),
                children_by_parent,
                robot,
                visited_links,
                ordered,
            )?;
        } else {
            visit_link(
                joint.child.link.as_str(),
                active_parent,
                combined_transform,
                children_by_parent,
                robot,
                visited_links,
                ordered,
            )?;
        }
    }

    Ok(())
}

fn is_active_joint(joint_type: &UrdfJointType) -> bool {
    matches!(
        joint_type,
        UrdfJointType::Revolute | UrdfJointType::Continuous | UrdfJointType::Prismatic
    )
}
