use std::fs;

use kinetix_core::{Data, JointType, aba, rnea};
use kinetix_spatial::Vector3;
use kinetix_urdf::{load_from_file, load_from_str};

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual={actual}, expected={expected}, tolerance={tolerance}"
    );
}

fn single_pendulum_urdf() -> &'static str {
    r#"
<robot name="single">
  <link name="base"/>
  <link name="link1">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.3"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
</robot>
"#
}

fn double_pendulum_urdf() -> &'static str {
    r#"
<robot name="double">
  <link name="base"/>
  <link name="link1">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.3"/>
    </inertial>
  </link>
  <link name="link2">
    <inertial>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.6"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
</robot>
"#
}

#[test]
fn single_pendulum_loads_one_active_joint() {
    let model = load_from_str::<1>(single_pendulum_urdf()).unwrap();

    assert_eq!(model.parents, [0]);
    assert_eq!(model.joint_types, [JointType::RevoluteZ]);
    assert_close(model.inertias[0].matrix()[(0, 0)], 0.1, 1.0e-12);
    assert_close(model.inertias[0].matrix()[(1, 1)], 0.2, 1.0e-12);
    assert_close(model.inertias[0].matrix()[(2, 2)], 0.3, 1.0e-12);
    assert_close(model.inertias[0].matrix()[(3, 3)], 2.0, 1.0e-12);
    assert_close(model.inertias[0].matrix()[(4, 4)], 2.0, 1.0e-12);
    assert_close(model.inertias[0].matrix()[(5, 5)], 2.0, 1.0e-12);
}

#[test]
fn double_pendulum_topology_and_inertia_are_flattened() {
    let model = load_from_str::<2>(double_pendulum_urdf()).unwrap();

    assert_eq!(model.parents, [0, 0]);
    assert_eq!(
        model.joint_types,
        [JointType::RevoluteZ, JointType::RevoluteZ]
    );

    assert_close(model.inertia_transforms[1].translation()[0], 1.0, 1.0e-12);
    assert_close(model.inertia_transforms[1].translation()[1], 0.0, 1.0e-12);
    assert_close(model.inertia_transforms[1].translation()[2], 0.0, 1.0e-12);

    let link2_inertia = model.inertias[1].matrix();
    assert_close(link2_inertia[(0, 0)], 0.4, 1.0e-12);
    assert_close(link2_inertia[(1, 1)], 1.25, 1.0e-12);
    assert_close(link2_inertia[(2, 2)], 1.35, 1.0e-12);
    assert_close(link2_inertia[(1, 5)], -1.5, 1.0e-12);
    assert_close(link2_inertia[(2, 4)], 1.5, 1.0e-12);
    assert_close(link2_inertia[(3, 3)], 3.0, 1.0e-12);
    assert_close(link2_inertia[(4, 4)], 3.0, 1.0e-12);
    assert_close(link2_inertia[(5, 5)], 3.0, 1.0e-12);
}

#[test]
fn load_from_file_uses_same_conversion_path() {
    let path = std::env::temp_dir().join(format!(
        "kinetix-single-{}-{}.urdf",
        std::process::id(),
        "loader"
    ));
    fs::write(&path, single_pendulum_urdf()).unwrap();

    let model = load_from_file::<_, 1>(&path).unwrap();
    fs::remove_file(path).unwrap();

    assert_eq!(model.parents, [0]);
    assert_eq!(model.joint_types, [JointType::RevoluteZ]);
}

#[test]
fn loaded_double_pendulum_feeds_core_rnea_aba_round_trip() {
    let mut model = load_from_str::<2>(double_pendulum_urdf()).unwrap();
    model.gravity = Vector3::new(0.0, -9.81, 0.0);

    let mut data = Data::<2>::default();
    data.q[0] = 0.25;
    data.q[1] = -0.5;
    data.q_dot[0] = 0.8;
    data.q_dot[1] = -0.35;
    data.q_ddot[0] = 1.2;
    data.q_ddot[1] = -0.6;
    let target = data.q_ddot;

    rnea(&model, &mut data, None);
    data.q_ddot.fill(0.0);
    aba(&model, &mut data, None);

    assert_close(data.q_ddot[0], target[0], 1.0e-10);
    assert_close(data.q_ddot[1], target[1], 1.0e-10);
}
