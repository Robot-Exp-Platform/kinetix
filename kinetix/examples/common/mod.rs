#![allow(dead_code)]

use kinetix::prelude::*;

pub fn rigid_inertia(mass: f64, center_of_mass: Vector3) -> SpatialInertia<WorldFrame> {
    SpatialInertia::from_mass_com_inertia(mass, center_of_mass, Matrix3::identity() * 0.01)
}

pub fn two_link_revolute_model() -> Model<2> {
    Model::new(
        [0, 0],
        [
            SpatialTransform::identity(),
            SpatialTransform::new(Matrix3::identity(), Vector3::new(1.0, 0.0, 0.0)),
        ],
        [
            rigid_inertia(1.7, Vector3::new(0.45, 0.0, 0.0)),
            rigid_inertia(1.1, Vector3::new(0.35, 0.0, 0.0)),
        ],
        [JointType::RevoluteZ, JointType::RevoluteZ],
        Vector3::new(0.0, -9.81, 0.0),
    )
}

pub fn double_pendulum_urdf() -> &'static str {
    r#"
<robot name="double_pendulum">
  <link name="base"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.45 0 0" rpy="0 0 0"/>
      <mass value="1.7"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="link2">
    <inertial>
      <origin xyz="0.35 0 0" rpy="0 0 0"/>
      <mass value="1.1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  <joint name="elbow" type="revolute">
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
</robot>
"#
}

pub fn seed_two_link_state(data: &mut Data<2>) {
    data.q[0] = 0.3;
    data.q[1] = -0.8;
    data.q_dot[0] = 0.7;
    data.q_dot[1] = -0.2;
    data.q_ddot[0] = 1.1;
    data.q_ddot[1] = -0.4;
}

pub fn print_vector<const N: usize>(label: &str, vector: &JointVector<N>) {
    print!("{label} = [");
    for i in 0..N {
        if i > 0 {
            print!(", ");
        }
        print!("{:.6}", vector[i]);
    }
    println!("]");
}

pub fn print_matrix<const N: usize>(label: &str, matrix: &JointMatrix<N>) {
    println!("{label} =");
    for row in 0..N {
        print!("  [");
        for col in 0..N {
            if col > 0 {
                print!(", ");
            }
            print!("{:>10.6}", matrix[(row, col)]);
        }
        println!("]");
    }
}
