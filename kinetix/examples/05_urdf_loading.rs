mod common;

use common::{double_pendulum_urdf, print_vector};
use kinetix::prelude::*;

fn main() -> Result<(), KinetixUrdfError> {
    let mut model = load_from_str::<2>(double_pendulum_urdf())?;
    model.gravity = Vector3::new(0.0, -9.81, 0.0);

    let mut data = Data::<2>::default();
    data.q[0] = 0.25;
    data.q[1] = -0.5;

    rnea(&model, &mut data, None);

    println!("parents     = {:?}", model.parents);
    println!("joint types  = {:?}", model.joint_types);
    print_vector("gravity torque", &data.tau);

    Ok(())
}
