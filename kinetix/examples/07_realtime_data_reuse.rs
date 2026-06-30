mod common;

use common::{print_vector, two_link_revolute_model};
use kinetix::prelude::*;

fn main() {
    let model = two_link_revolute_model();
    let mut data = Data::<2>::default();

    for step in 0..1_000 {
        let t = step as f64 * 0.001;
        data.q[0] = 0.25 * t.sin();
        data.q[1] = -0.35 * t.cos();
        data.q_dot[0] = 0.25 * t.cos();
        data.q_dot[1] = 0.35 * t.sin();
        data.q_ddot.fill(0.0);

        rnea(&model, &mut data, None);
    }

    print_vector("last gravity and velocity compensation torque", &data.tau);
    println!("Data was allocated once and reused for every control tick.");
}
