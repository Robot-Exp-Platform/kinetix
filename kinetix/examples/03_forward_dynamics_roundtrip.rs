mod common;

use common::{print_vector, seed_two_link_state, two_link_revolute_model};
use kinetix::prelude::*;

fn main() {
    let model = two_link_revolute_model();
    let mut data = Data::<2>::default();
    seed_two_link_state(&mut data);

    let target_q_ddot = data.q_ddot;
    rnea(&model, &mut data, None);
    let control_tau = data.tau;

    data.q_ddot.fill(0.0);
    data.tau = control_tau;
    aba(&model, &mut data, None);

    print_vector("tau from inverse dynamics", &control_tau);
    print_vector("target acceleration", &target_q_ddot);
    print_vector("aba recovered acceleration", &data.q_ddot);
    println!(
        "max error = {:.3e}",
        max_abs_diff(&target_q_ddot, &data.q_ddot)
    );
}

fn max_abs_diff<const N: usize>(lhs: &JointVector<N>, rhs: &JointVector<N>) -> f64 {
    let mut max_error = 0.0;
    for i in 0..N {
        let error = (lhs[i] - rhs[i]).abs();
        if error > max_error {
            max_error = error;
        }
    }
    max_error
}
