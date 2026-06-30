mod common;

use common::{print_vector, seed_two_link_state, two_link_revolute_model};
use kinetix::prelude::*;

fn main() {
    let model = two_link_revolute_model();
    let mut data = Data::<2>::default();
    seed_two_link_state(&mut data);

    rnea(&model, &mut data, None);

    print_vector("q", &data.q);
    print_vector("q_dot", &data.q_dot);
    print_vector("q_ddot", &data.q_ddot);
    print_vector("tau = rnea(model, q, q_dot, q_ddot)", &data.tau);
}
