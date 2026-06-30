mod common;

use common::{print_matrix, two_link_revolute_model};
use kinetix::prelude::*;

fn main() {
    let model = two_link_revolute_model();
    let q = JointVector::<2>::new(0.31, -0.43);
    let q_dot = JointVector::<2>::new(0.37, -0.22);
    let q_ddot = JointVector::<2>::new(0.91, -0.34);

    let (dtau_dq, dtau_dq_dot) = compute_rnea_derivatives(&model, &q, &q_dot, &q_ddot);

    print_matrix("d tau / d q", &dtau_dq);
    print_matrix("d tau / d q_dot", &dtau_dq_dot);
}
