mod common;

use common::{print_matrix, two_link_revolute_model};
use kinetix::prelude::*;

fn main() {
    let model = two_link_revolute_model();
    let mut data = Data::<2>::default();

    data.q[0] = 0.45;
    data.q[1] = -0.2;
    crba(&model, &mut data);

    print_matrix("M(q) from CRBA", &data.m_matrix);
    let determinant = data.m_matrix[(0, 0)] * data.m_matrix[(1, 1)]
        - data.m_matrix[(0, 1)] * data.m_matrix[(1, 0)];
    println!("det(M) = {:.6}", determinant);
}
