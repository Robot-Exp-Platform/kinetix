use kinetix_spatial::KScalar;

use crate::algorithms::spatial::{
    add_inertia, joint_inertia_column, joint_project_force, joint_transform,
    transform_force_to_parent, transform_inertia_to_parent,
};
use crate::data::TypedData;
use crate::model::TypedModel;

/// Composite Rigid Body Algorithm.
///
/// The generalized inertia matrix is written to `data.m_matrix`.
pub fn crba<T: KScalar, const DOF: usize>(
    model: &TypedModel<T, DOF>,
    data: &mut TypedData<T, DOF>,
) {
    data.m_matrix.fill(T::zero());

    for i in 0..DOF {
        let x_joint = joint_transform(model.joint_types[i], data.q[i]);
        data.li_x_p[i] = x_joint * model.inertia_transforms[i];
        data.i_a[i] = model.inertias[i];
    }

    for i in (0..DOF).rev() {
        if i != 0 {
            let parent = model.parents[i];
            let transformed = transform_inertia_to_parent(&data.li_x_p[i], &data.i_a[i]);
            data.i_a[parent] = add_inertia(data.i_a[parent], transformed);
        }
    }

    for i in 0..DOF {
        let mut force = joint_inertia_column(model.joint_types[i], &data.i_a[i]);

        data.m_matrix[(i, i)] = joint_project_force(model.joint_types[i], &force);

        let mut j = i;
        while j != 0 {
            force = transform_force_to_parent(&data.li_x_p[j], &force);
            j = model.parents[j];

            let value = joint_project_force(model.joint_types[j], &force);
            data.m_matrix[(i, j)] = value;
            data.m_matrix[(j, i)] = value;
        }
    }
}
