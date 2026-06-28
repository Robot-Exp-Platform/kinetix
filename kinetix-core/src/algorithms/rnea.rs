use kinetix_spatial::{KScalar, SpatialForce, SpatialMotion, Vector3, WorldFrame};

use crate::algorithms::spatial::{
    inertia_mul_motion, joint_motion, joint_transform, transform_force_to_parent,
};
use crate::data::TypedData;
use crate::model::TypedModel;

/// Recursive Newton-Euler inverse dynamics.
///
/// Inputs are read from `data.q`, `data.q_dot`, and `data.q_ddot`. The result is
/// written to `data.tau`. No heap allocation is performed.
pub fn rnea<T: KScalar, const DOF: usize>(
    model: &TypedModel<T, DOF>,
    data: &mut TypedData<T, DOF>,
    f_ext: Option<&[SpatialForce<WorldFrame, T>; DOF]>,
) {
    let base_acceleration =
        SpatialMotion::<WorldFrame, T>::new(Vector3::from_element(T::zero()), -model.gravity);

    for i in 0..DOF {
        let x_joint = joint_transform(model.joint_types[i], data.q[i]);
        data.li_x_p[i] = x_joint * model.inertia_transforms[i];

        let vj = joint_motion(model.joint_types[i], data.q_dot[i]);
        let aj = joint_motion(model.joint_types[i], data.q_ddot[i]);

        let parent_v = if i == 0 {
            SpatialMotion::zeros()
        } else {
            data.v[model.parents[i]]
        };
        let parent_a = if i == 0 {
            base_acceleration
        } else {
            data.a[model.parents[i]]
        };

        let parent_v_in_child = data.li_x_p[i].apply_motion(&parent_v);
        let parent_a_in_child = data.li_x_p[i].apply_motion(&parent_a);

        data.v[i] = parent_v_in_child + vj;
        data.a[i] = parent_a_in_child + aj + data.v[i].cross_motion(&vj);

        let momentum = inertia_mul_motion(&model.inertias[i], &data.v[i]);
        data.f[i] =
            inertia_mul_motion(&model.inertias[i], &data.a[i]) + data.v[i].cross_force(&momentum);

        if let Some(external_forces) = f_ext {
            data.f[i] = data.f[i] - external_forces[i];
        }
    }

    for i in (0..DOF).rev() {
        let s = model.joint_types[i].motion_subspace();
        data.tau[i] = crate::data::motion_dot_force(&s, &data.f[i]);

        if i != 0 {
            let parent = model.parents[i];
            data.f[parent] =
                data.f[parent] + transform_force_to_parent(&data.li_x_p[i], &data.f[i]);
        }
    }
}
