use kinetix_spatial::{KScalar, SpatialForce, SpatialMotion, Vector3, WorldFrame};

use crate::algorithms::spatial::{
    add_inertia, inertia_mul_motion, joint_motion, joint_transform, subtract_rank_one,
    transform_force_to_parent, transform_inertia_to_parent,
};
use crate::data::{TypedData, force_from_vector, motion_dot_force};
use crate::model::TypedModel;

const MIN_PIVOT: f64 = 1.0e-12;

/// Articulated Body Algorithm forward dynamics.
///
/// Inputs are read from `data.q`, `data.q_dot`, and `data.tau`. The solved
/// acceleration is written to `data.q_ddot`.
pub fn aba<T: KScalar, const DOF: usize>(
    model: &TypedModel<T, DOF>,
    data: &mut TypedData<T, DOF>,
    f_ext: Option<&[SpatialForce<WorldFrame, T>; DOF]>,
) {
    for i in 0..DOF {
        let x_joint = joint_transform(model.joint_types[i], data.q[i]);
        data.li_x_p[i] = x_joint * model.inertia_transforms[i];

        let vj = joint_motion(model.joint_types[i], data.q_dot[i]);
        let parent_v = if i == 0 {
            SpatialMotion::zeros()
        } else {
            data.v[model.parents[i]]
        };

        data.v[i] = data.li_x_p[i].apply_motion(&parent_v) + vj;
        data.c[i] = data.v[i].cross_motion(&vj);
        data.i_a[i] = model.inertias[i];

        let momentum = inertia_mul_motion(&data.i_a[i], &data.v[i]);
        data.p_a[i] = data.v[i].cross_force(&momentum);
        if let Some(external_forces) = f_ext {
            data.p_a[i] = data.p_a[i] - external_forces[i];
        }
    }

    for i in (0..DOF).rev() {
        let s = model.joint_types[i].motion_subspace();
        data.u[i] = inertia_mul_motion(&data.i_a[i], &s);
        data.d[i] = motion_dot_force(&s, &data.u[i]);
        data.joint_u[i] = data.tau[i] - motion_dot_force(&s, &data.p_a[i]);

        let articulated_inertia;
        let articulated_bias;
        if model.joint_types[i].is_active() && data.d[i].value().abs() > MIN_PIVOT {
            articulated_inertia = subtract_rank_one(&data.i_a[i], &data.u[i], data.d[i]);
            let bias_vector = data.p_a[i].to_vector()
                + articulated_inertia.matrix() * data.c[i].to_vector()
                + data.u[i].to_vector() * (data.joint_u[i] / data.d[i]);
            articulated_bias = force_from_vector(bias_vector);
        } else {
            articulated_inertia = data.i_a[i];
            let bias_vector =
                data.p_a[i].to_vector() + data.i_a[i].matrix() * data.c[i].to_vector();
            articulated_bias = force_from_vector(bias_vector);
        }

        if i != 0 {
            let parent = model.parents[i];
            let parent_inertia = transform_inertia_to_parent(&data.li_x_p[i], &articulated_inertia);
            let parent_bias = transform_force_to_parent(&data.li_x_p[i], &articulated_bias);
            data.i_a[parent] = add_inertia(data.i_a[parent], parent_inertia);
            data.p_a[parent] = data.p_a[parent] + parent_bias;
        }
    }

    let base_acceleration =
        SpatialMotion::<WorldFrame, T>::new(Vector3::from_element(T::zero()), -model.gravity);

    for i in 0..DOF {
        let parent_a = if i == 0 {
            base_acceleration
        } else {
            data.a[model.parents[i]]
        };

        data.a[i] = data.li_x_p[i].apply_motion(&parent_a) + data.c[i];

        if model.joint_types[i].is_active() && data.d[i].value().abs() > MIN_PIVOT {
            data.q_ddot[i] =
                (data.joint_u[i] - motion_dot_force(&data.a[i], &data.u[i])) / data.d[i];
            data.a[i] = data.a[i] + joint_motion(model.joint_types[i], data.q_ddot[i]);
        } else {
            data.q_ddot[i] = T::zero();
        }
    }
}
