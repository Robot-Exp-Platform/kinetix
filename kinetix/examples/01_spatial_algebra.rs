mod common;

use kinetix::prelude::*;

fn main() {
    let v1 = SpatialMotion::<WorldFrame>::from_vector(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
    let v2 =
        SpatialMotion::<WorldFrame>::from_vector(Vector6::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0));
    let wrench =
        SpatialForce::<WorldFrame>::from_vector(Vector6::new(0.7, 0.8, 0.9, 1.0, 1.1, 1.2));

    let transform = SpatialTransform::<WorldFrame, LinkFrame<1>>::new(
        rot_z(0.31),
        Vector3::new(0.4, -0.2, 0.7),
    );

    println!(
        "motion cross motion = {}",
        v1.cross_motion(&v2).to_vector().transpose()
    );
    println!(
        "motion cross force  = {}",
        v1.cross_force(&wrench).to_vector().transpose()
    );
    println!(
        "transformed motion  = {}",
        transform.apply_motion(&v1).to_vector().transpose()
    );
    println!(
        "transformed force   = {}",
        transform.apply_force(&wrench).to_vector().transpose()
    );
}

fn rot_z(theta: f64) -> Matrix3 {
    let (sin, cos) = theta.sin_cos();
    Matrix3::new(cos, -sin, 0.0, sin, cos, 0.0, 0.0, 0.0, 1.0)
}
