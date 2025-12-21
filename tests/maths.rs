use approx::assert_relative_eq;

use box2d_lite_rs::math::{Mat22, Vec2};
use box2d_lite_rs::math::utils::sign_nonzero;

#[test]
fn public_math_api_smoke() {
    let v = Vec2::new(1.0, 2.0);
    let r = Mat22::from_angle(0.0);
    let _ = r * v;
}

#[test]
fn vec2_and_mat22_work_together() {
    let r = Mat22::from_angle(core::f32::consts::FRAC_PI_2);
    let v = Vec2::new(1.0, 0.0);
    let out = r * v;

    assert_relative_eq!(out.x, 0.0, epsilon = 1e-5);
    assert_relative_eq!(out.y, 1.0, epsilon = 1e-5);
}

#[test]
fn sign_nonzero_is_public_and_correct() {
    assert_relative_eq!(sign_nonzero(-1.0), -1.0);
    assert_relative_eq!(sign_nonzero(0.0), 1.0);
}
