use approx::assert_relative_eq;

use box2d_lite_rs::dynamics::{BodyDef, World, WorldConfig};
use box2d_lite_rs::math::Vec2;

#[test]
fn integration_world_step_dt_zero_is_noop_for_state() {
    let mut world = World::new(Vec2::new(0.0, -10.0), 10);

    let h = world.create_body(BodyDef {
        width: Vec2::new(1.0, 1.0),
        position: Vec2::new(0.0, 2.0),
        mass: Some(1.0),
        ..Default::default()
    });

    let before_pos = world.body(h).position;
    let before_vel = world.body(h).velocity;

    world.step(0.0);

    let b = world.body(h);
    assert_relative_eq!(b.position.x, before_pos.x, epsilon = 1e-6);
    assert_relative_eq!(b.position.y, before_pos.y, epsilon = 1e-6);
    assert_relative_eq!(b.velocity.x, before_vel.x, epsilon = 1e-6);
    assert_relative_eq!(b.velocity.y, before_vel.y, epsilon = 1e-6);
}

#[test]
fn integration_config_position_correction_disables_bias() {
    // Build a world with two overlapping dynamic boxes.
    // With position correction enabled, bias impulses should push them apart.
    // With it disabled, the correction should be weaker (often none) in one step.
    let iterations = 10;
    let dt = 0.01;

    let mut w_on = World::with_config(
        Vec2::new(0.0, 0.0),
        iterations,
        WorldConfig {
            position_correction: true,
            ..WorldConfig::default()
        },
    );
    let mut w_off = World::with_config(
        Vec2::new(0.0, 0.0),
        iterations,
        WorldConfig {
            position_correction: false,
            ..WorldConfig::default()
        },
    );

    for w in [&mut w_on, &mut w_off] {
        w.create_body(BodyDef {
            width: Vec2::new(2.0, 2.0),
            position: Vec2::new(0.0, 0.0),
            mass: Some(1.0),
            ..Default::default()
        });
        w.create_body(BodyDef {
            width: Vec2::new(2.0, 2.0),
            position: Vec2::new(0.5, 0.0),
            mass: Some(1.0),
            ..Default::default()
        });
    }

    w_on.step(dt);
    w_off.step(dt);

    // Compare absolute x-velocity magnitude after a step;
    // position correction typically generates larger separating velocities.
    let v_on = w_on.bodies[0].velocity.x.abs() + w_on.bodies[1].velocity.x.abs();
    let v_off = w_off.bodies[0].velocity.x.abs() + w_off.bodies[1].velocity.x.abs();

    assert!(v_on >= v_off);
}
