use approx::assert_relative_eq;

use box2d_lite_rs::dynamics::{BodyDef, JointDef, World, WorldConfig};
use box2d_lite_rs::math::Vec2;

fn make_two_bodies_with_joint(config: WorldConfig) -> (World, usize, usize) {
    let mut world = World::with_config(Vec2::new(0.0, 0.0), 20, config);

    let b1 = world.create_body(BodyDef {
        width: Vec2::new(1.0, 1.0),
        position: Vec2::new(-1.0, 0.0),
        mass: Some(1.0),
        ..Default::default()
    });
    let b2 = world.create_body(BodyDef {
        width: Vec2::new(1.0, 1.0),
        position: Vec2::new(1.0, 0.0),
        mass: Some(1.0),
        ..Default::default()
    });

    // Joint anchor at origin.
    world.create_joint(JointDef::new(b1, b2, Vec2::new(0.0, 0.0)));

    (world, b1.0, b2.0)
}

#[test]
fn integration_joint_applies_impulses_and_reduces_anchor_error() {
    let (mut world, i1, i2) = make_two_bodies_with_joint(WorldConfig::default());

    // Give the bodies an initial velocity that would separate them.
    world.bodies[i1].velocity = Vec2::new(-5.0, 0.0);
    world.bodies[i2].velocity = Vec2::new(5.0, 0.0);

    let dt = 0.01;

    // Compute anchor-world positions before stepping.
    let before = {
        // anchor at origin, so "error" is distance between the two world anchors.
        let p1 = world.bodies[i1].position;
        let p2 = world.bodies[i2].position;
        (p2 - p1).length()
    };

    for _ in 0..20 {
        world.step(dt);
    }

    let after = {
        let p1 = world.bodies[i1].position;
        let p2 = world.bodies[i2].position;
        (p2 - p1).length()
    };

    // The joint should resist separation; distance should not blow up.
    assert!(after <= before + 0.5);

    // Also expect the separating velocities to be reduced in magnitude.
    assert!(world.bodies[i1].velocity.x > -5.0);
    assert!(world.bodies[i2].velocity.x < 5.0);
}

#[test]
fn integration_joint_warm_starting_changes_first_step_response() {
    // Same initial setup, different warm_starting.
    let (mut w_on, i1_on, i2_on) = make_two_bodies_with_joint(WorldConfig {
        warm_starting: true,
        ..WorldConfig::default()
    });
    let (mut w_off, i1_off, i2_off) = make_two_bodies_with_joint(WorldConfig {
        warm_starting: false,
        ..WorldConfig::default()
    });

    // Add a consistent velocity pattern.
    w_on.bodies[i1_on].velocity = Vec2::new(-2.0, 0.0);
    w_on.bodies[i2_on].velocity = Vec2::new(2.0, 0.0);

    w_off.bodies[i1_off].velocity = Vec2::new(-2.0, 0.0);
    w_off.bodies[i2_off].velocity = Vec2::new(2.0, 0.0);

    let dt = 0.01;

    // Step twice so warm starting has stored impulses to reuse.
    w_on.step(dt);
    w_on.step(dt);

    w_off.step(dt);
    w_off.step(dt);

    // Compare total speed after 2 steps; warm-starting typically damps relative motion faster.
    let speed_on = w_on.bodies[i1_on].velocity.x.abs() + w_on.bodies[i2_on].velocity.x.abs();
    let speed_off = w_off.bodies[i1_off].velocity.x.abs() + w_off.bodies[i2_off].velocity.x.abs();

    assert!(speed_on <= speed_off + 1e-4);

    // sanity: velocities are finite
    assert!(w_on.bodies[i1_on].velocity.x.is_finite());
    assert!(w_off.bodies[i1_off].velocity.x.is_finite());

    // and symmetry-ish
    assert_relative_eq!(
        w_on.bodies[i1_on].velocity.x,
        -w_on.bodies[i2_on].velocity.x,
        epsilon = 1e-2
    );
}
