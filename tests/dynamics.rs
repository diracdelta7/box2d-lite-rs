use approx::assert_relative_eq;

use box2d_lite_rs::dynamics::{BodyDef, World};
use box2d_lite_rs::math::Vec2;

#[test]
fn integration_smoke_world_step() {
    let mut world = World::new(Vec2::new(0.0, -9.8), 10);

    let h = world.create_body(BodyDef {
        width: Vec2::new(1.0, 1.0),
        position: Vec2::new(0.0, 1.0),
        mass: Some(1.0),
        ..Default::default()
    });

    world.step(0.1);

    let b = world.body(h);
    assert!(b.position.y < 1.0);
    assert_relative_eq!(b.velocity.x, 0.0, epsilon = 1e-6);
}
