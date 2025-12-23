use approx::assert_relative_eq;

use box2d_lite_rs::collision::arbiter::Contact;
use box2d_lite_rs::collision::collide;
use box2d_lite_rs::dynamics::{BodyDef, World};
use box2d_lite_rs::math::Vec2;

#[test]
fn integration_collide_non_overlapping_returns_zero() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 1);

    let a = world.create_body(BodyDef {
        width: Vec2::new(1.0, 1.0),
        position: Vec2::new(-10.0, 0.0),
        mass: Some(1.0),
        ..Default::default()
    });
    let b = world.create_body(BodyDef {
        width: Vec2::new(1.0, 1.0),
        position: Vec2::new(10.0, 0.0),
        mass: Some(1.0),
        ..Default::default()
    });

    let mut contacts = [Contact::default(); 2];
    let n = collide(&mut contacts, a, b, &world);
    assert_eq!(n, 0);
}

#[test]
fn integration_collide_overlapping_returns_contact() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 1);

    let a = world.create_body(BodyDef {
        width: Vec2::new(2.0, 2.0),
        position: Vec2::new(0.0, 0.0),
        mass: Some(1.0),
        ..Default::default()
    });
    let b = world.create_body(BodyDef {
        width: Vec2::new(2.0, 2.0),
        position: Vec2::new(0.5, 0.0),
        mass: Some(1.0),
        ..Default::default()
    });

    let mut contacts = [Contact::default(); 2];
    let n = collide(&mut contacts, a, b, &world);
    assert!(n > 0);

    // Normal should be unit-ish and separation should be <= 0 for contacts.
    for c in &contacts[..n] {
        let len = c.normal.length();
        assert_relative_eq!(len, 1.0, epsilon = 1e-3);
        assert!(c.separation <= 0.0);
    }
}
