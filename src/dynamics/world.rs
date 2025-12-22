use crate::dynamics::{Body, BodyDef};
use crate::math::Vec2;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodyHandle(pub usize);

pub struct World {
    bodies: Vec<Body>,
    gravity: Vec2,
    iterations: usize,
}

impl World {
    #[inline]
    pub fn new(gravity: Vec2, iterations: usize) -> Self {
        Self {
            bodies: Vec::new(),
            gravity: gravity,
            iterations: iterations,
        }
    }

    pub fn create_body(&mut self, def: BodyDef) -> BodyHandle {
        let id = self.bodies.len();
        self.bodies.push(Body::from_def(def));
        BodyHandle(id)
    }

    pub fn body(&self, h: BodyHandle) -> &Body {
        &self.bodies[h.0]
    }

    pub fn body_mut(&mut self, h: BodyHandle) -> &mut Body {
        &mut self.bodies[h.0]
    }

    pub fn broad_phase(&mut self) {
        // TODO: later. For now, do nothing.
    }

    pub fn step(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        self.broad_phase();

        // Integrate forces.
        for b in &mut self.bodies {
            if b.inv_mass == 0.0 {
                continue;
            }
            b.velocity += dt * (self.gravity + b.inv_mass * b.force);
            b.angular_velocity += dt * b.inv_i * b.torque;
        }

        // Integrate Velocities.
        for b in &mut self.bodies {
            b.position += dt * b.velocity;
            b.rotation += dt * b.angular_velocity;

            b.force.set(0.0, 0.0);
            b.torque = 0.0;
        }
    }

    // TODO: later
    // - pre-step contacts/joints using inv_dt
    // - iterative solver loop using self.iterations
}

// All tests are written by ChatGPT 5.2.
#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn create_body_returns_valid_handle() {
        let mut world = World::new(Vec2::new(0.0, -10.0), 10);
        let h = world.create_body(BodyDef {
            width: Vec2::new(1.0, 1.0),
            mass: Some(1.0),
            ..Default::default()
        });

        let b = world.body(h);
        assert_relative_eq!(b.width.x, 1.0);
        assert_relative_eq!(b.width.y, 1.0);
    }

    #[test]
    fn step_static_body_does_not_move() {
        let mut world = World::new(Vec2::new(0.0, -10.0), 10);

        let h = world.create_body(BodyDef {
            width: Vec2::new(1.0, 1.0),
            position: Vec2::new(0.0, 5.0),
            mass: None, // static
            ..Default::default()
        });

        world.step(0.1);

        let b = world.body(h);
        assert_relative_eq!(b.position.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(b.position.y, 5.0, epsilon = 1e-6);
        assert_relative_eq!(b.velocity.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(b.velocity.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn step_dynamic_body_accelerates_under_gravity() {
        let mut world = World::new(Vec2::new(0.0, -10.0), 10);

        let h = world.create_body(BodyDef {
            width: Vec2::new(1.0, 1.0),
            position: Vec2::new(0.0, 0.0),
            mass: Some(2.0), // inv_mass=0.5
            ..Default::default()
        });

        // One step of dt
        let dt = 0.1;
        world.step(dt);

        let b = world.body(h);

        // initial v = 0
        // v += dt * (gravity + inv_mass * force), force=0
        // v.y = -10 * 0.1 = -1
        assert_relative_eq!(b.velocity.y, -1.0, epsilon = 1e-6);

        // pos += dt * v => y = 0 + 0.1 * (-1) = -0.1
        assert_relative_eq!(b.position.y, -0.1, epsilon = 1e-6);
    }

    #[test]
    fn forces_are_cleared_after_step() {
        let mut world = World::new(Vec2::new(0.0, 0.0), 10);

        let h = world.create_body(BodyDef {
            width: Vec2::new(1.0, 1.0),
            mass: Some(1.0),
            ..Default::default()
        });

        {
            let b = world.body_mut(h);
            b.add_force(Vec2::new(3.0, 4.0));
            b.torque = 2.0;
        }

        world.step(0.1);

        let b = world.body(h);
        assert_relative_eq!(b.force.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(b.force.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(b.torque, 0.0, epsilon = 1e-6);
    }
}
