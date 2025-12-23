use crate::collision::{Arbiter, ArbiterKey};
use crate::dynamics::{Body, BodyDef, Joint, JointDef};
use crate::math::Vec2;
use std::collections::BTreeMap;
use std::collections::btree_map::Entry;

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
pub struct BodyHandle(pub usize);

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
pub struct JointHandle(pub usize);

#[derive(Copy, Clone, Debug)]
pub struct WorldConfig {
    pub accumulate_impulses: bool,
    pub warm_starting: bool,
    pub position_correction: bool,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            accumulate_impulses: true,
            warm_starting: true,
            position_correction: true,
        }
    }
}

pub struct World {
    pub gravity: Vec2,
    pub iterations: u32,
    pub config: WorldConfig,
    pub bodies: Vec<Body>,
    pub joints: Vec<Joint>,
    pub arbiters: BTreeMap<ArbiterKey, Arbiter>,
}

impl World {
    #[inline]
    pub fn new(gravity: Vec2, iterations: u32) -> Self {
        Self {
            bodies: Vec::new(),
            joints: Vec::new(),
            arbiters: BTreeMap::new(),
            gravity,
            iterations,
            config: WorldConfig::default(),
        }
    }

    pub fn with_config(gravity: Vec2, iterations: u32, config: WorldConfig) -> Self {
        Self {
            config,
            ..Self::new(gravity, iterations)
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

    pub fn bodies_two_mut(&mut self, a: BodyHandle, b: BodyHandle) -> (&mut Body, &mut Body) {
        let bodies = &mut self.bodies;
        bodies_two_mut(bodies, a, b)
    }

    pub fn create_joint(&mut self, def: JointDef) -> JointHandle {
        let id = self.joints.len();
        self.joints.push(Joint::from_def(&self, def));
        JointHandle(id)
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.joints.clear();
        self.arbiters.clear();
    }

    pub fn broad_phase(&mut self) {
        // O(n^2) broad-phase
        let n = self.bodies.len();
        for i in 0..n {
            let bi = BodyHandle(i);
            for j in i + 1..n {
                let bj = BodyHandle(j);

                if self.bodies[i].inv_mass == 0.0 && self.bodies[j].inv_mass == 0.0 {
                    continue;
                }

                let new_arb = Arbiter::new(bi, bj, &self);
                let key = ArbiterKey::new(bi, bj);

                if new_arb.num_contacts > 0 {
                    match self.arbiters.entry(key) {
                        Entry::Vacant(e) => {
                            e.insert(new_arb);
                        }
                        Entry::Occupied(mut e) => {
                            let arb = e.get_mut();
                            arb.update(&new_arb.contacts, self.config.warm_starting);
                        }
                    }
                } else {
                    self.arbiters.remove(&key);
                }
            }
        }
    }

    pub fn step(&mut self, dt: f32) {
        let inv_dt = if dt <= 0.0 { 0.0 } else { 1.0 / dt };

        self.broad_phase();

        // Split world so we can borrow parts at the same time.
        let World {
            bodies,
            joints,
            arbiters,
            gravity,
            iterations,
            config,
            ..
        } = self;

        // Integrate forces.
        for b in &mut bodies.iter_mut() {
            if b.inv_mass == 0.0 {
                continue;
            }
            b.velocity += dt * (*gravity + b.inv_mass * b.force);
            b.angular_velocity += dt * b.inv_i * b.torque;
        }

        // Perform pre-steps.
        for arb in &mut arbiters.values_mut() {
            arb.pre_step(inv_dt, bodies, config);
        }

        for joint in &mut joints.iter_mut() {
            joint.pre_step(inv_dt, bodies, config);
        }

        // Perform iterations
        for _ in 0..(*iterations as usize) {
            for arb in arbiters.values_mut() {
                arb.apply_impulse(bodies, config);
            }

            for joint in &mut joints.iter_mut() {
                joint.apply_impulse(bodies);
            }
        }

        // Integrate Velocities.
        for b in bodies {
            b.position += dt * b.velocity;
            b.rotation += dt * b.angular_velocity;

            b.force.set(0.0, 0.0);
            b.torque = 0.0;
        }
    }
}

pub fn bodies_two_mut(bodies: &mut [Body], a: BodyHandle, b: BodyHandle) -> (&mut Body, &mut Body) {
    assert!(a != b, "bodies_two_mut called with identical handles");

    let (i, j) = if a.0 <= b.0 { (a.0, b.0) } else { (b.0, a.0) };
    let (left, right) = bodies.split_at_mut(j);
    let bi = &mut left[i];
    let bj = &mut right[0];

    if a.0 < b.0 { (bi, bj) } else { (bj, bi) }
}

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
