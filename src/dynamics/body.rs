use crate::math::Vec2;

#[derive(Copy, Clone, Debug)]
pub struct BodyDef {
    pub width: Vec2,
    pub position: Vec2,
    pub rotation: f32,
    pub friction: f32,
    pub mass: Option<f32>, // None => static
}

impl Default for BodyDef {
    fn default() -> Self {
        Self {
            width: Vec2::new(1.0, 1.0),
            position: Vec2::new(0.0, 0.0),
            rotation: 0.0,
            friction: 0.2,
            mass: None,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Body {
    pub position: Vec2,
    pub rotation: f32,

    pub velocity: Vec2,
    pub angular_velocity: f32,

    pub force: Vec2,
    pub torque: f32,

    pub width: Vec2,

    pub friction: f32,
    pub inv_mass: f32,
    pub inv_i: f32,
}

impl Body {
    #[inline]
    pub fn add_force(&mut self, f: Vec2) {
        self.force += f;
    }

    #[inline]
    pub fn from_def(def: BodyDef) -> Self {
        let (inv_mass, inv_i) = match def.mass {
            // Dynamic body
            Some(mass) => {
                debug_assert!(mass > 0.0 && mass.is_finite());
                debug_assert!(def.width.x > 0.0 && def.width.y > 0.0);

                let inv_mass = 1.0 / mass;

                let i = mass * (def.width.x * def.width.x + def.width.y * def.width.y) / 12.0;
                let inv_i = 1.0 / i;

                (inv_mass, inv_i)
            }
            None => {
                // Static body
                (0.0, 0.0)
            }
        };

        Self {
            position: def.position,
            rotation: def.rotation,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: def.width,
            friction: def.friction,
            inv_mass,
            inv_i,
        }
    }
}

// All tests are written by ChatGPT 5.2.
#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn body_from_def_static_has_zero_inverse_mass_and_inertia() {
        let def = BodyDef {
            width: Vec2::new(2.0, 4.0),
            mass: None,
            ..Default::default()
        };
        let b = Body::from_def(def);

        assert_relative_eq!(b.inv_mass, 0.0);
        assert_relative_eq!(b.inv_i, 0.0);
        assert_relative_eq!(b.width.x, 2.0);
        assert_relative_eq!(b.width.y, 4.0);
    }

    #[test]
    fn body_from_def_dynamic_computes_inv_mass_and_inv_i() {
        let mass = 3.0;
        let w = Vec2::new(2.0, 4.0);

        let def = BodyDef {
            width: w,
            mass: Some(mass),
            ..Default::default()
        };
        let b = Body::from_def(def);

        // inv_mass = 1/m
        assert_relative_eq!(b.inv_mass, 1.0 / mass, epsilon = 1e-6);

        // i = m*(wx^2 + wy^2)/12
        let i = mass * (w.x * w.x + w.y * w.y) / 12.0;
        assert_relative_eq!(b.inv_i, 1.0 / i, epsilon = 1e-6);
    }

    #[test]
    fn add_force_accumulates() {
        let mut b = Body::from_def(BodyDef {
            width: Vec2::new(1.0, 1.0),
            mass: Some(1.0),
            ..Default::default()
        });

        b.add_force(Vec2::new(1.0, 2.0));
        b.add_force(Vec2::new(-0.5, 3.0));

        assert_relative_eq!(b.force.x, 0.5, epsilon = 1e-6);
        assert_relative_eq!(b.force.y, 5.0, epsilon = 1e-6);
    }
}
