use crate::dynamics::{Body, BodyHandle, World, WorldConfig, bodies_two_mut};
use crate::math::{Mat22, Vec2};

#[derive(Copy, Clone, Debug)]
pub struct JointDef {
    pub body1: BodyHandle,
    pub body2: BodyHandle,
    pub anchor: Vec2, // 世界坐标锚点
    pub softness: f32,
    pub bias_factor: f32,
}

impl JointDef {
    pub fn new(body1: BodyHandle, body2: BodyHandle, anchor: Vec2) -> Self {
        Self {
            body1,
            body2,
            anchor,
            softness: 0.0,
            bias_factor: 0.2,
        }
    }
}

pub struct Joint {
    m: Mat22,
    local_anchor1: Vec2,
    local_anchor2: Vec2,
    r1: Vec2,
    r2: Vec2,
    bias: Vec2,
    p: Vec2,
    body1: BodyHandle,
    body2: BodyHandle,
    bias_factor: f32,
    softness: f32,
}

impl Joint {
    #[inline]
    pub fn endpoints(&self, world: &World) -> (Vec2, Vec2) {
        let b1 = world.body(self.body1);
        let b2 = world.body(self.body2);

        let r1 = Mat22::from_angle(b1.rotation) * self.local_anchor1;
        let r2 = Mat22::from_angle(b2.rotation) * self.local_anchor2;

        (b1.position + r1, b2.position + r2)
    }

    #[inline]
    pub fn body_centers_and_anchors(&self, world: &World) -> (Vec2, Vec2, Vec2, Vec2) {
        let b1 = world.body(self.body1);
        let b2 = world.body(self.body2);

        let r1 = Mat22::from_angle(b1.rotation) * self.local_anchor1;
        let r2 = Mat22::from_angle(b2.rotation) * self.local_anchor2;

        let x1 = b1.position;
        let x2 = b2.position;
        let p1 = x1 + r1;
        let p2 = x2 + r2;

        (x1, p1, x2, p2)
    }

    #[inline]
    pub fn from_def(world: &World, def: JointDef) -> Self {
        let b1 = world.body(def.body1);
        let b2 = world.body(def.body2);

        let rot1_t = Mat22::from_angle(b1.rotation).transpose();
        let rot2_t = Mat22::from_angle(b2.rotation).transpose();

        let r1 = def.anchor - b1.position;
        let r2 = def.anchor - b2.position;

        let local_anchor1 = rot1_t * r1;
        let local_anchor2 = rot2_t * r2;

        Self {
            m: Mat22::default(),
            local_anchor1: local_anchor1,
            local_anchor2: local_anchor2,
            r1: Vec2::default(),
            r2: Vec2::default(),
            bias: Vec2::default(),
            p: Vec2::default(),
            body1: def.body1,
            body2: def.body2,
            bias_factor: def.bias_factor,
            softness: def.softness,
        }
    }

    pub fn set(&mut self, world: &World, b1: BodyHandle, b2: BodyHandle, anchor: Vec2) {
        self.body1 = b1;
        self.body2 = b2;

        let body1 = world.body(b1);
        let body2 = world.body(b2);

        let rot1_t = Mat22::from_angle(body1.rotation).transpose();
        let rot2_t = Mat22::from_angle(body2.rotation).transpose();

        self.local_anchor1 = rot1_t * (anchor - body1.position);
        self.local_anchor2 = rot2_t * (anchor - body2.position);

        self.p.set(0.0, 0.0);

        self.softness = 0.0;
        self.bias_factor = 0.2;
    }

    pub fn pre_step(&mut self, inv_dt: f32, bodies: &mut [Body], config: &WorldConfig) {
        // Cache configs, latter world will be mutably borrowed.
        let (body1, body2) = bodies_two_mut(bodies, self.body1, self.body2);

        let rot1 = Mat22::from_angle(body1.rotation);
        let rot2 = Mat22::from_angle(body2.rotation);

        self.r1 = rot1 * self.local_anchor1;
        self.r2 = rot2 * self.local_anchor2;

        let k1 = Mat22::new(
            Vec2::new(body1.inv_mass + body2.inv_mass, 0.0),
            Vec2::new(0.0, body1.inv_mass + body2.inv_mass),
        );

        let k2 = Mat22::new(
            Vec2::new(
                body1.inv_i * self.r1.y * self.r1.y,
                -body1.inv_i * self.r1.x * self.r1.y,
            ),
            Vec2::new(
                -body1.inv_i * self.r1.x * self.r1.y,
                body1.inv_i * self.r1.x * self.r1.x,
            ),
        );

        let k3 = Mat22::new(
            Vec2::new(
                body2.inv_i * self.r2.y * self.r2.y,
                -body2.inv_i * self.r2.x * self.r2.y,
            ),
            Vec2::new(
                -body2.inv_i * self.r2.x * self.r2.y,
                body2.inv_i * self.r2.x * self.r2.x,
            ),
        );

        let mut k = k1 + k2 + k3;
        k.col1.x += self.softness;
        k.col2.y += self.softness;

        self.m = k.invert();

        let p1 = body1.position + self.r1;
        let p2 = body2.position + self.r2;
        let dp = p2 - p1;

        if config.position_correction {
            self.bias = -self.bias_factor * inv_dt * dp;
        } else {
            self.bias.set(0.0, 0.0);
        }

        if config.warm_starting {
            body1.velocity -= body1.inv_mass * self.p;
            body1.angular_velocity -= body1.inv_i * self.r1.cross(self.p);

            body2.velocity += body2.inv_mass * self.p;
            body2.angular_velocity += body2.inv_i * self.r2.cross(self.p);
        } else {
            self.p.set(0.0, 0.0);
        }
    }

    pub fn apply_impulse(&mut self, bodies: &mut [Body]) {
        let (body1, body2) = bodies_two_mut(bodies, self.body1, self.body2);

        let dv = body2.velocity + Vec2::cross_scalar_vec(body2.angular_velocity, self.r2)
            - body1.velocity
            - Vec2::cross_scalar_vec(body1.angular_velocity, self.r1);
        let impulse = self.m * (self.bias - dv - self.softness * self.p);

        body1.velocity -= body1.inv_mass * impulse;
        body1.angular_velocity -= body1.inv_i * self.r1.cross(impulse);

        body2.velocity += body2.inv_mass * impulse;
        body2.angular_velocity += body2.inv_i * self.r2.cross(impulse);

        self.p += impulse;
    }
}
