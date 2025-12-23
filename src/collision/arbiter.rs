use crate::collision::{EdgeNumber, collide};
use crate::dynamics::{Body, BodyHandle, World, WorldConfig, bodies_two_mut};
use crate::math::Vec2;

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct FeaturePair {
    pub in_edge1: EdgeNumber,
    pub out_edge1: EdgeNumber,
    pub in_edge2: EdgeNumber,
    pub out_edge2: EdgeNumber,
}

impl FeaturePair {
    #[inline]
    pub const fn new(in1: EdgeNumber, out1: EdgeNumber, in2: EdgeNumber, out2: EdgeNumber) -> Self {
        Self {
            in_edge1: in1,
            out_edge1: out1,
            in_edge2: in2,
            out_edge2: out2,
        }
    }

    /// Pack into a stable 32-bit key (little-endian layout).
    #[inline]
    pub const fn key(self) -> u32 {
        (self.in_edge1 as u32)
            | ((self.out_edge1 as u32) << 8)
            | ((self.in_edge2 as u32) << 16)
            | ((self.out_edge2 as u32) << 24)
    }

    /// Inverse of `key()`.
    #[inline]
    pub const fn from_key(v: u32) -> Self {
        Self {
            in_edge1: EdgeNumber::from_u8((v & 0xFF) as u8),
            out_edge1: EdgeNumber::from_u8(((v >> 8) & 0xFF) as u8),
            in_edge2: EdgeNumber::from_u8(((v >> 16) & 0xFF) as u8),
            out_edge2: EdgeNumber::from_u8(((v >> 24) & 0xFF) as u8),
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Contact {
    pub position: Vec2,
    pub normal: Vec2,
    pub r1: Vec2,
    pub r2: Vec2,
    pub separation: f32,
    pub pn: f32,  // accumulated normal impulse
    pub pt: f32,  // accumulated tangent impulse
    pub pnb: f32, // accumulated normal impulse for position bias
    pub mass_normal: f32,
    pub mass_tangent: f32,
    pub bias: f32,
    pub feature: FeaturePair,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
pub struct ArbiterKey {
    pub body1: BodyHandle,
    pub body2: BodyHandle,
}

impl ArbiterKey {
    #[inline]
    pub fn new(b1: BodyHandle, b2: BodyHandle) -> Self {
        let (body1, body2) = if b1 <= b2 { (b1, b2) } else { (b2, b1) };
        Self { body1, body2 }
    }
}

pub const MAX_POINTS: usize = 2;

pub struct Arbiter {
    pub contacts: [Contact; MAX_POINTS],
    pub num_contacts: usize,

    pub body1: BodyHandle,
    pub body2: BodyHandle,

    pub friction: f32,
}

impl Arbiter {
    #[inline]
    pub fn new(b1: BodyHandle, b2: BodyHandle, world: &World) -> Self {
        let (body1, body2) = if b1 <= b2 { (b1, b2) } else { (b2, b1) };
        let mut contacts = [Contact::default(); MAX_POINTS];
        let num_contacts = collide(&mut contacts, body1, body2, world);

        Self {
            contacts: contacts,
            num_contacts: num_contacts,
            body1: body1,
            body2: body2,

            friction: 0.0,
        }
    }

    #[inline]
    pub fn update(&mut self, new_contacts: &[Contact], warm_starting: bool) {
        debug_assert!(new_contacts.len() <= 2);

        let mut merged_contacts = [Contact::default(); 2];

        for (i, new_c) in new_contacts.iter().enumerate() {
            let k = self.contacts[..self.num_contacts]
                .iter()
                .position(|old_c| new_c.feature.key() == old_c.feature.key());

            // Copy out new_c.
            let mut c = *new_c;

            if let Some(j) = k {
                if warm_starting {
                    let c_old = &self.contacts[j];
                    c.pn = c_old.pn;
                    c.pt = c_old.pt;
                    c.pnb = c_old.pnb;
                } else {
                    c.pn = 0.0;
                    c.pt = 0.0;
                    c.pnb = 0.0;
                }
            }
            merged_contacts[i] = c;
        }
        for i in 0..new_contacts.len() {
            self.contacts[i] = merged_contacts[i];
        }
        self.num_contacts = new_contacts.len();
    }

    pub fn pre_step(&mut self, inv_dt: f32, bodies: &mut [Body], config: &WorldConfig) {
        let k_allowed_penetration: f32 = 0.01;
        let k_bias_factor: f32 = if config.position_correction { 0.2 } else { 0.0 };

        let (body1, body2) = bodies_two_mut(bodies, self.body1, self.body2);

        for c in &mut self.contacts[..self.num_contacts] {
            let r1 = c.position - body1.position;
            let r2 = c.position - body2.position;

            // Precompute normal mass, tangent mass, and bias.
            let rn1 = r1.dot(c.normal);
            let rn2 = r2.dot(c.normal);
            let mut k_normal = body1.inv_mass + body2.inv_mass;
            k_normal +=
                body1.inv_i * (r1.dot(r1) - rn1 * rn1) + body2.inv_i * (r2.dot(r2) - rn2 * rn2);
            c.mass_normal = 1.0 / k_normal;

            let tangent = Vec2::cross_vec_scalar(c.normal, 1.0);
            let rt1 = r1.dot(tangent);
            let rt2 = r2.dot(tangent);
            let mut k_tangent = body1.inv_mass + body2.inv_mass;
            k_tangent +=
                body1.inv_i * (r1.dot(r1) - rt1 * rt1) + body2.inv_i * (r2.dot(r2) - rt2 * rt2);
            c.mass_tangent = 1.0 / k_tangent;

            c.bias = -k_bias_factor * inv_dt * (c.separation + k_allowed_penetration).min(0.0);

            if config.accumulate_impulses {
                // Apply normal + friction impulse
                let p = c.pn * c.normal + c.pt * tangent;

                body1.velocity -= body1.inv_mass * p;
                body1.angular_velocity -= body1.inv_i * r1.cross(p);

                body2.velocity += body2.inv_mass * p;
                body2.angular_velocity += body2.inv_i * r2.cross(p);
            }
        }
    }

    pub fn apply_impulse(&mut self, bodies: &mut [Body], config: &WorldConfig) {
        let (b1, b2) = if self.body1 <= self.body2 {
            (self.body1, self.body2)
        } else {
            (self.body2, self.body1)
        };
        let (b1, b2) = bodies_two_mut(bodies, b1, b2);

        for c in &mut self.contacts {
            c.r1 = c.position - b1.position;
            c.r2 = c.position - b2.position;

            // Relative velocity at contact
            let mut dv = b2.velocity + Vec2::cross_scalar_vec(b2.angular_velocity, c.r2)
                - b1.velocity
                - Vec2::cross_scalar_vec(b1.angular_velocity, c.r1);

            // Compute normal impulse
            let vn = dv.dot(c.normal);

            let mut dpn = c.mass_normal * (-vn + c.bias);

            if config.accumulate_impulses {
                // Clamp the accumulated impulse
                let pn0 = c.pn;
                c.pn = (pn0 + dpn).max(0.0);
                dpn = c.pn - pn0;
            } else {
                dpn = dpn.max(0.0);
            }

            // Apply contact impulse
            let pn = dpn * c.normal;

            b1.velocity -= b1.inv_mass * pn;
            b1.angular_velocity -= b1.inv_i * c.r1.cross(pn);

            b2.velocity += b2.inv_mass * pn;
            b2.angular_velocity += b2.inv_i * c.r2.cross(pn);

            // Relative velocity at contact
            dv = b2.velocity + Vec2::cross_scalar_vec(b2.angular_velocity, c.r2)
                - b1.velocity
                - Vec2::cross_scalar_vec(b1.angular_velocity, c.r1);

            let tangent = Vec2::cross_vec_scalar(c.normal, 1.0);
            let vt = dv.dot(tangent);
            let mut dpt = c.mass_tangent * (-vt);

            if config.accumulate_impulses {
                // Compute friction impulse
                let max_pt = self.friction * c.pn;

                // Clamp friction
                let old_tangent_impulse = c.pt;
                c.pt = (old_tangent_impulse + dpt).clamp(-max_pt, max_pt);
                dpt = c.pt - old_tangent_impulse;
            } else {
                let max_pt = self.friction * dpn;
                dpt = dpt.clamp(-max_pt, max_pt);
            }

            // apply contact impulse
            let pt = dpt * tangent;

            b1.velocity -= b1.inv_mass * pt;
            b1.angular_velocity -= b1.inv_i * c.r1.cross(pt);

            b2.velocity += b2.inv_mass * pt;
            b2.angular_velocity += b2.inv_i * c.r2.cross(pt);
        }
    }
}
