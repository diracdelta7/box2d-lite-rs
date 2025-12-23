use core::ops::{Add, Mul};

use crate::math::Vec2;

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Mat22 {
    pub col1: Vec2,
    pub col2: Vec2,
}

impl Mat22 {
    #[inline]
    pub const fn new(col1: Vec2, col2: Vec2) -> Self {
        Self { col1, col2 }
    }

    #[inline]
    pub fn from_angle(angle: f32) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Self::new(Vec2::new(c, s), Vec2::new(-s, c))
    }

    #[inline]
    pub fn transpose(self) -> Self {
        Self::new(
            Vec2::new(self.col1.x, self.col2.x),
            Vec2::new(self.col1.y, self.col2.y),
        )
    }

    #[inline]
    pub fn invert(self) -> Self {
        let a = self.col1.x;
        let b = self.col1.y;
        let c = self.col2.x;
        let d = self.col2.y;

        let det = a * d - b * c;
        debug_assert!(
            det.abs() > f32::EPSILON,
            "attempted to invert singular or near-singular Mat22"
        );

        let det = 1.0 / det;
        Self::new(Vec2::new(det * d, -det * b), Vec2::new(-det * c, det * a))
    }

    #[inline]
    pub fn abs(self) -> Self {
        Self::new(self.col1.abs(), self.col2.abs())
    }
}

impl Mul<Vec2> for Mat22 {
    type Output = Vec2;
    #[inline]
    fn mul(self, rhs: Vec2) -> Vec2 {
        self.col1 * rhs.x + self.col2 * rhs.y
    }
}

impl Add for Mat22 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.col1 + rhs.col1, self.col2 + rhs.col2)
    }
}

impl Mul for Mat22 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self::new(self * rhs.col1, self * rhs.col2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn from_angle_zero_is_identity() {
        let r = Mat22::from_angle(0.0);
        // Identity in column-major form: col1=(1,0), col2=(0,1)
        assert_relative_eq!(r.col1.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(r.col1.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(r.col2.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(r.col2.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn rotation_90_deg() {
        let r = Mat22::from_angle(core::f32::consts::FRAC_PI_2);
        let v = Vec2::new(1.0, 0.0);
        let out = r * v;

        // (1,0) rotated by +90° -> (0,1)
        assert_relative_eq!(out.x, 0.0, epsilon = 1e-5);
        assert_relative_eq!(out.y, 1.0, epsilon = 1e-5);
    }

    #[test]
    fn transpose_is_inverse_for_rotation() {
        let r = Mat22::from_angle(0.3);
        let rt = r.transpose();

        let v = Vec2::new(2.0, -1.0);
        let out = rt * (r * v);

        assert_relative_eq!(out.x, v.x, epsilon = 1e-5);
        assert_relative_eq!(out.y, v.y, epsilon = 1e-5);
    }

    #[test]
    fn transpose_is_correct_layout() {
        // A = [a b; c d] with col1=(a,c), col2=(b,d)
        let a = Mat22::new(Vec2::new(1.0, 3.0), Vec2::new(2.0, 4.0));
        // A^T should be [a c; b d] => col1=(a,b)=(1,2), col2=(c,d)=(3,4)
        let t = a.transpose();

        assert_relative_eq!(t.col1.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(t.col1.y, 2.0, epsilon = 1e-6);
        assert_relative_eq!(t.col2.x, 3.0, epsilon = 1e-6);
        assert_relative_eq!(t.col2.y, 4.0, epsilon = 1e-6);
    }

    #[test]
    fn invert_identity_check() {
        // A = diag(2,4)
        let a = Mat22::new(Vec2::new(2.0, 0.0), Vec2::new(0.0, 4.0));
        let inv = a.invert();

        let v = Vec2::new(2.0, 8.0);
        let out = inv * (a * v);

        assert_relative_eq!(out.x, v.x, epsilon = 1e-6);
        assert_relative_eq!(out.y, v.y, epsilon = 1e-6);
    }

    #[test]
    fn add_and_mul() {
        let a = Mat22::new(Vec2::new(1.0, 2.0), Vec2::new(3.0, 4.0));
        let b = Mat22::new(Vec2::new(5.0, 6.0), Vec2::new(7.0, 8.0));

        let c = a + b;
        assert_relative_eq!(c.col1.x, 6.0);
        assert_relative_eq!(c.col1.y, 8.0);
        assert_relative_eq!(c.col2.x, 10.0);
        assert_relative_eq!(c.col2.y, 12.0);

        // Validate matrix multiplication: (A*B).col1 == A*B.col1, etc.
        let ab = a * b;
        let col1 = a * b.col1;
        let col2 = a * b.col2;

        assert_relative_eq!(ab.col1.x, col1.x, epsilon = 1e-6);
        assert_relative_eq!(ab.col1.y, col1.y, epsilon = 1e-6);
        assert_relative_eq!(ab.col2.x, col2.x, epsilon = 1e-6);
        assert_relative_eq!(ab.col2.y, col2.y, epsilon = 1e-6);
    }

    #[test]
    fn abs_works() {
        let a = Mat22::new(Vec2::new(-1.0, 2.0), Vec2::new(3.0, -4.0));
        let b = a.abs();
        assert_relative_eq!(b.col1.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(b.col1.y, 2.0, epsilon = 1e-6);
        assert_relative_eq!(b.col2.x, 3.0, epsilon = 1e-6);
        assert_relative_eq!(b.col2.y, 4.0, epsilon = 1e-6);
    }
}
