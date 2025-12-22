use core::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn set(&mut self, x: f32, y: f32) {
        self.x = x;
        self.y = y;
    }

    #[inline]
    pub fn length(self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    #[inline]
    pub fn cross(self, other: Self) -> f32 {
        self.x * other.y - self.y * other.x
    }

    #[inline]
    pub fn cross_vec_scalar(a: Self, s: f32) -> Self {
        Self::new(s * a.y, -s * a.x)
    }

    #[inline]
    pub fn cross_scalar_vec(s: f32, a: Self) -> Self {
        Self::new(-s * a.y, s * a.x)
    }

    #[inline]
    pub fn abs(self) -> Self {
        Self::new(self.x.abs(), self.y.abs())
    }
}

impl Neg for Vec2 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl Add for Vec2 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Vec2 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl MulAssign<f32> for Vec2 {
    #[inline]
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl Mul<Vec2> for f32 {
    type Output = Vec2;
    #[inline]
    fn mul(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self * rhs.x, self * rhs.y)
    }
}

// All tests are written by ChatGPT 5.2.
#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn new_and_set() {
        let mut v = Vec2::new(1.0, 2.0);
        assert_relative_eq!(v.x, 1.0);
        assert_relative_eq!(v.y, 2.0);

        v.set(-3.0, 4.5);
        assert_relative_eq!(v.x, -3.0);
        assert_relative_eq!(v.y, 4.5);
    }

    #[test]
    fn add_sub_neg_mul() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(3.0, -4.0);

        let c = a + b;
        assert_relative_eq!(c.x, 4.0);
        assert_relative_eq!(c.y, -2.0);

        let d = a - b;
        assert_relative_eq!(d.x, -2.0);
        assert_relative_eq!(d.y, 6.0);

        let e = -a;
        assert_relative_eq!(e.x, -1.0);
        assert_relative_eq!(e.y, -2.0);

        let f = a * 2.0;
        assert_relative_eq!(f.x, 2.0);
        assert_relative_eq!(f.y, 4.0);

        let g = 2.0 * a;
        assert_relative_eq!(g.x, 2.0);
        assert_relative_eq!(g.y, 4.0);
    }

    #[test]
    fn add_assign_sub_assign_mul_assign() {
        let mut v = Vec2::new(1.0, 2.0);
        v += Vec2::new(3.0, 4.0);
        assert_relative_eq!(v.x, 4.0);
        assert_relative_eq!(v.y, 6.0);

        v -= Vec2::new(1.0, 2.0);
        assert_relative_eq!(v.x, 3.0);
        assert_relative_eq!(v.y, 4.0);

        v *= 0.5;
        assert_relative_eq!(v.x, 1.5);
        assert_relative_eq!(v.y, 2.0);
    }

    #[test]
    fn dot_cross_length() {
        let v = Vec2::new(3.0, 4.0);
        assert_relative_eq!(v.length(), 5.0, epsilon = 1e-6);
        assert_relative_eq!(v.dot(v), 25.0, epsilon = 1e-6);

        let a = Vec2::new(1.0, 0.0);
        let b = Vec2::new(0.0, 1.0);
        assert_relative_eq!(a.cross(b), 1.0, epsilon = 1e-6);
        assert_relative_eq!(b.cross(a), -1.0, epsilon = 1e-6);
    }

    #[test]
    fn cross_scalar_forms_match_box2d_lite() {
        let a = Vec2::new(2.0, 3.0);
        let s = 5.0;

        // Cross(a, s) = (s*a.y, -s*a.x)
        let v1 = Vec2::cross_vec_scalar(a, s);
        assert_relative_eq!(v1.x, s * a.y, epsilon = 1e-6);
        assert_relative_eq!(v1.y, -s * a.x, epsilon = 1e-6);

        // Cross(s, a) = (-s*a.y, s*a.x)
        let v2 = Vec2::cross_scalar_vec(s, a);
        assert_relative_eq!(v2.x, -s * a.y, epsilon = 1e-6);
        assert_relative_eq!(v2.y, s * a.x, epsilon = 1e-6);
    }

    #[test]
    fn abs_works() {
        let v = Vec2::new(-1.25, 2.5);
        let a = v.abs();
        assert_relative_eq!(a.x, 1.25, epsilon = 1e-6);
        assert_relative_eq!(a.y, 2.5, epsilon = 1e-6);
    }
}
