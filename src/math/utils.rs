use rand::Rng;

#[inline]
pub fn sign_nonzero(x: f32) -> f32 {
    if x < 0.0 { -1.0 } else { 1.0 }
}

/// Random number in [-1, 1]
#[inline]
pub fn random_unit(rng: &mut impl Rng) -> f32 {
    rng.gen_range(-1.0..=1.0)
}

#[inline]
pub fn random_range(rng: &mut impl Rng, lo: f32, hi: f32) -> f32 {
    rng.gen_range(lo..=hi)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    #[test]
    fn sign_nonzero_contract() {
        assert_relative_eq!(sign_nonzero(-1.0), -1.0);
        assert_relative_eq!(sign_nonzero(-0.0001), -1.0);

        // Must never return 0
        assert_relative_eq!(sign_nonzero(0.0), 1.0);
        assert_relative_eq!(sign_nonzero(0.0001), 1.0);
        assert_relative_eq!(sign_nonzero(42.0), 1.0);
    }

    #[test]
    fn random_unit_is_bounded() {
        let mut rng = StdRng::seed_from_u64(123);
        for _ in 0..10_000 {
            let v = random_unit(&mut rng);
            assert!(v >= -1.0 && v <= 1.0, "random_unit out of bounds: {v}");
        }
    }

    #[test]
    fn random_range_is_bounded_and_inclusive() {
        let mut rng = StdRng::seed_from_u64(456);
        let lo = -2.0;
        let hi = 3.25;
        for _ in 0..10_000 {
            let v = random_range(&mut rng, lo, hi);
            assert!(v >= lo && v <= hi, "random_range out of bounds: {v}");
        }
        // Degenerate interval should always return the endpoint.
        for _ in 0..100 {
            let v = random_range(&mut rng, 7.0, 7.0);
            assert_relative_eq!(v, 7.0);
        }
    }
}
