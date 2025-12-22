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

// All tests are written by ChatGPT 5.2.
#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn sign_nonzero_contract() {
        assert_relative_eq!(sign_nonzero(-1.0), -1.0);
        assert_relative_eq!(sign_nonzero(-0.0001), -1.0);

        // Must never return 0
        assert_relative_eq!(sign_nonzero(0.0), 1.0);
        assert_relative_eq!(sign_nonzero(0.0001), 1.0);
        assert_relative_eq!(sign_nonzero(42.0), 1.0);
    }
}
