//! Probabilistic occupancy configuration and cached log-odds arithmetic.

use crate::CuOccupancy;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Dimensionless inverse sensor probabilities and classification thresholds.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuOccupancyUpdateParams {
    /// Probability contributed by a hit; must exceed 0.5.
    pub p_hit: f64,
    /// Probability contributed by a miss; must be below 0.5.
    pub p_miss: f64,
    /// Minimum accumulated probability. The default 0.09 keeps one hit
    /// from lifting saturated free evidence above the free threshold.
    pub clamp_min: f64,
    /// Maximum accumulated probability.
    pub clamp_max: f64,
    /// Probabilities at or above this value classify as occupied.
    pub occupied_threshold: f64,
    /// Probabilities at or below this value classify as free.
    pub free_threshold: f64,
}

impl Default for CuOccupancyUpdateParams {
    fn default() -> Self {
        Self {
            p_hit: 0.7,
            p_miss: 0.4,
            clamp_min: 0.09,
            clamp_max: 0.97,
            occupied_threshold: 0.65,
            free_threshold: 0.2,
        }
    }
}

impl CuOccupancyUpdateParams {
    /// Checks finite probabilities in (0, 1), directional evidence, and
    /// clamp_min <= free_threshold < 0.5 < occupied_threshold <= clamp_max.
    /// Also rejects values whose f32 log-odds lose these relationships, and
    /// evidence too weak to move a cell away from either clamp in f32.
    pub fn validate(&self) -> CuResult<()> {
        for p in [
            self.p_hit,
            self.p_miss,
            self.clamp_min,
            self.clamp_max,
            self.occupied_threshold,
            self.free_threshold,
        ] {
            if !p.is_finite() || p <= 0.0 || p >= 1.0 {
                return Err(CuError::from(
                    "meander-grid: probabilities must be finite and in (0, 1)",
                ));
            }
        }
        let m = UpdateModel::new(self);
        if !(self.clamp_min <= self.free_threshold
            && self.free_threshold < 0.5
            && self.occupied_threshold > 0.5
            && self.occupied_threshold <= self.clamp_max
            && self.p_miss < 0.5
            && self.p_hit > 0.5
            && m.min <= m.free
            && m.free < 0.0
            && m.occupied > 0.0
            && m.occupied <= m.max
            && m.miss < 0.0
            && m.hit > 0.0
            // Each update must still move a saturated cell in f32, or it could never recover.
            && m.min + m.hit > m.min
            && m.max + m.miss < m.max)
        {
            return Err(CuError::from(
                "meander-grid: invalid occupancy evidence, clamps or thresholds",
            ));
        }
        Ok(())
    }
}

/// Logistic function, branched on sign so neither side overflows `exp`.
pub(crate) fn sigmoid(l: f64) -> f64 {
    if l >= 0.0 {
        1.0 / (1.0 + libm::exp(-l))
    } else {
        let e = libm::exp(l);
        e / (1.0 + e)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct UpdateModel {
    pub(crate) hit: f32,
    pub(crate) miss: f32,
    min: f32,
    max: f32,
    free: f32,
    occupied: f32,
}

impl UpdateModel {
    pub(crate) fn new(p: &CuOccupancyUpdateParams) -> Self {
        Self {
            hit: odds(p.p_hit),
            miss: odds(p.p_miss),
            min: odds(p.clamp_min),
            max: odds(p.clamp_max),
            free: odds(p.free_threshold),
            occupied: odds(p.occupied_threshold),
        }
    }

    /// Whether `value` is a log-odds this model could have produced.
    pub(crate) fn admits(&self, value: f32) -> bool {
        value.is_finite() && value >= self.min && value <= self.max
    }

    pub(crate) fn classify(&self, value: f32) -> CuOccupancy {
        if value <= self.free {
            CuOccupancy::Free
        } else if value >= self.occupied {
            CuOccupancy::Occupied
        } else {
            CuOccupancy::Unknown
        }
    }

    pub(crate) fn value(&self, state: CuOccupancy) -> f32 {
        match state {
            CuOccupancy::Unknown => 0.0,
            CuOccupancy::Free => self.min,
            CuOccupancy::Occupied => self.max,
        }
    }

    pub(crate) fn add(&self, value: &mut f32, delta: f32) -> CuOccupancy {
        *value = (*value + delta).clamp(self.min, self.max);
        self.classify(*value)
    }
}

fn odds(p: f64) -> f32 {
    (libm::log(p) - libm::log1p(-p)) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_inclusive_thresholds() {
        let model = UpdateModel::new(&CuOccupancyUpdateParams::default());
        assert_eq!(model.classify(model.free), CuOccupancy::Free);
        assert_eq!(model.classify(model.occupied), CuOccupancy::Occupied);
        assert_eq!(model.classify(model.free + 1e-5), CuOccupancy::Unknown);
        assert_eq!(model.classify(model.occupied - 1e-5), CuOccupancy::Unknown);
        assert_eq!(model.classify(0.0), CuOccupancy::Unknown);
    }

    #[test]
    fn test_rejects_evidence_too_weak_to_leave_a_clamp() {
        let weak_hit = CuOccupancyUpdateParams {
            p_hit: 0.500_000_01,
            ..CuOccupancyUpdateParams::default()
        };
        assert!(weak_hit.validate().is_err());
        let weak_miss = CuOccupancyUpdateParams {
            p_miss: 0.499_999_99,
            ..CuOccupancyUpdateParams::default()
        };
        assert!(weak_miss.validate().is_err());
    }

    #[test]
    fn test_sigmoid_stays_positive_for_extreme_log_odds() {
        assert!(sigmoid(-713.8) > 0.0);
        assert!(sigmoid(713.8) <= 1.0);
        assert!((sigmoid(0.0) - 0.5).abs() < 1e-12);
        assert!((sigmoid(-2.0) + sigmoid(2.0) - 1.0).abs() < 1e-12);
    }
}
