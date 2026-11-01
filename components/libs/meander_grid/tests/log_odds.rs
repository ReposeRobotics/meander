//! Evidence accumulation and classified compatibility for planar and voxel maps.

use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::{CuPoint2, CuPoint3};
use meander_grid::{
    CuCell2, CuCell3, CuGridGeometry2, CuGridGeometry3, CuOccupancy, CuOccupancyGrid,
    CuOccupancyUpdateParams, CuVoxelGrid,
};

macro_rules! evidence_tests {
    ($name:ident, $grid:expr, $cell:expr, $outside:expr, $origin:expr, $hit:expr) => {
        #[test]
        fn $name() {
            let mut grid = $grid;
            let cell = $cell;
            assert_eq!(grid.params(), &CuOccupancyUpdateParams::default());
            let origin = $origin;
            let hit = $hit;
            assert_eq!(grid.probability(cell), Some(0.5));
            assert_eq!(grid.log_odds(cell), Some(0.0));
            assert_eq!(grid.get(cell), Some(CuOccupancy::Unknown));
            assert_eq!(grid.probability($outside), None);
            assert_eq!(grid.log_odds($outside), None);
            assert!(grid.set($outside, CuOccupancy::Free).is_err());
            for _ in 0..1000 {
                grid.clear_ray(origin, hit).unwrap();
            }
            let min = grid.log_odds(cell).unwrap();
            assert!((grid.probability(cell).unwrap() - 0.09).abs() < 1e-7);
            grid.insert_ray(origin, hit).unwrap();
            assert_eq!(grid.get(cell), Some(CuOccupancy::Free));
            for _ in 0..10 {
                grid.insert_ray(origin, hit).unwrap();
            }
            assert_eq!(grid.get(cell), Some(CuOccupancy::Occupied));
            let max = grid.log_odds(cell).unwrap();
            assert!((grid.probability(cell).unwrap() - 0.97).abs() < 1e-7);
            for _ in 0..1000 {
                grid.insert_ray(origin, hit).unwrap();
            }
            assert_eq!(grid.log_odds(cell), Some(max));
            for _ in 0..13 {
                grid.clear_ray(origin, hit).unwrap();
            }
            assert_eq!(grid.get(cell), Some(CuOccupancy::Free));
            assert!(grid.log_odds(cell).unwrap() >= min);
            grid.fill(CuOccupancy::Unknown);
            grid.clear_ray(origin, hit).unwrap();
            assert_eq!(grid.get(cell), Some(CuOccupancy::Unknown));
            for _ in 0..3 {
                grid.clear_ray(origin, hit).unwrap();
            }
            assert_eq!(grid.get(cell), Some(CuOccupancy::Free));
            grid.set(cell, CuOccupancy::Occupied).unwrap();
            assert_eq!(grid.log_odds(cell), Some(max));
            grid.fill(CuOccupancy::Free);
            assert_eq!(grid.log_odds(cell), Some(min));
            grid.set(cell, CuOccupancy::Unknown).unwrap();
            grid.insert_ray(origin, hit).unwrap();
            assert_eq!(grid.get(cell), Some(CuOccupancy::Occupied));
        }
    };
}

fn m(x: f64) -> Length {
    Length::new::<meter>(x)
}

evidence_tests!(
    test_planar_evidence,
    CuOccupancyGrid::<3, 1>::from_params(
        CuGridGeometry2::new(m(1.0), CuPoint2::default()).unwrap(),
        CuOccupancyUpdateParams::default()
    )
    .unwrap(),
    CuCell2::new(2, 0),
    CuCell2::new(3, 0),
    CuPoint2::new(m(0.5), m(0.5)),
    CuPoint2::new(m(2.5), m(0.5))
);

evidence_tests!(
    test_voxel_evidence,
    CuVoxelGrid::<3, 1, 1>::from_params(
        CuGridGeometry3::new(m(1.0), CuPoint3::default()).unwrap(),
        CuOccupancyUpdateParams::default()
    )
    .unwrap(),
    CuCell3::new(2, 0, 0),
    CuCell3::new(2, 0, 1),
    CuPoint3::new(m(0.5), m(0.5), m(0.5)),
    CuPoint3::new(m(2.5), m(0.5), m(0.5))
);

#[test]
fn test_params_ron_and_validation() {
    let defaults = CuOccupancyUpdateParams::default();
    defaults.validate().unwrap();
    assert_eq!(
        ron::from_str::<CuOccupancyUpdateParams>("()").unwrap(),
        defaults
    );
    assert_eq!(
        ron::from_str::<CuOccupancyUpdateParams>(&ron::to_string(&defaults).unwrap()).unwrap(),
        defaults
    );
    assert!(ron::from_str::<CuOccupancyUpdateParams>("(p_hitt: 0.7)").is_err());
    for index in 0..6 {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, 1.0, -0.1] {
            let mut p = defaults.clone();
            let fields = [
                &mut p.p_hit,
                &mut p.p_miss,
                &mut p.clamp_min,
                &mut p.clamp_max,
                &mut p.occupied_threshold,
                &mut p.free_threshold,
            ];
            *fields.into_iter().nth(index).unwrap() = value;
            assert!(p.validate().is_err());
        }
    }
    for p in [
        CuOccupancyUpdateParams {
            p_hit: 0.5,
            ..defaults.clone()
        },
        CuOccupancyUpdateParams {
            p_miss: 0.5,
            ..defaults.clone()
        },
        CuOccupancyUpdateParams {
            free_threshold: 0.5,
            ..defaults.clone()
        },
        CuOccupancyUpdateParams {
            occupied_threshold: 0.5,
            ..defaults.clone()
        },
        CuOccupancyUpdateParams {
            clamp_min: 0.3,
            ..defaults.clone()
        },
        CuOccupancyUpdateParams {
            clamp_max: 0.6,
            ..defaults.clone()
        },
    ] {
        assert!(p.validate().is_err());
        assert!(
            CuOccupancyGrid::<1, 1>::from_params(
                CuGridGeometry2::new(m(1.0), CuPoint2::default()).unwrap(),
                p.clone()
            )
            .is_err()
        );
        assert!(
            CuVoxelGrid::<1, 1, 1>::from_params(
                CuGridGeometry3::new(m(1.0), CuPoint3::default()).unwrap(),
                p
            )
            .is_err()
        );
    }
}

#[test]
fn test_custom_thresholds_and_raw_views() {
    let params = CuOccupancyUpdateParams {
        p_hit: 0.7,
        p_miss: 0.3,
        occupied_threshold: 0.7,
        free_threshold: 0.3,
        ..CuOccupancyUpdateParams::default()
    };
    let point = CuPoint2::new(m(0.5), m(0.5));
    let cell = CuCell2::new(0, 0);
    let mut planar = CuOccupancyGrid::<1, 1>::from_params(
        CuGridGeometry2::new(m(1.0), CuPoint2::default()).unwrap(),
        params.clone(),
    )
    .unwrap();
    planar.insert_ray(point, point).unwrap();
    assert_eq!(planar.get(cell), Some(CuOccupancy::Occupied));
    assert_eq!(
        planar.log_odds(cell),
        Some(planar.log_odds_rows()[0][0] as f64)
    );
    assert_eq!(planar.cells()[0][0], planar.get(cell).unwrap());
    planar.fill(CuOccupancy::Unknown);
    planar.clear_ray(point, point).unwrap();
    assert_eq!(planar.get(cell), Some(CuOccupancy::Free));

    let point = CuPoint3::new(m(0.5), m(0.5), m(0.5));
    let cell = CuCell3::new(0, 0, 0);
    let mut voxel = CuVoxelGrid::<1, 1, 1>::from_params(
        CuGridGeometry3::new(m(1.0), CuPoint3::default()).unwrap(),
        params,
    )
    .unwrap();
    voxel.insert_ray(point, point).unwrap();
    assert_eq!(voxel.get(cell), Some(CuOccupancy::Occupied));
    assert_eq!(
        voxel.log_odds(cell),
        Some(voxel.log_odds_rows()[0][0][0] as f64)
    );
    assert_eq!(voxel.cells()[0][0][0], voxel.get(cell).unwrap());
    voxel.project_into(m(0.0), m(1.0), &mut planar).unwrap();
    assert_eq!(planar.get(CuCell2::new(0, 0)), Some(CuOccupancy::Occupied));
    voxel.fill(CuOccupancy::Unknown);
    voxel.clear_ray(point, point).unwrap();
    assert_eq!(voxel.get(cell), Some(CuOccupancy::Free));
}
