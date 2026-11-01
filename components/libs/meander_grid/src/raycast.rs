//! Allocation-free grid traversal (Amanatides & Woo, 1987).

/// Integer cell of continuous grid coordinates, or `None` outside `dims`.
pub(crate) fn cell_of<const K: usize>(p: [f64; K], dims: [usize; K]) -> Option<[usize; K]> {
    let mut cell = [0usize; K];
    for k in 0..K {
        let f = libm::floor(p[k]);
        if !f.is_finite() || f < 0.0 || f >= dims[k] as f64 {
            return None;
        }
        cell[k] = f as usize;
    }
    Some(cell)
}

/// Visits every cell the segment `start → end` passes through, in order,
/// starting with the cell containing `start` (which must be inside `dims`).
/// Stops at the cell containing `end` or where the segment leaves the grid.
/// Returns `false` if `start` is outside the grid.
pub(crate) fn traverse<const K: usize>(
    start: [f64; K],
    end: [f64; K],
    dims: [usize; K],
    mut visit: impl FnMut([usize; K]),
) -> bool {
    let Some(mut cell) = cell_of(start, dims) else {
        return false;
    };
    let mut target = [0i64; K];
    let mut step = [0i64; K];
    let mut t_max = [f64::INFINITY; K];
    let mut t_delta = [f64::INFINITY; K];
    let mut max_steps = 0usize;
    for k in 0..K {
        let f = libm::floor(end[k]);
        target[k] = if f.is_finite() { f as i64 } else { i64::MAX };
        let d = end[k] - start[k];
        if d > 0.0 {
            step[k] = 1;
            t_max[k] = (cell[k] as f64 + 1.0 - start[k]) / d;
            t_delta[k] = 1.0 / d;
        } else if d < 0.0 {
            step[k] = -1;
            t_max[k] = (start[k] - cell[k] as f64) / -d;
            t_delta[k] = -1.0 / d;
        }
        max_steps += dims[k];
    }
    for _ in 0..=max_steps {
        visit(cell);
        if (0..K).all(|k| cell[k] as i64 == target[k]) {
            break;
        }
        let mut axis = 0;
        for k in 1..K {
            if t_max[k] < t_max[axis] {
                axis = k;
            }
        }
        if t_max[axis] > 1.0 {
            break;
        }
        let next = cell[axis] as i64 + step[axis];
        if next < 0 || next >= dims[axis] as i64 {
            break;
        }
        cell[axis] = next as usize;
        t_max[axis] += t_delta[axis];
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_traverse_diagonal_is_connected() {
        let mut last: Option<[usize; 2]> = None;
        let mut count = 0;
        assert!(traverse([0.5, 0.5], [4.5, 2.5], [8, 8], |c| {
            if let Some(p) = last {
                let manhattan = p[0].abs_diff(c[0]) + p[1].abs_diff(c[1]);
                assert_eq!(manhattan, 1, "{p:?} -> {c:?}");
            }
            last = Some(c);
            count += 1;
        }));
        assert_eq!(last, Some([4, 2]));
        assert_eq!(count, 7);
    }

    #[test]
    fn test_traverse_stops_at_border() {
        let mut last = None;
        assert!(traverse([1.5, 1.5], [-10.0, 1.5], [4, 4], |c| last = Some(c)));
        assert_eq!(last, Some([0, 1]));
    }

    #[test]
    fn test_traverse_rejects_outside_start() {
        assert!(!traverse([-1.0, 0.0], [1.0, 1.0], [4, 4], |_| {}));
    }

    #[test]
    fn test_traverse_3d_reaches_target() {
        let mut last = None;
        assert!(traverse([0.2, 0.2, 0.2], [3.7, 1.1, 2.9], [4, 4, 4], |c| {
            last = Some(c)
        }));
        assert_eq!(last, Some([3, 1, 2]));
    }
}
