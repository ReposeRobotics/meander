//! Bidirectional Dijkstra over a Nav2-style cost grid.

use crate::params::CuPlannerParams;
use crate::report::CuPlanReport;
use crate::report::CuPlanStatus;
use core::f64::consts::SQRT_2;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::CuPoint2;
use meander_core::CuPose2;
use meander_grid::CuCell2;
use meander_grid::CuGridGeometry2;

/// Nav2 cost of an unknown cell.
const UNKNOWN: u8 = 255;
/// Normalizer of the costmap value in the edge cost.
const MAX_COST: f64 = 252.0;
/// Parent and heap-position sentinel: "none".
const NONE: u32 = u32::MAX;
/// 8-connected neighbor offsets in `(dx, dy)` grid steps.
const NEIGHBORS: [(i64, i64); 8] = [
    (-1, -1),
    (0, -1),
    (1, -1),
    (-1, 0),
    (1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
];

/// One heap entry: the best known distance to a cell.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct HeapEntry {
    cost: f64,
    /// Flat cell index `y * W + x`.
    cell: u32,
}

/// Scratch state of one search direction, reused between [`plan`](crate::CuBidirectionalDijkstra::plan) calls.
#[derive(Debug, Clone, PartialEq)]
struct PlanSide<const W: usize, const H: usize> {
    dist: [[f64; W]; H],
    parent: [[u32; W]; H],
    closed: [[bool; W]; H],
    heap: [[HeapEntry; W]; H],
    /// Heap position of each cell, or [`NONE`] when absent.
    pos: [[u32; W]; H],
    len: usize,
}

impl<const W: usize, const H: usize> PlanSide<W, H> {
    fn reset(&mut self) {
        for row in &mut self.dist {
            row.fill(f64::INFINITY);
        }
        for row in &mut self.parent {
            row.fill(NONE);
        }
        for row in &mut self.closed {
            row.fill(false);
        }
        for row in &mut self.pos {
            row.fill(NONE);
        }
        self.len = 0;
    }

    /// Smallest queued distance, or infinity when the queue is empty.
    fn top(&self) -> f64 {
        if self.len == 0 {
            f64::INFINITY
        } else {
            self.heap[0][0].cost
        }
    }

    fn at(&self, i: usize) -> HeapEntry {
        self.heap[i / W][i % W]
    }

    fn set(&mut self, i: usize, entry: HeapEntry) {
        self.heap[i / W][i % W] = entry;
        let c = entry.cell as usize;
        self.pos[c / W][c % W] = i as u32;
    }

    fn swap_entries(&mut self, i: usize, j: usize) {
        let (a, b) = (self.at(i), self.at(j));
        self.set(i, b);
        self.set(j, a);
    }

    fn sift_up(&mut self, mut i: usize) {
        while i > 0 {
            let parent = (i - 1) / 2;
            if self.at(parent).cost <= self.at(i).cost {
                break;
            }
            self.swap_entries(parent, i);
            i = parent;
        }
    }

    fn sift_down(&mut self, mut i: usize) {
        loop {
            let left = 2 * i + 1;
            if left >= self.len {
                break;
            }
            let right = left + 1;
            let mut smallest = left;
            if right < self.len && self.at(right).cost < self.at(left).cost {
                smallest = right;
            }
            if self.at(i).cost <= self.at(smallest).cost {
                break;
            }
            self.swap_entries(i, smallest);
            i = smallest;
        }
    }

    /// Queues `cell`, or lowers its key when already queued. Each cell is a
    /// member at most once, so the heap never exceeds its `W * H` capacity.
    fn push_or_decrease(&mut self, cell: u32, cost: f64) {
        let c = cell as usize;
        let queued = self.pos[c / W][c % W];
        if queued != NONE {
            let i = queued as usize;
            if cost < self.at(i).cost {
                self.set(i, HeapEntry { cost, cell });
                self.sift_up(i);
            }
            return;
        }
        if self.len >= W * H {
            return; // Unreachable: members are distinct cells, so one more always fits.
        }
        let i = self.len;
        self.len += 1;
        self.set(i, HeapEntry { cost, cell });
        self.sift_up(i);
    }

    fn pop_min(&mut self) -> Option<(u32, f64)> {
        if self.len == 0 {
            return None;
        }
        let top = self.at(0);
        self.len -= 1;
        if self.len > 0 {
            let last = self.at(self.len);
            self.set(0, last);
            self.sift_down(0);
        }
        let c = top.cell as usize;
        self.pos[c / W][c % W] = NONE;
        Some((top.cell, top.cost))
    }
}

/// Borrowed search inputs shared by both directions.
struct SearchView<'a, const W: usize, const H: usize> {
    costs: &'a [[u8; W]; H],
    res: f64,
    cost_weight: f64,
    lethal_cost: u8,
    allow_unknown: bool,
}

impl<const W: usize, const H: usize> SearchView<'_, W, H> {
    fn blocked(&self, x: usize, y: usize) -> bool {
        let cost = self.costs[y][x];
        if cost == UNKNOWN {
            !self.allow_unknown
        } else {
            cost >= self.lethal_cost
        }
    }

    /// Settles the closest queued cell of `side`, relaxing its neighbors.
    /// Forward steps charge the entered cell; reverse (goal-side) steps
    /// charge the cell expanded from, which is the forward cost of the same
    /// edge traversed backwards. `best` stays the cheapest true start-goal
    /// path through any settled cell.
    fn expand(
        &self,
        side: &mut PlanSide<W, H>,
        other_dist: &[[f64; W]; H],
        reverse: bool,
        best: &mut (f64, u32),
    ) {
        let Some((u, du)) = side.pop_min() else {
            return;
        };
        let (ux, uy) = (u as usize % W, u as usize / W);
        side.closed[uy][ux] = true;
        let other = other_dist[uy][ux];
        if other < f64::INFINITY {
            let candidate = du + other;
            if candidate < best.0 {
                *best = (candidate, u);
            }
        }
        for (dx, dy) in NEIGHBORS {
            let nx = ux as i64 + dx;
            let ny = uy as i64 + dy;
            if nx < 0 || ny < 0 || nx >= W as i64 || ny >= H as i64 {
                continue;
            }
            let (nx, ny) = (nx as usize, ny as usize);
            if self.blocked(nx, ny) || side.closed[ny][nx] {
                continue;
            }
            if dx != 0 && dy != 0 && (self.blocked(ux, ny) || self.blocked(nx, uy)) {
                continue; // No corner cutting past blocked cells.
            }
            let step = if dx != 0 && dy != 0 {
                self.res * SQRT_2
            } else {
                self.res
            };
            let charged = if reverse {
                self.costs[uy][ux]
            } else {
                self.costs[ny][nx]
            };
            let contribution = if charged == UNKNOWN {
                0.0
            } else {
                f64::from(charged)
            };
            let edge = step * (1.0 + self.cost_weight * contribution / MAX_COST);
            let nd = du + edge;
            if nd < side.dist[ny][nx] {
                side.dist[ny][nx] = nd;
                side.parent[ny][nx] = u;
                side.push_or_decrease((ny * W + nx) as u32, nd);
            }
        }
    }
}

/// Bidirectional Dijkstra on an 8-connected `W × H` cost grid.
///
/// Costs follow Nav2: 0 is free, 1..=252 are inflated, 253 is inscribed, 254
/// is lethal and 255 is unknown. Diagonal steps cost √2 × resolution and
/// never cut corners past blocked cells. The edge into a cell costs
/// `step * (1 + cost_weight * cell_cost / 252)`.
///
/// The search expands alternately from start and goal and stops once the
/// cheapest path through any settled cell costs no more than the closest
/// queued cell on either side. All distance, parent, queue and path storage
/// is fixed-capacity and owned by the planner, so `plan` allocates nothing;
/// large grids belong in a `Box` or a `static`, not on the stack.
#[derive(Debug, Clone, PartialEq)]
pub struct CuBidirectionalDijkstra<const W: usize, const H: usize> {
    params: CuPlannerParams,
    forward: PlanSide<W, H>,
    backward: PlanSide<W, H>,
    /// Scratch: reconstructed flat cell indices, start to goal.
    path: [[u32; W]; H],
}

impl<const W: usize, const H: usize> CuBidirectionalDijkstra<W, H> {
    /// Creates a planner, validating the parameters and the grid dimensions.
    pub fn from_params(params: CuPlannerParams) -> CuResult<Self> {
        params.validate()?;
        if W == 0 || H == 0 {
            return Err(CuError::from(
                "meander-planner: grid dimensions must be >= 1",
            ));
        }
        if (W as u64) * (H as u64) > u64::from(u32::MAX) {
            return Err(CuError::from(
                "meander-planner: W * H must fit in a u32 cell index",
            ));
        }
        Ok(Self {
            params,
            forward: PlanSide {
                dist: [[f64::INFINITY; W]; H],
                parent: [[NONE; W]; H],
                closed: [[false; W]; H],
                heap: [[HeapEntry {
                    cost: 0.0,
                    cell: NONE,
                }; W]; H],
                pos: [[NONE; W]; H],
                len: 0,
            },
            backward: PlanSide {
                dist: [[f64::INFINITY; W]; H],
                parent: [[NONE; W]; H],
                closed: [[false; W]; H],
                heap: [[HeapEntry {
                    cost: 0.0,
                    cell: NONE,
                }; W]; H],
                pos: [[NONE; W]; H],
                len: 0,
            },
            path: [[NONE; W]; H],
        })
    }

    /// Validated planner parameters.
    pub fn params(&self) -> &CuPlannerParams {
        &self.params
    }

    /// Plans from `start` to `goal` over `costs` (row-major `[y][x]`).
    ///
    /// Waypoints are cell centers in world coordinates with headings along
    /// the path; the final heading is the goal heading. Errors only when
    /// `geometry` is invalid; every search outcome is a [`CuPlanStatus`].
    pub fn plan(
        &mut self,
        costs: &[[u8; W]; H],
        geometry: &CuGridGeometry2,
        start: &CuPose2,
        goal: &CuPose2,
        out: &mut [CuPose2],
    ) -> CuResult<CuPlanReport> {
        geometry.validate()?;
        let view = SearchView {
            costs,
            res: geometry.resolution.get::<meter>(),
            cost_weight: self.params.cost_weight,
            lethal_cost: self.params.lethal_cost,
            allow_unknown: self.params.allow_unknown,
        };
        let failure = |status| {
            Ok(CuPlanReport {
                status,
                len: 0,
                cost: Length::new::<meter>(0.0),
            })
        };
        let Some(start_cell) = cell_of(geometry, start.position(), W, H) else {
            return failure(CuPlanStatus::OutsideMap);
        };
        let Some(goal_cell) = cell_of(geometry, goal.position(), W, H) else {
            return failure(CuPlanStatus::OutsideMap);
        };
        if view.blocked(start_cell.x, start_cell.y) {
            return failure(CuPlanStatus::StartBlocked);
        }
        let goal_cell = if view.blocked(goal_cell.x, goal_cell.y) {
            let Some(free) = self.nearest_free(&view, geometry, goal.position()) else {
                return failure(CuPlanStatus::GoalBlocked);
            };
            free
        } else {
            goal_cell
        };
        let (start_flat, goal_flat) = (flat::<W>(start_cell), flat::<W>(goal_cell));
        let (count, cost) = if start_flat == goal_flat {
            self.path[0][0] = start_flat;
            (1, 0.0)
        } else {
            let Some(found) = self.search(&view, start_flat, goal_flat) else {
                return failure(CuPlanStatus::NoPath);
            };
            found
        };
        let write = count.min(out.len());
        for (i, slot) in out[..write].iter_mut().enumerate() {
            let center = cell_center(geometry, unflat::<W>(self.path[i / W][i % W]));
            let theta = if i + 1 < count {
                let next = cell_center(geometry, unflat::<W>(self.path[(i + 1) / W][(i + 1) % W]));
                Angle::new::<radian>(libm::atan2(
                    next.y.get::<meter>() - center.y.get::<meter>(),
                    next.x.get::<meter>() - center.x.get::<meter>(),
                ))
            } else {
                goal.theta
            };
            *slot = CuPose2::new(center.x, center.y, theta);
        }
        Ok(CuPlanReport {
            status: if count <= out.len() {
                CuPlanStatus::Found
            } else {
                CuPlanStatus::PathTooLong
            },
            len: count,
            cost: Length::new::<meter>(cost),
        })
    }

    /// Nearest traversable cell center within `goal_tolerance` of `goal`.
    fn nearest_free(
        &self,
        view: &SearchView<W, H>,
        geometry: &CuGridGeometry2,
        goal: CuPoint2,
    ) -> Option<CuCell2> {
        let tolerance = self.params.goal_tolerance.get::<meter>();
        let mut nearest: Option<CuCell2> = None;
        let mut nearest_d = f64::INFINITY;
        for y in 0..H {
            for x in 0..W {
                if view.blocked(x, y) {
                    continue;
                }
                let center = cell_center(geometry, CuCell2::new(x, y));
                let dx = center.x.get::<meter>() - goal.x.get::<meter>();
                let dy = center.y.get::<meter>() - goal.y.get::<meter>();
                let d = libm::hypot(dx, dy);
                if d <= tolerance && d < nearest_d {
                    nearest_d = d;
                    nearest = Some(CuCell2::new(x, y));
                }
            }
        }
        nearest
    }

    /// Runs the bidirectional search; returns the waypoint count and cost.
    fn search(&mut self, view: &SearchView<W, H>, start: u32, goal: u32) -> Option<(usize, f64)> {
        self.forward.reset();
        self.backward.reset();
        let (sx, sy) = (start as usize % W, start as usize / W);
        let (gx, gy) = (goal as usize % W, goal as usize / W);
        self.forward.dist[sy][sx] = 0.0;
        self.forward.push_or_decrease(start, 0.0);
        self.backward.dist[gy][gx] = 0.0;
        self.backward.push_or_decrease(goal, 0.0);
        let mut best = (f64::INFINITY, NONE);
        let mut forward_turn = false;
        loop {
            let bound = self.forward.top().min(self.backward.top());
            if best.0 <= bound {
                break;
            }
            let expand_forward = if self.forward.len == 0 {
                false
            } else if self.backward.len == 0 {
                true
            } else {
                forward_turn = !forward_turn;
                forward_turn
            };
            if expand_forward {
                let (side, other) = (&mut self.forward, &self.backward);
                view.expand(side, &other.dist, false, &mut best);
            } else {
                let (side, other) = (&mut self.backward, &self.forward);
                view.expand(side, &other.dist, true, &mut best);
            }
        }
        if best.1 == NONE {
            return None;
        }
        let count = self.reconstruct(best.1)?;
        Some((count, best.0))
    }

    /// Writes the start-to-goal cell chain through `meeting` into path scratch.
    fn reconstruct(&mut self, meeting: u32) -> Option<usize> {
        let mut count = 0;
        let mut cell = meeting;
        loop {
            if count >= W * H {
                return None; // Unreachable: parents strictly descend to the start.
            }
            self.path[count / W][count % W] = cell;
            count += 1;
            let (x, y) = (cell as usize % W, cell as usize / W);
            let parent = self.forward.parent[y][x];
            if parent == NONE {
                break;
            }
            cell = parent;
        }
        for i in 0..count / 2 {
            let (a, b) = (
                self.path[i / W][i % W],
                self.path[(count - 1 - i) / W][(count - 1 - i) % W],
            );
            self.path[i / W][i % W] = b;
            self.path[(count - 1 - i) / W][(count - 1 - i) % W] = a;
        }
        let (mx, my) = (meeting as usize % W, meeting as usize / W);
        let mut cell = self.backward.parent[my][mx];
        while cell != NONE {
            if count >= W * H {
                return None; // Unreachable: parents strictly descend to the goal.
            }
            self.path[count / W][count % W] = cell;
            count += 1;
            let (x, y) = (cell as usize % W, cell as usize / W);
            cell = self.backward.parent[y][x];
        }
        Some(count)
    }
}

/// Flat index `y * W + x` of a cell.
fn flat<const W: usize>(cell: CuCell2) -> u32 {
    (cell.y * W + cell.x) as u32
}

fn unflat<const W: usize>(flat: u32) -> CuCell2 {
    CuCell2::new(flat as usize % W, flat as usize / W)
}

/// The cell containing a world point, or `None` outside the grid.
fn cell_of(geometry: &CuGridGeometry2, point: CuPoint2, w: usize, h: usize) -> Option<CuCell2> {
    let r = geometry.resolution.get::<meter>();
    let fx = libm::floor((point.x.get::<meter>() - geometry.origin.x.get::<meter>()) / r);
    let fy = libm::floor((point.y.get::<meter>() - geometry.origin.y.get::<meter>()) / r);
    if !fx.is_finite()
        || !fy.is_finite()
        || fx < 0.0
        || fy < 0.0
        || fx >= w as f64
        || fy >= h as f64
    {
        return None;
    }
    Some(CuCell2::new(fx as usize, fy as usize))
}

/// World position of a cell's center.
fn cell_center(geometry: &CuGridGeometry2, cell: CuCell2) -> CuPoint2 {
    let r = geometry.resolution.get::<meter>();
    CuPoint2::new(
        Length::new::<meter>(geometry.origin.x.get::<meter>() + (cell.x as f64 + 0.5) * r),
        Length::new::<meter>(geometry.origin.y.get::<meter>() + (cell.y as f64 + 0.5) * r),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;
    use core::f64::consts::PI;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }

    fn pose(x: f64, y: f64, theta: f64) -> CuPose2 {
        CuPose2::new(m(x), m(y), Angle::new::<radian>(theta))
    }

    fn geometry(res: f64) -> CuGridGeometry2 {
        CuGridGeometry2::new(m(res), CuPoint2::new(m(0.0), m(0.0))).unwrap()
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-9 * b.abs().max(1.0)
    }

    /// Plain O(V²) unidirectional Dijkstra with the entering-cell edge model.
    fn reference_cost<const W: usize, const H: usize>(
        costs: &[[u8; W]; H],
        res: f64,
        params: &CuPlannerParams,
        start: (usize, usize),
        goal: (usize, usize),
    ) -> Option<f64> {
        let blocked = |x: usize, y: usize| {
            let cost = costs[y][x];
            if cost == UNKNOWN {
                !params.allow_unknown
            } else {
                cost >= params.lethal_cost
            }
        };
        if blocked(start.0, start.1) || blocked(goal.0, goal.1) {
            return None;
        }
        let mut dist = Vec::new();
        dist.resize(W * H, f64::INFINITY);
        let mut done = Vec::new();
        done.resize(W * H, false);
        dist[start.1 * W + start.0] = 0.0;
        for _ in 0..W * H {
            let mut u = W * H;
            let mut best = f64::INFINITY;
            for (i, &d) in dist.iter().enumerate() {
                if !done[i] && d < best {
                    best = d;
                    u = i;
                }
            }
            if u == W * H || best.is_infinite() {
                break;
            }
            if u == goal.1 * W + goal.0 {
                return Some(best);
            }
            done[u] = true;
            let (ux, uy) = (u % W, u / W);
            for (dx, dy) in NEIGHBORS {
                let nx = ux as i64 + dx;
                let ny = uy as i64 + dy;
                if nx < 0 || ny < 0 || nx >= W as i64 || ny >= H as i64 {
                    continue;
                }
                let (nx, ny) = (nx as usize, ny as usize);
                if blocked(nx, ny) || done[ny * W + nx] {
                    continue;
                }
                if dx != 0 && dy != 0 && (blocked(ux, ny) || blocked(nx, uy)) {
                    continue;
                }
                let step = if dx != 0 && dy != 0 {
                    res * SQRT_2
                } else {
                    res
                };
                let contribution = if costs[ny][nx] == UNKNOWN {
                    0.0
                } else {
                    f64::from(costs[ny][nx])
                };
                let nd = best + step * (1.0 + params.cost_weight * contribution / MAX_COST);
                if nd < dist[ny * W + nx] {
                    dist[ny * W + nx] = nd;
                }
            }
        }
        let goal = dist[goal.1 * W + goal.0];
        if goal.is_infinite() { None } else { Some(goal) }
    }

    /// Checks the waypoints form a legal cell walk from `start` to `goal`
    /// and add up to `cost` under the entering-cell edge model.
    fn check_walk<const W: usize, const H: usize>(
        costs: &[[u8; W]; H],
        res: f64,
        params: &CuPlannerParams,
        waypoints: &[CuPose2],
        start: (usize, usize),
        goal: (usize, usize),
        cost: f64,
    ) {
        let blocked = |x: usize, y: usize| {
            let c = costs[y][x];
            if c == UNKNOWN {
                !params.allow_unknown
            } else {
                c >= params.lethal_cost
            }
        };
        let cells: Vec<(usize, usize)> = waypoints
            .iter()
            .map(|p| {
                let x = libm::floor(p.x.get::<meter>() / res) as usize;
                let y = libm::floor(p.y.get::<meter>() / res) as usize;
                assert!(x < W && y < H, "waypoint outside the map");
                assert!(!blocked(x, y), "waypoint on a blocked cell");
                assert_eq!(p.x.get::<meter>(), (x as f64 + 0.5) * res);
                assert_eq!(p.y.get::<meter>(), (y as f64 + 0.5) * res);
                (x, y)
            })
            .collect();
        assert_eq!(cells.first(), Some(&start));
        assert_eq!(cells.last(), Some(&goal));
        let mut total = 0.0;
        for pair in cells.windows(2) {
            let [(ax, ay), (bx, by)] = [(pair[0].0, pair[0].1), (pair[1].0, pair[1].1)];
            let (dx, dy) = (ax.abs_diff(bx), ay.abs_diff(by));
            assert!(
                dx <= 1 && dy <= 1 && (dx, dy) != (0, 0),
                "non-adjacent step"
            );
            if dx == 1 && dy == 1 {
                assert!(!blocked(ax, by) && !blocked(bx, ay), "corner cut");
            }
            let step = if dx == 1 && dy == 1 {
                res * SQRT_2
            } else {
                res
            };
            let contribution = if costs[by][bx] == UNKNOWN {
                0.0
            } else {
                f64::from(costs[by][bx])
            };
            total += step * (1.0 + params.cost_weight * contribution / MAX_COST);
        }
        assert!(
            close(total, cost),
            "walk cost {total} != report cost {cost}"
        );
    }

    #[test]
    fn test_from_params_rejects_bad_inputs() {
        assert!(CuBidirectionalDijkstra::<0, 8>::from_params(CuPlannerParams::default()).is_err());
        assert!(CuBidirectionalDijkstra::<8, 0>::from_params(CuPlannerParams::default()).is_err());
        let params = CuPlannerParams {
            cost_weight: -1.0,
            ..CuPlannerParams::default()
        };
        assert!(CuBidirectionalDijkstra::<8, 8>::from_params(params).is_err());
    }

    #[test]
    fn test_open_grid_diagonal_is_optimal() {
        let geometry = geometry(1.0);
        let mut planner =
            CuBidirectionalDijkstra::<8, 8>::from_params(CuPlannerParams::default()).unwrap();
        let costs = [[0u8; 8]; 8];
        let mut out = [CuPose2::default(); 64];
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(7.5, 7.5, PI / 2.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::Found);
        assert_eq!(report.len, 8);
        assert!(close(report.cost.get::<meter>(), 7.0 * SQRT_2));
        let expected = reference_cost(&costs, 1.0, planner.params(), (0, 0), (7, 7)).unwrap();
        assert!(close(report.cost.get::<meter>(), expected));
        assert_eq!(out[0].x.get::<meter>(), 0.5);
        assert_eq!(out[0].y.get::<meter>(), 0.5);
        assert!(close(out[0].theta.get::<radian>(), PI / 4.0));
        assert_eq!(out[7].x.get::<meter>(), 7.5);
        assert_eq!(out[7].y.get::<meter>(), 7.5);
        assert!(close(out[7].theta.get::<radian>(), PI / 2.0));
        check_walk(
            &costs,
            1.0,
            planner.params(),
            &out[..8],
            (0, 0),
            (7, 7),
            expected,
        );
    }

    #[test]
    fn test_open_grid_straight_is_optimal() {
        let geometry = geometry(0.5);
        let mut planner =
            CuBidirectionalDijkstra::<8, 8>::from_params(CuPlannerParams::default()).unwrap();
        let costs = [[0u8; 8]; 8];
        let mut out = [CuPose2::default(); 64];
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.25, 1.75, 0.0),
                &pose(3.75, 1.75, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::Found);
        assert_eq!(report.len, 8);
        assert!(close(report.cost.get::<meter>(), 3.5));
        let expected = reference_cost(&costs, 0.5, planner.params(), (0, 3), (7, 3)).unwrap();
        assert!(close(report.cost.get::<meter>(), expected));
        assert!(out[..8].iter().all(|p| close(p.theta.get::<radian>(), 0.0)));
    }

    #[test]
    fn test_wall_with_gap_routes_through_gap() {
        let geometry = geometry(1.0);
        let mut planner =
            CuBidirectionalDijkstra::<10, 10>::from_params(CuPlannerParams::default()).unwrap();
        let mut costs = [[0u8; 10]; 10];
        for (y, row) in costs.iter_mut().enumerate() {
            if y != 5 {
                row[5] = 254;
            }
        }
        let mut out = [CuPose2::default(); 100];
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(9.5, 9.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::Found);
        assert!(
            out[..report.len]
                .iter()
                .any(|p| p.x.get::<meter>() == 5.5 && p.y.get::<meter>() == 5.5),
            "path misses the gap"
        );
        let expected = reference_cost(&costs, 1.0, planner.params(), (0, 0), (9, 9)).unwrap();
        assert!(close(report.cost.get::<meter>(), expected));
        check_walk(
            &costs,
            1.0,
            planner.params(),
            &out[..report.len],
            (0, 0),
            (9, 9),
            expected,
        );
    }

    #[test]
    fn test_wall_without_gap_has_no_path() {
        let geometry = geometry(1.0);
        let mut planner =
            CuBidirectionalDijkstra::<10, 10>::from_params(CuPlannerParams::default()).unwrap();
        let mut costs = [[0u8; 10]; 10];
        for row in &mut costs {
            row[5] = 254;
        }
        let mut out = [CuPose2::default(); 100];
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(9.5, 9.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::NoPath);
        assert_eq!(report.len, 0);
        assert_eq!(report.cost.get::<meter>(), 0.0);
        assert_eq!(
            reference_cost(&costs, 1.0, planner.params(), (0, 0), (9, 9)),
            None
        );
    }

    #[test]
    fn test_blocked_start_goal_and_outside_map() {
        let geometry = geometry(1.0);
        let mut planner =
            CuBidirectionalDijkstra::<8, 8>::from_params(CuPlannerParams::default()).unwrap();
        let mut costs = [[0u8; 8]; 8];
        let mut out = [CuPose2::default(); 64];
        costs[0][0] = 254;
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(7.5, 7.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::StartBlocked);
        assert_eq!(report.len, 0);
        costs[0][0] = 0;
        costs[7][7] = 254;
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(7.5, 7.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::GoalBlocked);
        assert_eq!(report.len, 0);
        costs[7][7] = 0;
        for (start, goal) in [
            (pose(-0.5, 0.5, 0.0), pose(7.5, 7.5, 0.0)),
            (pose(0.5, 0.5, 0.0), pose(8.5, 0.5, 0.0)),
            (pose(f64::NAN, 0.5, 0.0), pose(7.5, 7.5, 0.0)),
        ] {
            let report = planner
                .plan(&costs, &geometry, &start, &goal, &mut out)
                .unwrap();
            assert_eq!(report.status, CuPlanStatus::OutsideMap);
            assert_eq!(report.len, 0);
        }
        let bad = CuGridGeometry2 {
            resolution: m(0.0),
            origin: CuPoint2::new(m(0.0), m(0.0)),
        };
        assert!(
            planner
                .plan(
                    &costs,
                    &bad,
                    &pose(0.5, 0.5, 0.0),
                    &pose(7.5, 7.5, 0.0),
                    &mut out
                )
                .is_err()
        );
    }

    #[test]
    fn test_goal_tolerance_finds_nearest_free_cell() {
        let geometry = geometry(1.0);
        let params = CuPlannerParams {
            goal_tolerance: m(2.0),
            ..CuPlannerParams::default()
        };
        let mut planner = CuBidirectionalDijkstra::<6, 6>::from_params(params).unwrap();
        let mut costs = [[0u8; 6]; 6];
        costs[5][5] = 254;
        costs[5][4] = 254;
        costs[4][5] = 254;
        let mut out = [CuPose2::default(); 36];
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(5.5, 5.5, -1.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::Found);
        let last = out[report.len - 1];
        assert_eq!(last.x.get::<meter>(), 4.5);
        assert_eq!(last.y.get::<meter>(), 4.5);
        assert!(close(last.theta.get::<radian>(), -1.0));

        let params = CuPlannerParams {
            goal_tolerance: m(1.0),
            ..CuPlannerParams::default()
        };
        let mut planner = CuBidirectionalDijkstra::<6, 6>::from_params(params).unwrap();
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(5.5, 5.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::GoalBlocked);
    }

    #[test]
    fn test_start_equals_goal_is_one_waypoint() {
        let geometry = geometry(1.0);
        let mut planner =
            CuBidirectionalDijkstra::<8, 8>::from_params(CuPlannerParams::default()).unwrap();
        let costs = [[0u8; 8]; 8];
        let mut out = [CuPose2::default(); 64];
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(2.5, 2.5, 1.0),
                &pose(2.7, 2.3, -0.5),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::Found);
        assert_eq!(report.len, 1);
        assert_eq!(report.cost.get::<meter>(), 0.0);
        assert_eq!(out[0].x.get::<meter>(), 2.5);
        assert_eq!(out[0].y.get::<meter>(), 2.5);
        assert!(close(out[0].theta.get::<radian>(), -0.5));
    }

    #[test]
    fn test_diagonal_never_cuts_blocked_corners() {
        let geometry = geometry(1.0);
        let mut planner =
            CuBidirectionalDijkstra::<2, 2>::from_params(CuPlannerParams::default()).unwrap();
        let costs = [[0, 254], [254, 0]];
        let mut out = [CuPose2::default(); 4];
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(1.5, 1.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::NoPath);

        let mut planner =
            CuBidirectionalDijkstra::<3, 3>::from_params(CuPlannerParams::default()).unwrap();
        let mut costs = [[0u8; 3]; 3];
        costs[0][1] = 254;
        let mut out = [CuPose2::default(); 9];
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(2.5, 2.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::Found);
        // The direct diagonal is cut-blocked, so the only first step is up.
        assert_eq!(out[1].x.get::<meter>(), 0.5);
        assert_eq!(out[1].y.get::<meter>(), 1.5);
        let expected = reference_cost(&costs, 1.0, planner.params(), (0, 0), (2, 2)).unwrap();
        assert!(close(report.cost.get::<meter>(), expected));
    }

    #[test]
    fn test_unknown_cells_need_opt_in() {
        let geometry = geometry(1.0);
        let mut costs = [[0u8; 5]; 5];
        for row in &mut costs {
            row[2] = 255;
        }
        let mut out = [CuPose2::default(); 25];
        let mut planner =
            CuBidirectionalDijkstra::<5, 5>::from_params(CuPlannerParams::default()).unwrap();
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 2.5, 0.0),
                &pose(4.5, 2.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::NoPath);

        let params = CuPlannerParams {
            allow_unknown: true,
            ..CuPlannerParams::default()
        };
        let mut planner = CuBidirectionalDijkstra::<5, 5>::from_params(params).unwrap();
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 2.5, 0.0),
                &pose(4.5, 2.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::Found);
        assert!(close(report.cost.get::<meter>(), 4.0));
    }

    #[test]
    fn test_cost_weight_steers_around_inflated_cells() {
        let geometry = geometry(1.0);
        let mut costs = [[0u8; 10]; 6];
        for cell in costs[2].iter_mut().skip(1).take(8) {
            *cell = 252;
        }
        let mut out = [CuPose2::default(); 60];
        let params = CuPlannerParams {
            cost_weight: 0.0,
            ..CuPlannerParams::default()
        };
        let mut planner = CuBidirectionalDijkstra::<10, 6>::from_params(params).unwrap();
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 2.5, 0.0),
                &pose(9.5, 2.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::Found);
        assert_eq!(report.len, 10);
        assert!(close(report.cost.get::<meter>(), 9.0));
        assert!(out[..10].iter().all(|p| p.y.get::<meter>() == 2.5));

        let params = CuPlannerParams {
            cost_weight: 10.0,
            ..CuPlannerParams::default()
        };
        let mut planner = CuBidirectionalDijkstra::<10, 6>::from_params(params).unwrap();
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 2.5, 0.0),
                &pose(9.5, 2.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::Found);
        assert!(
            out[..report.len].iter().all(|p| {
                let x = libm::floor(p.x.get::<meter>()) as usize;
                let y = libm::floor(p.y.get::<meter>()) as usize;
                costs[y][x] != 252
            }),
            "weighted path still uses the inflated corridor"
        );
        let expected = reference_cost(&costs, 1.0, planner.params(), (0, 2), (9, 2)).unwrap();
        assert!(close(report.cost.get::<meter>(), expected));
    }

    #[test]
    fn test_path_too_long_reports_full_length() {
        let geometry = geometry(1.0);
        let mut planner =
            CuBidirectionalDijkstra::<8, 8>::from_params(CuPlannerParams::default()).unwrap();
        let costs = [[0u8; 8]; 8];
        let mut out = [CuPose2::default(); 3];
        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(7.5, 7.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::PathTooLong);
        assert_eq!(report.len, 8);
        assert!(close(report.cost.get::<meter>(), 7.0 * SQRT_2));
        assert_eq!(out[0].x.get::<meter>(), 0.5);
        assert!(close(out[0].theta.get::<radian>(), PI / 4.0));
        assert_eq!(out[2].x.get::<meter>(), 2.5);
        assert_eq!(out[2].y.get::<meter>(), 2.5);
        assert!(close(out[2].theta.get::<radian>(), PI / 4.0));

        let report = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(7.5, 7.5, 0.0),
                &mut [],
            )
            .unwrap();
        assert_eq!(report.status, CuPlanStatus::PathTooLong);
        assert_eq!(report.len, 8);
    }

    #[test]
    fn test_planner_reuse_between_calls() {
        let geometry = geometry(1.0);
        let mut planner =
            CuBidirectionalDijkstra::<8, 8>::from_params(CuPlannerParams::default()).unwrap();
        let costs = [[0u8; 8]; 8];
        let mut out = [CuPose2::default(); 64];
        let first = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(7.5, 7.5, 0.0),
                &mut out,
            )
            .unwrap();
        let first_waypoints = out;
        let second = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 7.5, 0.0),
                &pose(7.5, 0.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(second.status, CuPlanStatus::Found);
        assert_eq!(second.len, 8);
        let mut blocked = [[0u8; 8]; 8];
        for row in &mut blocked {
            row[4] = 254;
        }
        let stuck = planner
            .plan(
                &blocked,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(7.5, 7.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(stuck.status, CuPlanStatus::NoPath);
        let repeat = planner
            .plan(
                &costs,
                &geometry,
                &pose(0.5, 0.5, 0.0),
                &pose(7.5, 7.5, 0.0),
                &mut out,
            )
            .unwrap();
        assert_eq!(repeat, first);
        assert_eq!(out, first_waypoints);
    }

    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x
        }

        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
    }

    #[test]
    fn test_matches_unidirectional_dijkstra_on_random_grids() {
        const W: usize = 12;
        const H: usize = 10;
        let geometry = geometry(1.0);
        let mut rng = Rng(0x1234_5678_9abc_def1);
        let mut out = [CuPose2::default(); W * H];
        let mut found = 0;
        for iter in 0..30 {
            let params = CuPlannerParams {
                cost_weight: [0.0, 0.5, 1.0, 3.0][iter % 4],
                lethal_cost: if iter % 3 == 0 { 254 } else { 253 },
                allow_unknown: iter % 2 == 0,
                ..CuPlannerParams::default()
            };
            let mut costs = [[0u8; W]; H];
            for row in &mut costs {
                for cell in row.iter_mut() {
                    *cell = match rng.below(100) {
                        0..=59 => 0,
                        60..=79 => (rng.below(252) + 1) as u8,
                        80..=89 => (253 + rng.below(2)) as u8,
                        _ => 255,
                    };
                }
            }
            let start = (rng.below(W as u64) as usize, rng.below(H as u64) as usize);
            let goal = (rng.below(W as u64) as usize, rng.below(H as u64) as usize);
            costs[start.1][start.0] = 0;
            costs[goal.1][goal.0] = 0;
            let mut planner = CuBidirectionalDijkstra::<W, H>::from_params(params.clone()).unwrap();
            let start_pose = pose(start.0 as f64 + 0.5, start.1 as f64 + 0.5, 0.0);
            let goal_pose = pose(goal.0 as f64 + 0.5, goal.1 as f64 + 0.5, 1.0);
            // Both directions on the same planner also exercise scratch reuse.
            for (from, to, cells) in [
                (start_pose, goal_pose, (start, goal)),
                (goal_pose, start_pose, (goal, start)),
            ] {
                let report = planner
                    .plan(&costs, &geometry, &from, &to, &mut out)
                    .unwrap();
                match reference_cost(&costs, 1.0, &params, cells.0, cells.1) {
                    Some(expected) => {
                        found += 1;
                        assert_eq!(report.status, CuPlanStatus::Found, "iter={iter}");
                        assert!(close(report.cost.get::<meter>(), expected), "iter={iter}");
                        check_walk(
                            &costs,
                            1.0,
                            &params,
                            &out[..report.len],
                            cells.0,
                            cells.1,
                            expected,
                        );
                        let last_theta = out[report.len - 1].theta.get::<radian>();
                        assert!(close(last_theta, to.theta.get::<radian>()), "iter={iter}");
                    }
                    None => assert_eq!(report.status, CuPlanStatus::NoPath, "iter={iter}"),
                }
            }
        }
        assert!(found > 0, "seeded grids never connected start and goal");
    }
}
