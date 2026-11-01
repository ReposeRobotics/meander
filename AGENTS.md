# Meander Agent Guidance

**Meander** (this repository) is a Copper-idiomatic (`cu29`) mobile robot
navigation stack, in the spirit of ROS 2 Nav2, built in strict alignment with
`copper-project/copper-rs`. It is a separate project from Repose
(`ReposeRobotics/repose`, the manipulation stack): Meander may depend on
`repose-*` crates, but never presents itself as part of Repose. Crates are
named `meander-*`.

## Skills

Before writing or reviewing code, read the applicable `SKILL.md` under `.agents/skills/`:

| Work | Skill |
|---|---|
| Architecture / reading apps | `.agents/skills/copper-arch/SKILL.md` |
| Rust / documentation / console or UI style | `.agents/skills/copper-coding-style/SKILL.md` |
| Core / runtime constraints | `.agents/skills/copper-core-dev/SKILL.md` |
| Public traits, adapters, config policies | `.agents/skills/copper-api-flavor/SKILL.md` |
| Sources, tasks, sinks, bridges | `.agents/skills/copper-component-design/SKILL.md` |
| RON config | `.agents/skills/copper-ron-config/SKILL.md` |
| Build, checks, justfiles, releases | `.agents/skills/copper-workflow/SKILL.md` |

Public API shape (params structs, constructors, units, visibility) follows `docs/api-conventions.md`.
Versioning, breaking-change rules, and releases follow `docs/versioning.md`; record user-visible changes under `Unreleased` in `CHANGELOG.md`.

## Non-Negotiable Copper House Rules

1. **Static over dynamic:** Grid dimensions and capacities are `const` generics; zero heap allocations on the real-time path.
2. **In-place `&mut` outputs (`copper-api-flavor`):** Hot-path methods write into caller-provided `&mut` buffers or `&mut Self::Output<'o>`. Never return payloads by value on the hot path.
3. **`std` / `no_std` discipline (`copper-coding-style`):** Shared crates start with `#![cfg_attr(not(feature = "std"), no_std)]`. Never use `std::` outside `#[cfg(feature = "std")]`.
4. **Physical units (`cu29::units`):** Public APIs and payloads use `cu29::units::si::f64::{Length, Angle, Velocity, AngularVelocity, Time}` rather than bare floats whose unit exists only in comments.
5. **Error handling:** Fallible operations return `CuResult<T>`. Never `unwrap()`, `expect()`, or `panic!` in library or task code.
6. **Payload & Task contracts:**
   - Payloads derive `#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]`.
   - `Tov` stays on the `CuMsg` envelope, never inside payload structs.
   - Tasks derive `Reflect` and implement `Freezable` + `CuTask` (or `CuAnytimeTask`).
7. **Workflow:** Run `just fmt` after edits and `just pr-check` before committing.

## Roadmap

See `ROADMAP.md`. 0.1 is deliberately small: `meander-core` (2D/3D geometry,
poses, twists) and `meander-grid` (2D occupancy grids, 3D voxel grids,
ray casting). Planners, controllers and costmap layers come later.
