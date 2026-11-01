# Versioning

Meander follows [Semantic Versioning](https://semver.org/) with the pre-1.0
rules below. This document is the source of truth; the README links here.

## One version for the whole workspace

- Every library crate inherits `version` from `[workspace.package]`. Crates are
  never versioned or tagged individually: "Meander 0.1" means every
  `meander-*` crate at `0.1.x`.
- Releases are tagged `vX.Y.Z` on `master`.

## What counts as breaking

Pre-1.0, a **minor** bump (`0.1 → 0.2`) is a breaking release and a **patch**
bump (`0.1.0 → 0.1.1`) is never breaking. A change is breaking if it changes
any of these:

1. **Rust public API**: removed or renamed items, changed signatures, new trait
   requirements, new required fields on public structs, changed
   const-generic parameters.
2. **Payload encoding**: any change to the field layout or order of a payload
   that derives `Encode`/`Decode`. This breaks existing Copper logs and
   replay even when the Rust API looks unchanged.
3. **RON configuration**: renamed, removed, or reinterpreted task config keys,
   or changed units or defaults that alter behavior.
4. **Copper (`cu29`) minor version**: moving to a new `cu29` minor is always
   a Meander minor bump.
5. **MSRV**: raising `rust-version`.
6. **Repose dependency**: moving to a new `repose-*` minor when its types
   appear in Meander's public API.

Patch releases may fix bugs, add new items and improve docs or performance,
as long as none of the above change. Breaking releases ship a migration note
under `docs/` when users have to change code.

## Copper compatibility

The README has a compatibility table mapping Meander minors to `cu29` and the
MSRV. Update it with every release that changes either one.

## 1.0

1.0 happens when the core, map, planner and controller contracts are stable;
crates.io publishing starts at 1.0. From 1.0 on, normal SemVer applies.

## Release process

`master` is protected, so the version bump lands through a PR and the tag is
created afterwards on the merged commit.

1. `just wt lucas/chore/release-vX.Y.Z` and work in that worktree.
2. In `CHANGELOG.md`, move the `Unreleased` entries under `## [X.Y.Z] - date`
   and update the compare links; update the README compatibility table if
   needed. Commit.
3. `just release <patch|minor>` (dry run), then
   `just release <patch|minor> --execute`. This runs `cargo release`, which
   bumps the shared version and commits; it neither tags, pushes nor publishes.
4. Run `just pr-check`, push, and open the release PR. Squash-merge it once CI
   is green.
5. From the main checkout, `git pull` and run `just tag`. This tags the
   merged commit `vX.Y.Z` (taken from `Cargo.toml`) and pushes the tag. The
   `Release` workflow then creates the GitHub release.
