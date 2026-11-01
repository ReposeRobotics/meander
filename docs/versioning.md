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

1. Update `CHANGELOG.md`: move `Unreleased` entries under the new version
   with the date, and update the README compatibility table if needed.
2. `just pr-check`, which includes `just semver-check` against the latest
   `v*` tag. For a deliberate break, confirm with
   `SEMVER_RELEASE_TYPE=minor just pr-check`.
3. `just release <patch|minor>`: dry run; then `just release <level> --execute`.
   This bumps the shared version, commits and creates the `vX.Y.Z` tag. It does
   not push or publish.
4. Push the commit and tag, and publish the GitHub release using the
   changelog section as the notes.
