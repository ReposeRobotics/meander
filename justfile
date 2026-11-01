# Default local PR gate: format, lint (std + no_std), and test (std + no_std + doctests).
default: pr-check

# Full local verification gate matching copper-rs workflow.
pr-check: fmt lint test semver-check

# Format all Rust sources in the workspace.
fmt:
    cargo fmt --all

# Check formatting and run clippy with warnings denied on both std and no_std surfaces.
lint:
    cargo fmt --all -- --check
    cargo clippy --workspace --all-targets --features std -- --deny warnings
    cargo clippy --workspace --lib --no-default-features -- --deny warnings
    cargo clippy --workspace --all-targets --features std,cu29/reflect -- --deny warnings

# Run unit tests (std + no_std) with nextest and verify doctests.
test:
    cargo nextest run --workspace --features std
    cargo nextest run --workspace --no-default-features
    cargo test --workspace --doc --features std
    cargo nextest run --workspace --features std,cu29/reflect

# Print the current public API for review (needs cargo-public-api + pinned nightly).
api-check:
    cargo public-api -p meander-core
    cargo public-api -p meander-grid

# Check every library crate for SemVer breaks against the latest release tag (needs cargo-semver-checks).
# Skipped until the first `v*` tag exists; crates absent from the baseline are skipped.
semver-check:
    #!/usr/bin/env bash
    set -euo pipefail
    baseline="$(git describe --tags --abbrev=0 --match 'v*' 2>/dev/null || true)"
    if [ -z "$baseline" ]; then echo "semver-check: no release tag yet, skipping"; exit 0; fi
    release_type="${SEMVER_RELEASE_TYPE:-}"
    args=()
    for pkg in $(cargo metadata --no-deps --format-version 1 | jq -r '.packages[].name'); do
        if git grep -q "^name = \"$pkg\"" "$baseline" -- '*Cargo.toml'; then
            args+=(-p "$pkg")
        else
            echo "semver-check: skipping $pkg (not in $baseline)"
        fi
    done
    if [ ${#args[@]} -eq 0 ]; then exit 0; fi
    cargo semver-checks check-release "${args[@]}" --baseline-rev "$baseline" ${release_type:+--release-type "$release_type"}

# Bump the shared workspace version, commit and tag vX.Y.Z (needs cargo-release).
# Dry run by default; pass --execute to apply. Never pushes or publishes.
release level *args:
    cargo release {{level}} --workspace {{args}}

# Verify strict no_std compatibility across shared crates.
nostd-ci:
    cargo check --workspace --no-default-features
    cargo clippy --workspace --lib --no-default-features -- --deny warnings
    cargo nextest run --workspace --no-default-features
