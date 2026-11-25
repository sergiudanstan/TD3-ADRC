# Contributing

Contributions are welcome. This is a single-maintainer project, so please open an issue
before starting anything substantial.

**Security problems do not belong in issues.** Follow [SECURITY.md](SECURITY.md).

## Branches, pull requests and releases

- `main` is the only long-lived branch and is protected: changes land through a pull
  request with the required checks green, commits on `main` are signed, and force-pushes
  are refused. Pull requests are squash-merged; GitHub deletes the branch on merge.
- Branch names: `<type>/<topic>` (`feat/`, `fix/`, `docs/`, `chore/`, `release/`);
  agents use `agent/<name>/<topic>`.
- A release is a pull request that sets the version in the `VERSION` file and moves
  `[Unreleased]` in [CHANGELOG.md](CHANGELOG.md) to a dated version. After it merges, a
  signed `vX.Y.Z` tag on that commit triggers `release.yml`, which refuses to run if the
  tag, `VERSION` and the changelog disagree. Tags are never moved or deleted.

## Running it locally

```matlab
cd TD3-ADRC
setup_paths;
demo_comparison;                       % controller comparison, writes a PNG to results/figures
addpath('tests'); run_all_tests;       % the ten unit tests, same file CI runs
```

`run_all_tests` prints PASSED/FAILED per test and returns a struct with `passed` and
`failed`; CI fails unless all ten pass. Add a test with any behaviour change. The Blender
script is checked in CI with `ruff check --select E9,F63,F7,F82 blender/`.

MATLAB files in this repository are indented with 8 spaces per level.
