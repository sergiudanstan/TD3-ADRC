# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.1] - 2026-11-26

### Fixed
- README: removed the "37.28% improvement" claim and the transcribed hardware results table. The
  paper's abstract (the only part verified) states no such figure, and the table contradicted
  the claim. The README now cites the paper's DOI and says the demo results come from an
  untrained agent.

### Removed
- `CLAUDE.md` and `plan.md` (agent and planning files, not referenced by CI, tests or docs).

## [0.1.0] - 2026-11-25

First tagged release. A MATLAB implementation of TD3-ADRC for quadrotor attitude control
after Cai et al. (2026): a 12-state quadrotor model, ADRC with a parameterized tanh
nonlinearity (tracking differentiator, extended state observer, nonlinear feedback), a TD3
agent with actor, twin critics and a replay buffer written in plain MATLAB, comparison,
wind-disturbance and trajectory-tracking demos, a Blender export and animation script, and
MATLAB MCP tool entry points.

### Added
- Unit tests (`tests/run_all_tests.m`, ten tests) run in CI on every push and pull request
  with base MATLAB R2025b; the CI step fails unless all ten pass.
- `ruff` check of the Blender script in CI.
- CI, CodeQL (Python and workflows), OpenSSF Scorecard and a tag-triggered release workflow
  with build provenance; Dependabot for GitHub Actions.
- `SECURITY.md`, `CONTRIBUTING.md`, this changelog and a `VERSION` file.
- README badges and a Development section.

### Fixed
- README listed the Deep Learning Toolbox, Reinforcement Learning Toolbox and Simulink as
  requirements; none of them is called anywhere in `src/`, `params/` or `tests/`.

[Unreleased]: https://github.com/sergiudanstan/TD3-ADRC/compare/v0.1.1...HEAD
[0.1.1]: https://github.com/sergiudanstan/TD3-ADRC/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/sergiudanstan/TD3-ADRC/releases/tag/v0.1.0
