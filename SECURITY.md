# Security Policy

This is a set of MATLAB functions and demos for simulating and training a quadrotor
attitude controller, plus a Blender script that renders exported flight data. It runs on
the user's own machine with that user's permissions and contains no server, network client
or authentication of its own. `src/mcp_tools/td3_adrc_mcp_tools.m` is meant to be called
from a MATLAB MCP server that you run separately, so whoever can call that server can
call these functions.

## Supported versions

| Version | Supported |
|---|---|
| 0.1.x | ✅ Current |

Single maintainer; only the latest release receives fixes.

## Reporting a vulnerability

**Please do not open a public issue for a security problem.** Report it privately:
**[Report a vulnerability](https://github.com/sergiudanstan/TD3-ADRC/security/advisories/new)**
(Security → Advisories → Report a vulnerability).

| Stage | Target |
|---|---|
| Acknowledgement | within 3 business days |
| Triage | within 10 business days |
| Fix, or a documented decision not to fix | within 90 days |

These are best-effort targets from a single unpaid maintainer, not a contractual SLA.
There is no bug bounty; credit is offered in the advisory and changelog.

## Scope

In scope:

- Loading saved agents, networks and replay buffers with MATLAB `load` / `save`
  (`td3_agent`, `actor_network`, `critic_network`, `replay_buffer`, `demo_wind_disturbance`).
  A `.mat` file is data from whoever produced it; do not load one you do not trust. We have
  not audited what a crafted file can do beyond what MATLAB itself allows.
- The file path arguments of the save, load and export functions (`export_to_blender`,
  `export_comparison_to_blender`, `td3_adrc_mcp_tools`), where a path chosen by an
  untrusted caller reaches outside `results/`.
- `blender/scripts/animate_flight.py`, which reads a JSON file and a manifest from
  `results/animations/` (`json.load`, no execution of file content).
- Anything in this repository's workflows that could publish a release not built from `main`.

Out of scope: vulnerabilities in MATLAB, Blender or the MATLAB MCP server you use to expose
these functions (report upstream), and the control performance of the trained policy. The
demo gains are simulation results from an untrained or lightly trained agent; a worse-than-
expected result is a correctness bug, file it as a normal issue. Do not fly hardware with
this code without your own safety review.

## Supply chain

CI runs the MATLAB unit tests (base MATLAB, no toolbox) and `ruff` on the Blender script on
every push, `pip-audit` on the hash-pinned CI toolchain, CodeQL on the Python script and the
workflow files (CodeQL does not analyse MATLAB, so the MATLAB source has no static security
analysis), and `zizmor` on the workflows. Actions are pinned to full commit SHAs and kept
current by Dependabot. Releases attach a source archive with Sigstore build provenance:

```
gh attestation verify TD3-ADRC-v0.1.0.tar.gz --repo sergiudanstan/TD3-ADRC
```
