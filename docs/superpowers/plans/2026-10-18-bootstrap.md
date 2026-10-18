# Quest LeRobot extension implementation plan

> Execution: superpowers:executing-plans, inline. The user explicitly requested implementation on 2026-10-18.

**Goal:** Create an independent, tested SO-101 contract package and real LeRobot Quest input extension.
**Architecture:** Official LeRobot owns hardware. Immutable measured state and commands remain separate. The plugin emits controller input, never unverified motor targets.
**Tech stack:** Python 3.12, setuptools, pytest, websockets 15, official LeRobot 0.6.2.
**Spec:** docs/design.md, approved by the user's instruction to proceed.

## Global constraints

- No edits to the old repository, no motor connection, no calibration writes.
- No copying the old server, assets or monorepo dependencies.
- SO-101: five arm joints in explicit degree/radian contracts; gripper 0–100/0–1.
- Never equate calibrated motor coordinates with URDF coordinates without a verified mapping.
- Keep base contracts usable without LeRobot/Torch.
- Pin and test the real optional LeRobot integration.

## Review focus

1. Measured state must survive when commands differ, including a jammed gripper.
2. Invalid shapes, non-finite numbers, booleans used as numbers and mutation after construction.
3. Stale/out-of-order input and reconnect with grip already held.
4. Authentication, one-client ownership and shutdown of the input listener.
5. Wheel discovery outside the checkout, including the exact distribution-name prefix.

## Task 1: state, actions and calibration mapping

Files: src/lerobot_teleoperator_quest/{contracts,so101}.py; tests/test_contracts.py.
Interfaces: So101State(joints_rad, gripper), Telemetry(measured, requested, applied, received_at); from_follower_action, from_follower_observation, to_follower_action; UrdfMapping.to_urdf/from_urdf.
- [x] Write tests for independent telemetry, immutable values, strict keys, units, mappings and malformed data.
- [x] Run `python -m pytest tests/test_contracts.py -q`; observed missing-package failure.
- [x] Implement the declared types and conversions using only the standard library.
- [x] Run the same command; observed all tests passing.

## Task 2: latest-input state and real LeRobot plugin

Files: src/lerobot_teleoperator_quest/{input,config,quest}.py; tests/test_input.py; tests/test_plugin.py.
Interfaces: InputBuffer.open_session/submit/latest/close_session, QuestConfig, Quest.connect/get_action/disconnect and read-only feedback.
- [x] Write core input tests and real plugin factory/WebSocket tests.
- [x] Run `python -m pytest tests/test_input.py tests/test_plugin.py -q`; observed missing-module failure.
- [x] Implement validated input, host receipt timestamps, sequence/session fencing and release-before-grip gating.
- [x] Implement a real LeRobot subclass and authenticated single-client WebSocket listener, with finite shutdown.
- [x] Run the same command; observed all tests passing against installed LeRobot 0.6.2.

## Task 3: package, verify and deliver independent repository

Files: README.md, AGENTS.md, docs/roadmap.md, .github/workflows/check.yml, scripts/check_installation.py.
- [x] Document actual implemented scope and why the stock identity-processor CLI cannot drive this Cartesian input.
- [x] Run full pytest and ruff; build a wheel using `pip wheel --no-deps --no-build-isolation`.
- [x] Install wheel into an isolated environment outside checkout; verify import without optional dependencies.
- [x] Install into a separate environment with real LeRobot available; verify upstream discovery and factory.
- [x] Obtain a fresh code review, fix material findings and rerun affected checks.
- [x] Copy only this new project into the absent C:/development/quest-lerobot directory; initialize its independent Git history and commit verified files on codex/bootstrap.

## Execution rulings and progress

- User's explicit implementation request authorizes continuous execution; no extra design checkpoint.
- New repository is isolated by construction. No worktree of the old repository is appropriate.
- Build in the writable staging directory, then copy the verified project to the requested sibling with sandbox approval; never overwrite an existing destination.
- Baseline found locally: official LeRobot 0.6.2, checkout commit 6adf51511b7625090eade8d82d9f61a1846ebe56. Integration tests must use its real classes, not mocked equivalents.
- Task 1: complete. Original contract tests failed with missing package, then 28 passed; later overflow regression expanded coverage.
- Task 2: complete. Original plugin/input tests failed with missing modules, then real transport/factory tests passed.
- Review: independent reviewer identified incomplete optional imports, pending-handshake socket shutdown and degree-conversion overflow. Each received a failing regression and a passing fix. Latest full suite: 74 passed.
- Ruling: official version metadata 0.6.2 is not a PyPI release. Pin the immutable, verified official Git commit rather than a non-installable version constraint.
- Ruling: this sandbox cannot use the user's pre-existing pytest/pip cache directories. Tests use project-local `--basetemp .test-tmp`; wheel builds use `--no-cache-dir`. Neither changes product behavior or verification requirements.
- Ruling: wheel probes use `--ignore-installed` inside their temporary venv so an inherited source installation cannot masquerade as the built wheel.

- Task 3: complete. Verified 74 tests, ruff and both installed-wheel probes before copy; tests and lint pass again in the independent destination. No remote configured or hardware accessed.
