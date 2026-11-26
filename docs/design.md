# Quest–LeRobot: initial repository design

Date: 2026-10-16
Status: approved by the user's instruction to implement on 2026-10-18.
Local repository: `C:\development\quest-lerobot`.
Python import package: `lerobot_teleoperator_quest`.

## Purpose and scope

Build an independently installable Quest/WebXR teleoperation integration for
official Hugging Face LeRobot. The user has two SO-ARM arms (this document
originally assumed one SO-101, whose bench bring-up is complete); controlling
both at once was added as a goal on 2026-11-26. B601 hardware support is outside this project.
The existing `roboarms-ar-teleop` checkout remains unchanged and available for
current hardware testing.

Success means the official SO-101 follower owns the USB connection, calibration,
observations and motor commands, while this integration supplies Quest input,
Cartesian targeting and a checked conversion into follower actions. Recording
uses measured observations and separately retained commands.

Creating a local repository does not imply publishing it to GitHub or PyPI.
No hardware connection, calibration write or motor movement is part of the
initial repository bootstrap.

## Approaches considered

1. **External LeRobot teleoperator plus action processor — recommended.**
   Keep LeRobot's official follower and dataset facilities. A Quest input
   plugin emits controller samples; an observation-aware processor handles
   anchoring and IK. This needs an explicit processor composition step but
   gives each component one responsibility.
2. **Keep the existing server and add a LeRobot hardware backend.**
   This preserves more existing UI behavior quickly, but retains a parallel
   application, protocol and recording stack. It is a migration option rather
   than the clean starting point requested here.
3. **Fork LeRobot and add Quest support internally.**
   Direct integration is possible but creates an upstream maintenance burden.
   The external plugin extension mechanism makes a fork unnecessary.

## Ownership and data flow

```text
Quest WebXR client
    -> validated controller samples over WebSocket
    -> Quest Teleoperator.get_action(): controller input only
    -> action processor(controller input, measured robot observation)
       -> clutch/deadman state
       -> frame transform and Cartesian target
       -> IK in an explicitly defined URDF frame
       -> joint and rate limits
       -> follower action dictionary
    -> official SO-101 follower.send_action()

official follower.get_observation()
    -> measured feedback for the processor and telemetry
    -> observation.state in LeRobot recordings

requested action / processed action / returned applied action
    -> distinct command fields, never substituted for observations
```

LeRobot's current teleoperate loop calls `get_action()` without robot feedback,
then passes `(raw_action, obs)` to its teleoperator action processor. A generic
`send_feedback()` call is not supplied for SO-101 by that loop. Therefore IK
cannot silently depend on that callback, a global robot object or a second
serial connection. The first runnable integration must explicitly compose the
processor through supported public APIs; it must not claim the stock CLI's
default identity processors are sufficient.

There is one owner of the serial connection: the official follower. This
project contains no replacement Feetech bus, motor setup or motor calibration
implementation. XR-to-base registration is separate from motor calibration.

## Action and calibration contract

- Use six named follower fields in fixed declared order:
  `shoulder_pan.pos`, `shoulder_lift.pos`, `elbow_flex.pos`, `wrist_flex.pos`,
  `wrist_roll.pos`, `gripper.pos`.
- Configure the initial official follower with `use_degrees=True`. Its five
  arm joint values are degrees and its gripper value is calibrated 0–100.
  Do not auto-detect units or support the alternative -100–100 joint mode in
  the first milestone.
- Internal kinematics use radians, metres and xyzw quaternions. The gripper
  remains a separate calibrated opening value and is not an IK joint.
- Degrees-to-radians conversion alone does not establish agreement between
  calibrated servo coordinates and URDF coordinates. A verified per-joint
  mapping must explicitly encode sign and zero reference. Real Cartesian
  control remains disabled until that mapping has been validated on the arm.
- Reuse an existing official LeRobot calibration by explicit robot ID and
  directory. Do not silently import this repo's old YAML calibration, run
  calibration, write homing offsets or assume old offsets remain valid after
  another tool changes the servos.
- Reject missing, extra, non-finite or out-of-contract action values. A unit
  mismatch is an error rather than a clipping operation.
- Every feedback snapshot carries measured positions separately from requested
  and applied commands, plus monotonic timestamps. Report unavailable applied
  command information as unavailable rather than inventing feedback.

## Motion gate requirements for the runnable integration

The bootstrap contains contract code and tests, not a live-control command.
Before a real-control command is added, its processor must enforce these rules:

1. Start disarmed; calibration and fresh measured feedback are prerequisites.
2. Anchor every new grip from measured FK, not a previous requested target.
3. Latch translation/rotation scale on engagement so detent changes cannot jump
   the target. Limit joint velocity using actual elapsed monotonic time.
4. Enforce configured joint and workspace limits; reject invalid or failed IK.
5. Reject old/out-of-order input; use host receipt time for a 200 ms watchdog,
   not the headset's unrelated clock.
6. Grip release, stale input, disconnect or stop prevents further target
   progression. Establish a bounded hold from fresh measured state, subject to
   the official follower's supported position-hold semantics. Stale feedback
   faults the session; it must never seed a new motion target.
7. Reconnection cannot re-arm an already-held grip. Require a new release and
   press. A stop remains latched until explicitly cleared while disarmed.
8. The command loop must continue ticking when WebSocket input stops. Input
   reception must not block the watchdog.

Process termination, lost USB communication and loss of power are separate
failure cases: a Python input watchdog cannot promise a physical stop after
the host stops running. Document and bench-test shutdown/hold behavior before
declaring hardware support.

## Initial repository contents

The first implementation milestone is a tested foundation, not a copy of the
entire old application:

```text
README.md                         purpose, status, development commands
AGENTS.md                         ownership and verification constraints
pyproject.toml                    Python 3.12 project, test/build configuration
src/lerobot_teleoperator_quest/
  __init__.py                     package exports
  contracts.py                    input, measured state and command types
  so101.py                        explicit follower keys and unit conversions
tests/
  test_contracts.py               validation and separate state/command values
  test_so101.py                   unit conversions and mapping refusal cases
  test_installation.py            import from an installed wheel outside checkout
docs/
  design.md                      approved version of this design
  roadmap.md                     later integration and hardware acceptance gates
.github/workflows/check.yml       unit tests, lint and wheel installation check
.gitignore
```

Do not add a placeholder `Teleoperator` subclass with unimplemented abstract
methods or advertise `lerobot-teleoperate --teleop.type=quest` before discovery
and processor composition pass real integration tests. Add those components
as a subsequent runnable milestone.

Use Python 3.12 for the baseline. Official LeRobot commit
`b07adcc5e3ce42c26a248236dbb3571f84c28c43` reports version 0.7.1 and is the
input-plugin integration baseline. The runtime extra pins that immutable Git
commit; the metadata version label is not a published PyPI release. The real
factory and input contracts are tested; the IK processor and physical mapping
remain the next milestone, not a compatibility claim for live motion.
Keep base contract tests independent of Torch and serial hardware. Put the
real LeRobot compatibility test in a separate explicit integration gate.

No checkout-relative imports, editable sibling dependencies, old monorepo
paths or `sys.path` manipulation. Any later model resources must be included
in the built package and loaded with package-resource APIs. No runtime fetch
of mutable upstream robot models.

## Verification and acceptance

Initial repository acceptance requires:

- `python -m pytest` passes on Python 3.12 without a connected robot.
- Lint passes and a wheel builds and installs into an isolated environment.
- An import outside the checkout succeeds without the old repo on `sys.path`.
- Tests prove a measured pose differs from its requested command through
  serialization; a jammed gripper observation is not replaced by its goal.
- Tests prove the declared names, ordering and degree/radian/gripper conversion
  endpoints. URDF conversion without a verified mapping is refused.
- Non-finite values, invalid shapes, missing keys and invalid gripper ranges
  are rejected.
- Documentation distinguishes implemented features from future milestones.

Later acceptance gates, before claiming real teleoperation:

1. Real pinned LeRobot plugin discovery and observation-aware processor test.
2. Fake-follower end-to-end control, watchdog and disconnect/re-arm tests.
3. Packaged SO-101 kinematics and verified calibration-to-URDF mapping.
4. Minimal HTTPS Quest client with explicit ownership of one connected input.
5. Human-supervised SO-101 joint checks and low-speed Cartesian movement.
6. Official LeRobot recording/reload smoke test with measured observations,
   applied-action semantics, frame timing and camera features checked.

TEACH, saved paths, autonomous playback, B601 support, cloud recording storage,
Pi deployment, and the old replay applications are outside the initial scope.

## Upstream references inspected

- https://huggingface.co/docs/lerobot/integrate_hardware
- https://github.com/huggingface/lerobot/blob/main/pyproject.toml
- https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/teleoperator.py
- https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py
- https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/config_so_follower.py
- https://github.com/huggingface/lerobot/blob/main/src/lerobot/scripts/lerobot_teleoperate.py

These main-branch links are architectural references; implementation must pin
the exact upstream dependency used by its compatibility tests.

## Implementation scope ruling (2026-10-18)

The user's explicit instruction to implement the extension authorizes the
foundation plus the real input plugin. `Quest` is concrete and tested through
official discovery and factory creation; it receives authenticated version-1
poses, exports `quest.*` features and stores measured feedback separately.
It neither opens a robot nor produces follower joint actions. A custom
observation-aware control processor and the browser UI remain subsequent work.
No legacy source files or models are copied into this implementation.
