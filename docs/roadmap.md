# Roadmap

## Completed foundation

- Immutable SO-101 state/action and telemetry contracts.
- Explicit unit and calibration-bound URDF mapping functions.
- Real upstream Quest teleoperator registration and factory creation.
- Authenticated, single-owner WebSocket controller input with host-side timeout.
- Offline tests, independent wheel install and upstream discovery checks.
- Motion gate core: disarmed start, measured-state grip anchoring, latched
  scale, joint/speed/gripper/workspace limits, input and feedback watchdogs,
  latched stop, and release/re-press after any gap or reconnection. Tested with
  a fake follower that lags and jams, and a toy stand-in for kinematics.
- Two-arm mode: version-2 messages with both hands, per-hand grip gates, and a
  group that stops both arms when either stops or faults.
- LeRobot processor step composing the gate with Quest input and follower
  observations for one arm or two, run through the real pipeline class.
- Draft WebXR page; its messages are tested in Node against the real listener.
- Virtual arm in the headset page, drawn from the host's state, with grab-to-move
  and a preview mode in which the real arm waits. The page explains every
  state in plain words on the page and on a panel in the headset view.
  Checked in a desktop browser only.
- The upstream SO-101 URDF packaged with provenance, with standard-library
  forward and inverse kinematics. Forward kinematics matches an independent
  URDF library; the gate has driven the model to Cartesian targets in tests.

## Next: observation-aware control processor

Compose Quest input and current official follower observations through LeRobot's
processor API. Keep official LeRobot as the only serial/calibration owner.
Before enabling physical movement:

1. Done: the kinematic model and FK/IK are packaged (see above).
2. Verify each calibrated follower joint's zero/direction against that model.
   Bind the mapping to the existing official robot ID/calibration; never
   recalibrate or rewrite EEPROM silently.
3. Give the motion gate bench-measured limit values and the verified mapping.
   It has run against the real model only with a stand-in mapping.
4. Capture the action the follower actually applied beside the requested
   one. The processor step records measured and requested state only.
5. Decide where the operator arms, stops and clears from, and build it.
6. Bench-verify shutdown, disconnect, hold and low-speed movement with the human
   operator. No automated test may energise the real arm.

## Then: Quest client and datasets

- Try the draft WebXR page on the headset over HTTPS and fix what that finds:
  frame rate with two arms drawn, where the virtual arm and the panel appear,
  whether the panel text is comfortable to read, whether grabbing the gripper
  feels right, and whether the buzzes help or annoy.
- A separate plugin that builds a 3D Gaussian splat of the scene from
  wrist-camera recordings, with camera poses from this package's kinematics,
  to stand behind the virtual arm for teleoperation from another room. The
  splat is a snapshot; live elements are drawn into it.
- For two arms: a second official calibration, mapping, limits and base
  rotation, and a bench check that each hand drives the arm it should.
- A runner that composes the verified processor and official SO-101 follower.
- Official LeRobot dataset record/reload smoke test, retaining measured state
  and the action actually applied after all processing/clipping.
- Camera names, frame timing and task metadata tested through the real library.

The old TEACH/playback/replay applications, B601 drivers, cloud storage and Pi
deployment are not part of the bootstrap. Add features only when they support
this SO-101 workflow and after the motion boundary above is verified.
