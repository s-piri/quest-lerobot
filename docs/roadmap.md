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

## Next: observation-aware control processor

Compose Quest input and current official follower observations through LeRobot's
processor API. Keep official LeRobot as the only serial/calibration owner.
Before enabling physical movement:

1. Package a provenance-recorded SO-101 kinematic model and implement FK/IK.
2. Verify each calibrated follower joint's zero/direction against that model.
   Bind the mapping to the existing official robot ID/calibration; never
   recalibrate or rewrite EEPROM silently.
3. Give the motion gate the real FK/IK and bench-measured limit values. Its
   anchoring and limit logic exist but have only run against a toy model.
4. Capture the action the follower actually applied beside the requested
   one. The processor step records measured and requested state only.
5. Decide where the operator arms, stops and clears from, and build it.
6. Bench-verify shutdown, disconnect, hold and low-speed movement with the human
   operator. No automated test may energise the real arm.

## Then: Quest client and datasets

- Try the draft WebXR page on the headset over HTTPS and fix what that finds.
- For two arms: a second official calibration, mapping, limits and base
  rotation, and a bench check that each hand drives the arm it should.
- A runner that composes the verified processor and official SO-101 follower.
- Official LeRobot dataset record/reload smoke test, retaining measured state
  and the action actually applied after all processing/clipping.
- Camera names, frame timing and task metadata tested through the real library.

The old TEACH/playback/replay applications, B601 drivers, cloud storage and Pi
deployment are not part of the bootstrap. Add features only when they support
this SO-101 workflow and after the motion boundary above is verified.
