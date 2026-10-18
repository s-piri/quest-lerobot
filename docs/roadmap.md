# Roadmap

## Completed foundation

- Immutable SO-101 state/action and telemetry contracts.
- Explicit unit and calibration-bound URDF mapping functions.
- Real upstream Quest teleoperator registration and factory creation.
- Authenticated, single-owner WebSocket controller input with host-side timeout.
- Offline tests, independent wheel install and upstream discovery checks.

## Next: observation-aware control processor

Compose Quest input and current official follower observations through LeRobot's
processor API. Keep official LeRobot as the only serial/calibration owner.
Before enabling physical movement:

1. Package a provenance-recorded SO-101 kinematic model and implement FK/IK.
2. Verify each calibrated follower joint's zero/direction against that model.
   Bind the mapping to the existing official robot ID/calibration; never
   recalibrate or rewrite EEPROM silently.
3. Re-anchor every grip from measured FK. Latch scale on engage and apply
   Cartesian workspace, joint, velocity and gripper limits.
4. Add a host-ticked motion gate, latched stop, stale-feedback faults and
   release/re-press after reconnect. The input timeout alone is not this gate.
5. Test with a fake follower whose measured state lags and whose gripper jams.
6. Bench-verify shutdown, disconnect, hold and low-speed movement with the human
   operator. No automated test may energise the real arm.

## Then: Quest client and datasets

- Minimal HTTPS WebXR page with deliberate pairing and explicit reference frame.
- A runner that composes the verified processor and official SO-101 follower.
- Official LeRobot dataset record/reload smoke test, retaining measured state
  and the action actually applied after all processing/clipping.
- Camera names, frame timing and task metadata tested through the real library.

The old TEACH/playback/replay applications, B601 drivers, cloud storage and Pi
deployment are not part of the bootstrap. Add features only when they support
this SO-101 workflow and after the motion boundary above is verified.
