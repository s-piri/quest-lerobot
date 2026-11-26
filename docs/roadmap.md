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
- A per-arm setup file and a tool that fits the joint mapping from poses
  copied onto the limp arm; the procedure is in `bench.md`. A fit is not a
  verification.
- Splat steps 1 and 2 as code, tested on synthetic data only: taking a
  recording, calibrating the wrist camera's lens and mount from board shots,
  and exporting images with camera poses and gripper masks for
  nerfstudio-style trainers and COLMAP. See `splat.md`.
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
   recalibrate or rewrite EEPROM silently. The fitting tool and procedure
   exist (`bench.md`); the measuring and the supervised check are bench work.
3. Give the motion gate bench-measured limit values and the verified mapping.
   It has run against the real model only with a stand-in mapping.
4. Done: the processor step records what the follower reports it applied,
   beside measured and requested, when the host passes it `send_action`'s
   return value. Recording all three in a dataset is still to do.
5. Decide where the operator arms, stops and clears from, and build it.
6. Bench-verify shutdown, disconnect, hold and low-speed movement with the human
   operator. No automated test may energise the real arm.

## Then: Quest client and datasets

- Try the draft WebXR page on the headset over HTTPS and fix what that finds:
  frame rate with two arms drawn, where the virtual arm and the panel appear,
  whether the panel text is comfortable to read, whether grabbing the gripper
  feels right, and whether the buzzes help or annoy.
- For two arms: a second official calibration, mapping, limits and base
  rotation, and a bench check that each hand drives the arm it should.
- A runner that composes the verified processor and official SO-101 follower.
- Official LeRobot dataset record/reload smoke test, retaining measured state
  and the action actually applied after all processing/clipping.
- Camera names, frame timing and task metadata tested through the real library.
  LeRobot's dataset module does not import in the global Python on the
  development machine (it has `av` 18 where LeRobot needs 15), so dataset work
  needs a clean environment first.

## Then: a splat of the scene for teleoperation from another room

Two cameras are barely enough to teleoperate an arm you cannot see. The plan
is a 3D Gaussian splat of the scene, built from wrist-camera recordings, shown
in the headset behind the virtual arm.

A splat is a snapshot: it shows the scene as scanned, not objects moved since.
So it is the backdrop only, and what is live is drawn into it: the arm from
its measured joints, and the wrist-camera image at the camera's true pose. A
splat that updates itself is a later upgrade, if a stereo or depth camera is
added.

It lives in this repository, split by weight, under three rules:

- The core stays dependency-free. Building tools go in their own subpackage
  behind an optional install, as LeRobot itself is; installing the teleoperator
  pulls in none of it, and importing the package never imports it.
- COLMAP and the splat trainer are not dependencies. They run as external
  programs in their own environment; the code here only exports frames and
  camera poses for them and brings the result back.
- Splat files are never committed. They are local files the page loads.

Viewing is part of the client page, because the splat shares a scene with the
virtual arm. If building grows into something larger, the subpackage can move
out then.

Steps. The first two exist as code and are documented in `splat.md`, but
have only run on synthetic data:

1. Export a recording to frames and camera poses: joint angles through the
   verified mapping and this package's kinematics, then a fixed camera-to-wrist
   transform. Poses from joints are the starting guess that reconstruction
   refines; they supply metric scale and the arm's base frame.
2. Calibrate the camera's lens and its camera-to-wrist transform from still
   shots of a printed board. How closely the views agree on where the board is
   also checks the joint mapping.
3. Record a dedicated slow scan of a static scene with exposure locked.
   Ordinary teleoperation recordings will not do: things move. The export
   masks the gripper from the arm's meshes; check its picture of the mask.
4. Reconstruct and train on the GPU, then check the result against the arm.
5. Draw the splat in the page, registered to the arm's base, and measure what
   a standalone headset can render smoothly.
6. Place the live wrist-camera image in the scene.

The old TEACH/playback/replay applications, B601 drivers, cloud storage and Pi
deployment are not part of the bootstrap. Add features only when they support
this SO-101 workflow and after the motion boundary above is verified.
