# The scene splat: from recording to camera poses

This covers the first two steps of the splat plan in `roadmap.md`: calibrating
the wrist camera, and turning a recording into images and camera poses for
reconstruction. Everything here was tested against synthetic data only. **None
of it has been run with a real arm or camera.** Reconstruction, training and
drawing the splat are not built yet.

Install the optional tools into the environment you use for the arm:

```bash
python -m pip install ".[splat]"
```

That adds numpy and OpenCV. COLMAP and a splat trainer are separate programs
and are not installed by this.

## What you need first

- The arm's setup file with its joint mapping, from `bench.md`. Camera poses
  come from joint readings through that mapping, so a wrong mapping gives wrong
  poses.
- A LeRobot environment in which the follower and its wrist camera work.

## A recording

Every tool here reads the same simple folder:

```
my_recording/
  shots.jsonl
  images/000000.png
  images/000001.png
```

Each line of `shots.jsonl` is one picture and the joint readings taken with it:

```json
{"time": 12.5, "image": "images/000000.png", "follower": {"shoulder_pan.pos": 1.2, "...": 0}}
```

`time` is seconds on any steady clock, and must increase.

`splat.capture` writes such a folder from a connected robot. It only reads
observations and never sends a command. **This snippet was written from the
pinned LeRobot source and has not been run**, and the camera settings are
examples to adjust:

```python
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
from lerobot_teleoperator_quest.splat.capture import record_scan, record_stills

cameras = {"wrist": OpenCVCameraConfig(index_or_path=0, width=640, height=480, fps=30)}
robot = SOFollower(SOFollowerRobotConfig(port="COM5", id="left_arm", cameras=cameras))
robot.connect(calibrate=False)      # motors on, holding position
robot.bus.disable_torque()          # motors off: support the arm first
try:
    record_stills(robot, "board_shots", "wrist", shots=20)        # for calibration
    # record_scan(robot, "scene_scan", "wrist", seconds=90)       # for a scene
finally:
    robot.disconnect()
```

With the motors off you move the arm by hand. That is the simplest way to
take both kinds of recording before teleoperation works.

## Calibrating the camera

Print a chessboard with an odd number of inner corners one way and an even
number the other, such as 9 by 6 inner corners (10 by 7 squares). Measure one
square with a ruler. Fix the board flat where the arm can see it, and **do not
move it** until you are done.

Take about 20 still shots with `record_stills`, with the whole board in view
each time, from clearly different directions and distances: tilt the wrist,
roll it, and swing the arm. Hold still for each shot. Then:

```bash
python -m lerobot_teleoperator_quest.splat calibrate board_shots \
    --setup left_setup.json --calibration <the arm's calibration file> \
    --square 0.024 --out left_setup_with_camera.json
```

It writes a new setup file with the camera added and reports two numbers:

- **Lens error**, in pixels. Under one pixel is good. Higher usually means
  blurred shots.
- **How far apart the views place the board**, in millimetres. The board did
  not move, so every view should put it in the same place. A large figure
  means the joint mapping is off, the board moved, or the arm was not still.
  This is also the best available check of the joint mapping itself.

## Scanning a scene

A splat assumes nothing moves. Clear the scene of anything that will, lock the
camera's exposure if you can, and keep the gripper at one opening. Then run
`record_scan` and move the arm slowly so the camera sees the scene from as many
directions as the arm allows. Slow matters twice: fast frames blur, and they
are where a timing error between picture and joints costs most.

## Exporting

```bash
python -m lerobot_teleoperator_quest.splat export scene_scan \
    --setup left_setup_with_camera.json --calibration <the arm's calibration file> \
    --meshes client/assets --out scene
```

It keeps a frame only when the camera has moved 1 cm or turned 3° since the
last kept one, drops frames taken while the camera moved faster than 15 cm a
second, and says how many it left out and why. `--camera-delay` shifts the
joint readings if you find the pictures lag them.

The scene folder holds the kept images, `transforms.json` for nerfstudio-style
trainers, a `colmap/` text model with known poses, and `report.json`. Poses
are in the arm's base frame, in metres. COLMAP's own library was used once to
confirm it reads the model and projects points where intended.

### Hiding the gripper

The wrist camera always sees the gripper it sits on. Left in, the gripper
would be reconstructed as an object floating beside every viewpoint. With
`--meshes`, each view gets a mask in `masks/`: white where the picture is
scene, black over the gripper. The masks are named as COLMAP expects (the
image's name with `.png` added) and listed in `transforms.json`.

A mask is drawn from the arm's meshes through the calibrated camera, then
grown by a margin (`--mask-margin`, 12 pixels by default), because printed
parts, the camera mount and the meshes never agree exactly. The moving jaw's
angle is a straight-line guess from the gripper reading. So **open
`masks_check.png`** in the scene folder: it shows the first view with the
masked area in red, which must cover the whole gripper and little else. If it
is off, the camera's mount calibration is the first thing to redo. If the jaw
is the only part missed, keep the gripper closed during the scan and raise the
margin.

## What comes after, and what to expect

The poses are a starting guess. Servo backlash and link flex put them off by
millimetres, which blurs a splat, so reconstruction should refine them: either
COLMAP triangulating and adjusting from the known poses, or the trainer's own
pose optimisation. Tell the trainer **not** to re-orient, re-centre or re-scale
the poses, or the splat will no longer sit in the arm's base frame.

Still to build: running the reconstruction, checking the result against the
arm, and drawing it in the headset.
