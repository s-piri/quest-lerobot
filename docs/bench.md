# At the bench: measuring one arm's setup

Everything physical in this project waits on one file per arm, the **setup**.
It records how the arm's own readings relate to the kinematic model, and later
its limits, the headset-to-base rotation and the wrist camera. Each setup is
bound to that arm's official LeRobot calibration file, so it stops being
accepted if the arm is recalibrated.

This page covers the first and blocking part: the joint mapping. Do it once
per arm. Two arms need two setups.

## What is being measured

The follower reports each joint in its own calibrated degrees. The model has
its own zero and its own positive direction for each joint. For every joint
the mapping stores:

- a **direction**, `+1` or `-1`: whether the follower's reading rises or falls
  as the model's angle rises;
- a **zero**: the follower's reading when the model's angle is zero.

Getting a direction wrong makes that joint move the opposite way to your hand.
That is why a fitted mapping is not trusted until the arm has been moved
slowly under supervision.

## The six poses

You copy six poses from the model onto the real arm, with the motors limp, and
note the follower's readings at each. Open the client page on any computer to
see each pose; the address after `index.html` selects it. Drag the picture to
look at the arm from another side, or add a view to the address: `&view=0,10`
looks from the arm's right side, `&view=90,10` from the front, `&view=-90,10`
from behind and `&view=0,85` from above.

| Pose | Page address | What changes from the zero pose |
|---|---|---|
| Zero | `?pose=0,0,0,0,0,0.5` | Upper arm upright, forearm level and pointing forward, gripper level, moving jaw on top. |
| Pan | `?pose=0.7854,0,0,0,0,0.5` | The whole arm swings 45° toward **its right**, seen from behind it. |
| Lift | `?pose=0,0.7854,0,0,0,0.5` | The upper arm tips 45° **forward**, lowering the gripper. |
| Elbow | `?pose=0,0,0.7854,0,0,0.5` | The forearm drops 45° **downward** from level. |
| Wrist | `?pose=0,0,0,0.7854,0,0.5` | The gripper tips 45° **downward**. |
| Roll | `?pose=0,0,0,0,0.7854,0.5` | The gripper rolls 45° so the moving jaw, on top at zero, leans toward the arm's **left**. |

"Forward" is the way the gripper points at zero, "behind" is the opposite
side, and left and right are as seen from behind the arm looking forward.
These directions were computed from the model, not observed on an arm.

Matching a pose by eye is good to a few degrees. That is enough to settle the
directions and to start, and the fit reports how well the poses agree. A
square or a phone's level on the upper arm and forearm helps with zero.

## Reading the arm

Use official LeRobot, in an environment where it and its motor library are
installed. **This snippet was written from the pinned source and has not been
run on an arm.** Connecting turns the motors on, holding the arm where it is;
the next line turns them off again, so **support the arm before running it**.

```python
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

robot = SOFollower(SOFollowerRobotConfig(port="COM5", id="left_arm"))
robot.connect(calibrate=False)      # motors on, holding position
robot.bus.disable_torque()          # motors off: the arm goes limp
print("calibration file:", robot.calibration_fpath)

# Repeat for each pose: place the arm, hold it still, then
print({k: v for k, v in robot.get_observation().items() if k.endswith(".pos")})

robot.disconnect()
```

Use the port and `id` you calibrated the arm with. If `connect` offers to
calibrate, stop: the arm has no official calibration yet, and that comes first.

## The samples file

Write the six readings into a JSON file, pairing each with the model pose you
copied (the five numbers from the page address, without the last one):

```json
[
  {"model": [0, 0, 0, 0, 0],
   "follower": {"shoulder_pan.pos": 1.2, "shoulder_lift.pos": -3.4, "elbow_flex.pos": 0.5,
                "wrist_flex.pos": 2.0, "wrist_roll.pos": -0.8, "gripper.pos": 40.0}},
  {"model": [0.7854, 0, 0, 0, 0],
   "follower": {"shoulder_pan.pos": -43.1, "shoulder_lift.pos": -3.0, "elbow_flex.pos": 0.9,
                "wrist_flex.pos": 1.7, "wrist_roll.pos": -1.1, "gripper.pos": 40.0}}
]
```

More poses are welcome. Any pose works as long as each joint is seen across
at least 20°.

## Fitting

```bash
python -m lerobot_teleoperator_quest.bench fit \
    --samples left_samples.json \
    --calibration <the calibration file printed above> \
    --out left_setup.json
```

It prints each joint's direction, its zero, and the worst disagreement between
your readings and the fit. A joint flagged above 5° was probably posed
unevenly: redo its poses instead of accepting it. If a joint's direction cannot
be settled, the command says so and writes nothing.

`python -m lerobot_teleoperator_quest.bench show left_setup.json --calibration <file>`
prints a setup and checks it still matches the arm's calibration file. A
mismatch means the arm was recalibrated, or the file belongs to the other arm.

## What this does not do

- It does not verify the mapping. The fit only says your six poses are
  consistent with each other.
- It does not set limits, the headset-to-base rotation or the camera. Those
  parts of the setup file exist but are measured separately.
- Nothing here moves the arm.
