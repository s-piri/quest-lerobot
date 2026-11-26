# Quest for LeRobot

An independent Quest input extension for official Hugging Face LeRobot,
starting with explicit SO-101 contracts. Python 3.12 or newer.

## Implemented

- A concrete `Quest` / `QuestConfig` LeRobot teleoperator registered as `quest`.
- An authenticated WebSocket input listener with a single active client,
  host receipt timestamps, increasing sequences, a 200 ms input timeout and
  release-before-grip gating on connection and recovery.
- A two-arm mode (`bimanual=True`): one message carries both hands, each hand
  has its own grip gate, and a stop or fault in either arm stops both.
- Separate immutable measured state, requested command and applied command.
- Official SO-101 `.pos` names, degree/radian and gripper 0–100/0–1 conversions.
- Explicit URDF zero/sign mapping bound to a calibration-file fingerprint.
- A host-ticked motion gate (`control.py`): starts disarmed, anchors each grip
  from measured state, latches scale, applies joint, speed, gripper and
  workspace limits, and latches stops and stale-feedback faults.
- A LeRobot processor step (`processor.py`) that turns Quest input and the
  follower's observation into follower `.pos` goals for one arm or two.
- A draft WebXR page (`client/`) that sends controller poses and draws a
  virtual arm: solid where the hand asks the arm to be, faint where the arm
  is. Its messages and arm maths are tested in Node, and its flat preview was
  checked in a desktop browser. It has not been run on a headset.
- A preview mode: the virtual arm follows the hand while the real arm waits.
- The upstream SO-101 URDF, packaged unmodified with its provenance, and
  forward and inverse kinematics for it in the standard library
  (`kinematics.py`). Forward kinematics is checked against an independent
  URDF library.
- A per-arm setup file (`bench.py`) holding the joint mapping and, optionally,
  limits, the headset-to-base rotation and a wrist camera, bound to the arm's
  calibration file, with a tool that fits the mapping from poses copied onto
  the limp arm. The procedure is in [docs/bench.md](docs/bench.md).
- The first steps toward a scene splat (`splat/`, an optional install):
  taking a recording, calibrating the wrist camera's lens and mount, and
  exporting images with camera poses for reconstruction. See
  [docs/splat.md](docs/splat.md). Tested on synthetic data only.
- Wheel installation and actual upstream plugin discovery checks.

**It still does not drive an arm.** The gate and the processor step have only
run against fake followers. The kinematic model is the upstream CAD export,
not a measurement of your arm: there is no verified mapping from follower
values to its joints, no limit values and no runner that connects to a robot.
Those need bench work and come next. The stock
`lerobot-teleoperate --teleop.type=quest` identity processor cannot convert
these Cartesian input features into follower joint actions.

There are no imports, sibling dependencies, motor drivers or assets from the
old teleop repository. Official LeRobot owns motor communication and calibration.

## Install

Use an isolated Python 3.12 environment. Install only the standard-library
contracts with `python -m pip install .`, or the real input plugin with:

```bash
python -m pip install ".[dev,lerobot]"
```

The extra pins official LeRobot to commit
`b07adcc5e3ce42c26a248236dbb3571f84c28c43` (its version metadata says 0.7.1).
It uses an immutable Git URL because that version label is not a PyPI release.
This requires Git and can install Torch through LeRobot. It neither forks nor
patches the official repository. Core imports do not need Torch or LeRobot.

## Input-only example

```python
from lerobot_teleoperator_quest import Quest, QuestConfig

quest = Quest(QuestConfig(port=8765))
try:
    quest.connect()
    print("Input URL:", quest.url)
    print("Temporary pairing token:", quest.access_token)
    # Poll quest.get_action() in your application loop. No robot is opened.
    input("Press Enter to stop the listener. ")
finally:
    quest.disconnect()
```

The loopback listener uses `ws://127.0.0.1:8765`. To expose input to a Quest
on your LAN, configure `host`, a trusted TLS `cert` and `key`, and the exact
browser origin (including its port) in `origins`. The plugin does not serve a
web page or generate certificates. An HTTPS page needs a trusted `wss://`
endpoint. `quest.url` shows the bind address; replace a wildcard bind address
with the host's LAN name/IP when connecting from another device.

The first WebSocket message is:

```json
{"type":"auth","token":"the temporary pairing token"}
```

After the server replies `{"type":"ready","version":1,"session":...}`, send
strict version-1 pose messages:

```json
{"type":"pose","version":1,"seq":0,"position":[0,0,0],"orientation":[0,0,0,1],"grip":false,"gripper":0,"scale":0.25}
```

Positions are metres in WebXR reference space, orientations are unit xyzw
quaternions, gripper and scale are 0–1, and `seq` increases per connection.
Send a released grip before pressing it. The server returns an `accepted`
acknowledgment for each valid sample. Do not put client timestamps in a sample:
freshness uses the receiving host's monotonic clock. Acknowledgment means input
was accepted, not that a robot moved.

`get_action()` returns `quest.*` input fields. Stale input returns
`quest.active=false` and `quest.grip=false`. It never returns follower `.pos`
commands. Readiness is a software input state, not a physical emergency stop.

### Two arms

`QuestConfig(bimanual=True)` replies `"version":2` and accepts only version-2
messages, which always name both hands. A hand that is not tracked is `null`:

```json
{"type":"pose","version":2,"seq":0,
 "left":{"position":[0,0,0],"orientation":[0,0,0,1],"grip":false,"gripper":0,"scale":0.25},
 "right":null}
```

`get_action()` then returns `quest.left.*` and `quest.right.*`. Each hand must
release before its own press counts, and again after it loses tracking. Both
hands share one sequence and go stale together. A listener in one mode closes
the connection on the other mode's message.

### Preview and the virtual arm

A hand may carry one optional field, `"follow"`. It defaults to `true`. With
`false`, the gripped hand still moves the goal, but the command sent to the
follower stands still; when `follow` turns `true` the command approaches that
goal at the speed limits. Releasing the grip discards a previewed goal, and
the next grip starts again from the measured arm. `get_action()` exports it as
`quest.follow`.

The host tells the headset what to draw with `Quest.send_state(state)`, where
`state` comes from `QuestControlStep.headset_state()`:

```json
{"type":"state","arms":{"":{"state":"engaged","reason":"previewing",
 "base":[0,0,0,1],"measured":[0.1,0.2,0.3,0,0,0.5],"goal":[0.3,0.2,0.3,0,0,0.5]}}}
```

Joints are the model's radians followed by the gripper opening from 0 to 1,
and are `null` until the arm has been armed. `base` is that arm's rotation
from headset space to its base frame. Delivery never blocks the control loop:
the listener keeps only the newest state and sends it after acknowledging the
client's next sample, so a state can be skipped but never arrives out of order.

### Browser client

`client/index.html` is a static page for the headset's browser. It pairs with
the address and token, enters passthrough where available, and sends grip
poses in the `local-floor` reference space: metres, y up, origin on the floor.
Squeezing the grip is the deadman and a pulled trigger closes the gripper.

It draws each arm from the host's state, using simplified copies of the
upstream meshes in `client/assets`. By default you move an arm by grabbing its
virtual gripper: a squeeze counts only if it starts within 12 cm of it, and
hand and gripper then move one to one. Unticking that setting lets you squeeze
anywhere and use a motion scale. "Preview first" sends `follow` only while A or
X is held.

The page always says what is happening and what to do next, in plain words,
both on the page and on a panel that floats in the headset view. That includes
why nothing is drawn (the arm appears only once the host has armed, because the
joint mapping needs the calibration given then), a hand that is out of sight,
a target out of reach, a stop or fault and that it must be cleared on the host,
a lost connection and why it was refused, and a picture that has stopped
updating. In the headset a ball marks where to grab and turns green within
reach and blue while held, the arm turns orange when out of reach and red when
stopped, and the controller buzzes on a grab, on going out of reach and on a
stop. A thumbstick press brings the arm and the panel back in front of you.

`quest.pairing_link("https://your-host/client/")` returns the page's address
with this run's listener and token filled in, so nobody types a token in a
headset. It reveals the token, so treat the link as one. The page remembers the
address and settings between visits, never the token.

While no sample is flowing the page sends `{"type":"ping"}`, which is not
input and keeps nothing alive: it only lets the host deliver its state, so the
picture keeps up before the headset view starts and while a hand is out of
sight. Without a host, `?pose=0.6,-0.5,0.7,0.6,0.3,0.1` shows that pose,
`?state=<JSON>` a whole host state, and `?panel=1` the headset panel.

The plugin does not serve it: host the `client/` folder over HTTPS yourself, list
that exact origin in `origins`, and give the listener a certificate the headset
trusts. A link may prefill the fields after `#`, for example
`#url=wss://host:8765&token=...&mode=both`.

## SO-101 integration contract

Configure the future official follower with `use_degrees=True`. Its five arm
values use degrees; its gripper uses calibrated 0–100. `So101State` expresses
those same calibrated coordinates in radians and 0–1, **not URDF coordinates**.
Conversions must use named `.pos` fields, not positional guesses.

`UrdfMapping` requires explicitly measured signs/zeros and the SHA-256 of the
current official calibration JSON. A matching hash only detects a changed
file; it does not verify the physical mapping or the servos' EEPROM contents.
Do not reuse the old YAML calibration as if it were official LeRobot calibration.
This extension does not calibrate motors or write homing offsets.

`send_feedback(observation)` stores measured positions only. Official SO-101's
standard loop does not automatically call it; the future application must pass
observations explicitly. Requested/applied commands stay unavailable unless
captured independently by their owner. The input plugin cannot provide an
arm watchdog, IK limits, a stop latch or recording semantics by itself.

## Kinematics

```python
from lerobot_teleoperator_quest.kinematics import so101

model = so101()                       # base_link to gripper_frame_link
pose = model.fk(joints)               # URDF radians in, a Pose in the base frame out
joints = model.ik(pose, seed)         # None when the position is out of reach
```

The model is `so101_new_calib.urdf` from TheRobotStudio/SO-ARM100, pinned by
commit and checksum in `models/PROVENANCE.md`. Its joint values are URDF
radians, not follower values: `UrdfMapping` converts between them, and that
mapping must be measured on the arm.

The arm has five joints, so it cannot reach every orientation. `ik` reaches
the position exactly and turns toward the requested orientation only as far as
it can while staying near the seed. Called each tick with the previous answer,
the orientation closes in over a few ticks. A solve takes about a millisecond.

## Motion gate

`Controller.tick(sample, session, feedback, now)` is called at the control rate
whether or not input arrived, and returns a `Step` whose `command` is a request
for the caller to send, or `None` to send nothing. It needs a `UrdfMapping`,
explicit `Limits` (there are no SO-101 defaults), the WebXR-to-base rotation and
an object providing `fk` and `ik`. All timestamps must come from the same host
monotonic clock as the input.

- `arm()` needs fresh measured feedback and the mapped calibration fingerprint,
  and sends nothing. Motion needs a grip release and then a press, again after
  any input gap, reconnection, stop or fault.
- Releasing, stale input or a new session holds at the state measured at that
  moment. `stop()` latches until `clear()`, which returns to disarmed.
- Stale feedback latches a fault and sends nothing, because an unknown arm
  state must not seed a hold or a target.

A stop here is software state. Nothing in this process can act after the host
stops running, the USB link drops or power is lost.

`Group({"left": ..., "right": ...})` puts two controllers under one stop. Each
arm keeps its own mapping, limits, base rotation and grip, so one hand can
hold while the other moves. Arming is both arms or neither, and a stop or
fault in either arm stops the other at its measured state.

## LeRobot processor

```python
from lerobot_teleoperator_quest.processor import QuestControlStep, quest_processor

step = QuestControlStep(controller_or_group)
pipeline = quest_processor(step)          # call as pipeline((teleop_action, observation))
step.arm(calibration_fingerprint)         # or {"left": ..., "right": ...}
```

The pipeline returns the six follower `.pos` goals, or twelve prefixed
`left_` and `right_` as the official bimanual SO follower expects. The host
application calls `step.arm()`, `step.stop()` and `step.clear()`; each arm's
latest decision is in `step.steps` and its measured and requested states in
`step.telemetry`. Pass the follower's reply to `step.applied(robot.send_action(goals))`
and the telemetry also holds what was actually applied, which a follower that
clips its goals can make differ from what was requested. Until then it is
`None`, never a copy of the request.

LeRobot's loops send an action on every tick, so when the gate decides to send
nothing this step repeats the goal it returned last. Before it has returned
any goal it latches the first measured state once. An observation counts as
fresh when it is a new object; a reused or unusable one faults an armed gate.

## Development

```bash
python -m pip install ".[dev]"
python -m pytest -m "not integration"
python -m ruff check .
python -m ruff format --check .
python -m pip wheel . --no-deps --no-build-isolation --wheel-dir dist
python scripts/check_installation.py --wheel dist/lerobot_teleoperator_quest-0.1.0-py3-none-any.whl
```

With `.[lerobot]` installed, run the full suite and the installed-wheel real
integration check as well:

```bash
python -m pytest
python scripts/check_installation.py --integration --wheel dist/lerobot_teleoperator_quest-0.1.0-py3-none-any.whl
```

No test opens a serial port or commands hardware. Core-only environments skip
the integration modules; CI has a separate job that installs and requires them.
The client checks need Node 22 or newer
(`node --test client/protocol.test.mjs client/arm.test.mjs client/guide.test.mjs`), and one check
constructs the official follower classes, which needs `rustypot`.
`scripts/build_client_assets.py` rebuilds the headset's arm model and
`scripts/make_fk_fixtures.py` the reference poses; both are development tools
with their own dependencies, described at the top of each file.
See [the roadmap](docs/roadmap.md) and [design](docs/design.md).
