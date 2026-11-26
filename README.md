# Quest for LeRobot

An independent Quest input extension for official Hugging Face LeRobot,
starting with explicit SO-101 contracts. Python 3.12 or newer.

## Implemented

- A concrete `Quest` / `QuestConfig` LeRobot teleoperator registered as `quest`.
- An authenticated WebSocket input listener with a single active controller,
  host receipt timestamps, increasing sequences, a 200 ms input timeout and
  release-before-grip gating on connection and recovery.
- Separate immutable measured state, requested command and applied command.
- Official SO-101 `.pos` names, degree/radian and gripper 0–100/0–1 conversions.
- Explicit URDF zero/sign mapping bound to a calibration-file fingerprint.
- A host-ticked motion gate (`control.py`): starts disarmed, anchors each grip
  from measured state, latches scale, applies joint, speed, gripper and
  workspace limits, and latches stops and stale-feedback faults.
- Wheel installation and actual upstream plugin discovery checks.

This milestone receives and validates input. **It does not drive the arm.**
The motion gate is tested only against a fake follower and a toy stand-in for
kinematics; it has no SO-101 model, no limit values and no connection to a
robot. Real FK/IK, the verified physical mapping, LeRobot processor
composition, the Quest browser client and the hardware runner come next. The stock
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
the integration module; CI has a separate job that installs and requires it.
See [the roadmap](docs/roadmap.md) and [design](docs/design.md).
