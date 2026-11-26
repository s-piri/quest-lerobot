# Working in Quest for LeRobot

This is an independent Python 3.12 project. The user has two SO-ARM arms, and
controlling both at once is a goal; there is no B601 requirement. The old teleop checkout is reference material,
not a runtime or build dependency.

## Verification

- Run `python -m pytest`, `python -m ruff check .`, and `python -m ruff format --check .`.
- Core-only checks may skip optional integration tests; the integration gate
  must install the pinned official extra and run the real factory/socket tests.
- Build and install a wheel outside the checkout with
  `python scripts/check_installation.py --wheel <wheel>` and repeat with
  `--integration` where official LeRobot is available.
- Never lower test requirements or mock the upstream factory to claim support.
- No automated check may connect to or move the real robot.

## Deliberate decisions

**`quest.*` input keys are not follower `.pos` actions — do not rename them to match.**
The teleoperator receives Cartesian controller poses. LeRobot's default action
processor is an identity transform; renaming the fields would disguise missing
IK and calibration mapping as a usable hardware command. A processor with fresh
measured observations must explicitly produce follower actions.

**`lerobot_teleoperator_quest` distribution name uses underscores — preserve it.**
The pinned upstream discovery code compares raw distribution metadata against
`lerobot_teleoperator_` and imports that same name. Hyphenating the distribution
name makes an installed plugin invisible to that implementation. The installed
wheel check calls the real discovery function in a fresh process to guard it.

**LeRobot is pinned to an official Git commit — do not replace it with `>=0.4`.**
The tested upstream commit reports version 0.7.1, but that version label is not
a published PyPI release. A loose version can select different APIs and Python
requirements. Update the immutable pin only with real integration verification.

**Core contracts are independent of LeRobot; plugin imports are conditional.**
The base wheel imports and handles states without Torch or device SDKs. If
LeRobot and websockets are present, importing the package registers its real Quest config for
official discovery. Integration errors are not swallowed as silent skips.

**Follower radians do not mean URDF radians — do not use an identity mapping.**
Radians convert units, not calibration zero/direction conventions. An explicit
operator-verified mapping is bound to an official calibration fingerprint.
A hash detects file changes only; it cannot inspect physical calibration or
servo EEPROM. There is no motion runner until that mapping is bench-verified.

**Measured, requested and applied values stay separate.**
Arm lag and gripper stalls are useful measured state. Replacing measurements
with targets silently corrupts demonstrations. Missing applied values are None,
not copies of requested values.

**Kinematics is our own standard-library code — do not swap in upstream's solver.**
LeRobot's `RobotKinematics` needs `placo`, for which no Windows build was found
on 2026-11-26, and the user develops on Windows. The packaged URDF is the
unmodified upstream file, checked by checksum at load; update the file, the
commit and the checksum together and regenerate the reference poses.

**Inverse kinematics solves position first and stays near its seed.**
The arm has five joints, so most orientations are unreachable. Weighting
orientation against position bends the tool off target, and chasing an
unreachable turn without a pull toward the seed swings the joints a long way
for no gain. The final position-only pass is what makes position win near
singular poses.

**The goal and the command are separate — do not merge them back.**
The goal is where the gripped hand asks the arm to be and is what the headset
draws; the command is what is sent and approaches the goal at the speed
limits, only while the sample says to follow. Limits bound the goal against
the command, where the arm is being held, not against the previous goal.

**Headset state rides on acknowledgments, newest only — do not send it from the loop.**
A direct send from the control loop would block on a slow headset. The
listener thread delivers the newest queued state after acknowledging a sample,
so states can be skipped and tests must not count them.

**Every state the operator can be in has a sentence — keep it that way.**
Inside the headset the page's own text is invisible, so guidance is drawn on a
panel there too. Wording lives in `client/guide.js` and is tested: a new host
state or reason needs a sentence saying what is happening and what to do, not
the host's raw wording. A ping is not input and must never refresh the input
watchdog; it exists only so the picture keeps up while no sample is flowing.

**A hold latches one measurement; limits never pull the arm inward on their own.**
Re-seeding the hold from each new measurement lets gravity sag walk the goal
downward. Clamping a state that starts outside a limit moves the arm with no
movement of the hand, so limits only refuse motion further out. The first
command of a grip is exactly the measured state, and arming sends nothing.

**The processor step repeats its last goal when the gate sends nothing.**
LeRobot's loops call `send_action` every tick. Returning the measured state
each time would re-seed the hold and let sag walk it downward; repeating the
same goal changes nothing for a position servo. Only the first goal ever
returned is latched from a measurement.

**Bimanual input is one message with one sequence — do not split it per hand.**
Two streams would let one hand's samples go stale or reorder against the
other's. Each hand still has its own release-before-press gate, and a hand
that loses tracking must release again.

**Invalidate input before closing the socket — do not move it after `close()`.**
A closing WebSocket handshake can wait on the peer. Keeping the last armed
sample alive during that wait exposes it after input has already been rejected.
Session IDs also fence stale handler cleanup from a newly connected controller.

**Track accepted raw sockets, not just WebSocket handlers.**
The websockets 15 sync handler runs only after TLS/HTTP handshakes complete.
Wrapping its documented `Server.handler` before starting acceptance lets
disconnect close opening handshakes too and join the associated workers.
Tracking only authenticated peers leaks sockets during shutdown and reconnect.

**Pairing tokens are runtime values, not dataclass config.**
Upstream CLIs log `asdict(config)`. A token field would leak credentials into
those logs even with `repr=False`. Generate a new token per listener start and
only reveal it through an explicit local pairing UI or caller request.
