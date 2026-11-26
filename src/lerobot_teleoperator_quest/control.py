"""Host-ticked motion gate: permission, grip anchoring and limits.

It opens no robot and contains no kinematic model. The caller supplies measured
feedback, a verified mapping and kinematics, and owns sending any command.
Every time value must come from the same host monotonic clock as the input.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from threading import Lock
from typing import Protocol

from .contracts import So101State, Telemetry, finite, timestamp, unit_interval, vector
from .input import ControllerSample
from .pose import Pose, conjugate, multiply, rotate, scale_rotation, unit_quaternion
from .so101 import UrdfMapping

DISARMED = "disarmed"
ARMED = "armed"
ENGAGED = "engaged"
STOPPED = "stopped"
FAULTED = "faulted"


class Kinematics(Protocol):
    """URDF joint radians and the robot base frame. `ik` returns None when it fails."""

    def fk(self, joints: tuple[float, ...]) -> Pose: ...

    def ik(self, target: Pose, seed: tuple[float, ...]) -> tuple[float, ...] | None: ...


def _positive(value: object, name: str) -> float:
    value = finite(value, name)
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


def _ordered(low, high, size: int, name: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    low, high = vector(low, size, f"{name}_min"), vector(high, size, f"{name}_max")
    if any(a >= b for a, b in zip(low, high, strict=True)):
        raise ValueError(f"{name}_min must be below {name}_max")
    return low, high


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _bound(wanted: float, low: float, high: float, start: float) -> float:
    """Clamp into the limits without ever demanding motion from a start outside them.

    A value already beyond a limit may return inward but can go no further out.
    Snapping it to the limit would move the arm with no movement of the hand.
    """
    return _clamp(wanted, min(low, start), max(high, start))


def _approach(current: float, goal: float, step: float) -> float:
    return current + _clamp(goal - current, -step, step)


@dataclass(frozen=True)
class Limits:
    """Operator-supplied bounds. Joints are calibrated follower radians, NOT URDF.

    There are deliberately no SO-101 defaults: the values come from the bench.
    """

    joint_min: tuple[float, ...]
    joint_max: tuple[float, ...]
    joint_speed: tuple[float, ...]  # radians per second
    gripper_min: float
    gripper_max: float
    gripper_speed: float  # fraction of the calibrated range per second
    workspace_min: tuple[float, ...]  # metres in the robot base frame
    workspace_max: tuple[float, ...]
    max_step_s: float  # a late tick may not move further than this much time allows

    def __post_init__(self):
        for name, size in (("joint", 5), ("workspace", 3)):
            low, high = _ordered(
                getattr(self, f"{name}_min"), getattr(self, f"{name}_max"), size, name
            )
            object.__setattr__(self, f"{name}_min", low)
            object.__setattr__(self, f"{name}_max", high)
        speed = vector(self.joint_speed, 5, "joint_speed")
        object.__setattr__(self, "joint_speed", tuple(_positive(s, "joint_speed") for s in speed))
        low = unit_interval(self.gripper_min, "gripper_min")
        high = unit_interval(self.gripper_max, "gripper_max")
        if low >= high:
            raise ValueError("gripper_min must be below gripper_max")
        object.__setattr__(self, "gripper_min", low)
        object.__setattr__(self, "gripper_max", high)
        object.__setattr__(self, "gripper_speed", _positive(self.gripper_speed, "gripper_speed"))
        object.__setattr__(self, "max_step_s", _positive(self.max_step_s, "max_step_s"))


@dataclass(frozen=True)
class Anchor:
    """One grip: the controller pose and the MEASURED tool pose when it engaged.

    Both scales are latched here, so a later change cannot jump the target.
    """

    controller: Pose  # WebXR reference space
    tool: Pose  # robot base frame
    scale: float
    rotation_scale: float

    def target(self, controller: Pose, xr_to_base) -> Pose:
        moved = rotate(
            xr_to_base,
            tuple(
                now - start
                for now, start in zip(controller.position, self.controller.position, strict=True)
            ),
        )
        turned = multiply(controller.orientation, conjugate(self.controller.orientation))
        turned = multiply(multiply(xr_to_base, turned), conjugate(xr_to_base))
        return Pose(
            tuple(t + self.scale * m for t, m in zip(self.tool.position, moved, strict=True)),
            multiply(scale_rotation(turned, self.rotation_scale), self.tool.orientation),
        )


@dataclass(frozen=True)
class Step:
    """One tick's decision. `command` is a request; None means send nothing.

    `goal` is where the gripped hand asks the arm to be. The command approaches
    it at the speed limits, and stands still while the sample says not to follow.
    """

    state: str
    command: So101State | None
    reason: str
    goal: So101State | None = None


class Controller:
    """Starts disarmed. Only an engaged grip progresses a target.

    Readiness here is software state, not a physical emergency stop: nothing in
    this process can act after the host stops running or the USB link is lost.
    """

    def __init__(
        self,
        *,
        kinematics: Kinematics,
        mapping: UrdfMapping,
        limits: Limits,
        xr_to_base,
        feedback_timeout: float,
        input_timeout: float = 0.2,
        rotation_scale: float = 1.0,
    ):
        if not isinstance(mapping, UrdfMapping) or not isinstance(limits, Limits):
            raise ValueError("a verified URDF mapping and explicit limits are required")
        self.kinematics = kinematics
        self.mapping = mapping
        self.limits = limits
        self.xr_to_base = unit_quaternion(xr_to_base, "xr_to_base")
        self.feedback_timeout = _positive(feedback_timeout, "feedback_timeout")
        self.input_timeout = _positive(input_timeout, "input_timeout")
        self.rotation_scale = finite(rotation_scale, "rotation_scale")
        if self.rotation_scale < 0:
            raise ValueError("rotation_scale must not be negative")
        self._lock = Lock()
        self._state = DISARMED
        self._reason = DISARMED
        self._calibration = ""
        self._now: float | None = None
        self._seen = (0, -1)
        self._released: int | None = None
        self._session = 0
        self._anchor: Anchor | None = None
        self._goal: So101State | None = None
        self._command: So101State | None = None
        self._hold: So101State | None = None
        self._hold_pending = False

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    def urdf_joints(self, state: So101State | None) -> tuple[float, ...] | None:
        """A state in the model's joint radians, for display. None until armed."""
        with self._lock:
            if state is None or not self._calibration:
                return None
            return self.mapping.to_urdf(state, self._calibration)

    def arm(self, feedback: Telemetry | None, calibration: str, now: float):
        """Permit a later release-then-press to engage. It sends nothing itself."""
        with self._lock:
            if self._state in (STOPPED, FAULTED):
                raise RuntimeError(f"cannot arm while {self._state}: clear it first")
            if self._state != DISARMED:
                raise RuntimeError(f"cannot arm while {self._state}")
            if not self._fresh(feedback, timestamp(now)):
                raise RuntimeError("arming requires fresh measured feedback")
            self.mapping.to_urdf(feedback.measured, calibration)  # refuses another calibration
            self._calibration = calibration
            self._state = ARMED
            self._reason = ARMED
            self._released = None
            self._hold = None

    def disarm(self):
        with self._lock:
            if self._state != ARMED:
                raise RuntimeError(f"cannot disarm while {self._state}; stop instead")
            self._state = DISARMED
            self._hold = None

    def stop(self, reason: str = "stop requested"):
        """Latch a stop from any state. The next tick holds at fresh measured state."""
        with self._lock:
            self._latch(STOPPED, reason)

    def clear(self):
        """Return a latched stop or fault to disarmed. It never re-arms."""
        with self._lock:
            if self._state not in (STOPPED, FAULTED):
                raise RuntimeError("there is no latched stop or fault to clear")
            self._state = DISARMED
            self._reason = DISARMED
            self._hold = None
            self._hold_pending = False

    def tick(
        self,
        sample: ControllerSample | None,
        session: int,
        feedback: Telemetry | None,
        now: float,
    ) -> Step:
        """Call at the control rate whether or not input arrived."""
        with self._lock:
            now = timestamp(now)
            if self._now is not None and now < self._now:
                raise ValueError("host clock moved backwards")
            elapsed = 0.0 if self._now is None else min(now - self._now, self.limits.max_step_s)
            self._now = now
            if self._state == DISARMED:
                return Step(DISARMED, None, DISARMED)
            fresh = self._fresh(feedback, now)
            if self._state in (STOPPED, FAULTED):
                if self._hold_pending:
                    self._hold_pending = False
                    self._hold = feedback.measured if fresh else None
                return Step(self._state, self._hold, self._reason)
            if not fresh:
                # Unknown arm state must never seed a hold or a target.
                self._latch(FAULTED, "measured feedback is stale")
                self._hold_pending = False
                self._hold = None
                return Step(FAULTED, None, self._reason)

            usable = self._usable(sample, session, now)
            if self._state == ENGAGED and not (usable and sample.grip and session == self._session):
                self._state = ARMED
                self._anchor = None
                self._goal = None
                self._command = None
                self._released = None
                self._hold = feedback.measured
            if self._state == ARMED:
                if not usable:
                    self._released = None
                    reason = "no fresh input"
                elif not sample.grip:
                    self._released = session
                    reason = "ready"
                elif self._released != session:
                    reason = "release the grip before pressing it"
                else:
                    reason = self._engage(sample, session, feedback.measured)
                    elapsed = 0.0  # a grip's first command is exactly the measured state
                if self._state != ENGAGED:
                    return Step(self._state, self._hold, reason)
            return self._track(sample, elapsed)

    def _fresh(self, feedback: Telemetry | None, now: float) -> bool:
        return (
            isinstance(feedback, Telemetry)
            and 0 <= now - feedback.received_at <= self.feedback_timeout
        )

    def _usable(self, sample: ControllerSample | None, session: int, now: float) -> bool:
        if not isinstance(sample, ControllerSample) or type(session) is not int or session <= 0:
            return False
        if not 0 <= now - sample.received_at <= self.input_timeout:
            return False
        if (session, sample.sequence) < self._seen:
            return False  # an older session or an out-of-order sample
        self._seen = (session, sample.sequence)
        return True

    def _latch(self, state: str, reason: str):
        if self._state in (STOPPED, FAULTED):
            return  # the first cause stays latched
        # Only a moving arm needs a new hold; an idle hold stays as already commanded.
        self._hold_pending = self._state == ENGAGED
        if self._state != ARMED:
            self._hold = None
        self._state = state
        self._reason = reason
        self._anchor = None
        self._goal = None
        self._command = None
        self._released = None

    def _engage(self, sample: ControllerSample, session: int, measured: So101State) -> str:
        """Anchor from measured FK, never from a previous request."""
        tool = self.kinematics.fk(self.mapping.to_urdf(measured, self._calibration))
        if not isinstance(tool, Pose):
            self._latch(FAULTED, "forward kinematics did not return a pose")
            self._hold_pending = False
            return self._reason
        self._anchor = Anchor(
            Pose(sample.position, sample.orientation), tool, sample.scale, self.rotation_scale
        )
        self._goal = measured
        self._command = measured
        self._session = session
        self._state = ENGAGED
        return ENGAGED

    def _track(self, sample: ControllerSample, elapsed: float) -> Step:
        limits = self.limits
        target = self._anchor.target(Pose(sample.position, sample.orientation), self.xr_to_base)
        target = Pose(
            tuple(
                _bound(p, low, high, start)
                for p, low, high, start in zip(
                    target.position,
                    limits.workspace_min,
                    limits.workspace_max,
                    self._anchor.tool.position,
                    strict=True,
                )
            ),
            target.orientation,
        )
        # The goal is where the hand asks the arm to be; the command is what is
        # sent, and it approaches the goal only while the sample says to follow.
        seed = self.mapping.to_urdf(self._goal, self._calibration)
        try:
            solution = self.kinematics.ik(target, seed)
            wanted = (
                None
                if solution is None
                else self.mapping.from_urdf(
                    solution, self._goal.gripper, self._calibration
                ).joints_rad
            )
        except ValueError:
            wanted = None
        if wanted is not None:
            # Bounded against where the arm is being held, not the previous goal.
            joints = tuple(
                _bound(value, low, high, current)
                for current, value, low, high in zip(
                    self._command.joints_rad,
                    wanted,
                    limits.joint_min,
                    limits.joint_max,
                    strict=True,
                )
            )
            gripper = _bound(
                sample.gripper, limits.gripper_min, limits.gripper_max, self._command.gripper
            )
            self._goal = So101State(joints, _clamp(gripper, 0.0, 1.0))
        if sample.follow:
            joints = tuple(
                _approach(current, goal, speed * elapsed)
                for current, goal, speed in zip(
                    self._command.joints_rad,
                    self._goal.joints_rad,
                    limits.joint_speed,
                    strict=True,
                )
            )
            gripper = _approach(
                self._command.gripper, self._goal.gripper, limits.gripper_speed * elapsed
            )
            self._command = So101State(joints, _clamp(gripper, 0.0, 1.0))
        if wanted is None:
            reason = "inverse kinematics rejected the target"
        else:
            reason = "tracking" if sample.follow else "previewing"
        return Step(ENGAGED, self._command, reason, self._goal)


class Group:
    """Several arms under one stop: a stop or fault in any arm stops them all.

    Each arm keeps its own controller, mapping, limits and grip, so one hand can
    hold while the other moves. Arming is all arms or none.
    """

    def __init__(self, controllers: Mapping[str, Controller]):
        if not isinstance(controllers, Mapping) or not controllers:
            raise ValueError("a group needs at least one controller")
        if any(not isinstance(c, Controller) for c in controllers.values()):
            raise ValueError("a group contains only controllers")
        if len({id(c) for c in controllers.values()}) != len(controllers):
            raise ValueError("each arm needs its own controller")
        self.controllers = dict(controllers)
        self._lock = Lock()

    @property
    def states(self) -> dict[str, str]:
        return {name: controller.state for name, controller in self.controllers.items()}

    def arm(self, feedback: Mapping, calibration: Mapping, now: float):
        with self._lock:
            armed = []
            try:
                for name, controller in self.controllers.items():
                    controller.arm(feedback.get(name), calibration.get(name), now)
                    armed.append(controller)
            except (RuntimeError, ValueError):
                for controller in armed:
                    controller.disarm()
                raise

    def disarm(self):
        with self._lock:
            states = self.states
            if any(state != ARMED for state in states.values()):
                raise RuntimeError(f"cannot disarm while {states}; stop instead")
            for controller in self.controllers.values():
                controller.disarm()

    def stop(self, reason: str = "stop requested"):
        with self._lock:
            for controller in self.controllers.values():
                controller.stop(reason)

    def clear(self):
        with self._lock:
            latched = [c for c in self.controllers.values() if c.state in (STOPPED, FAULTED)]
            if not latched:
                raise RuntimeError("there is no latched stop or fault to clear")
            for controller in latched:
                controller.clear()

    def tick(self, inputs: Mapping, feedback: Mapping, now: float) -> dict[str, Step]:
        """`inputs` maps each arm to its `(sample, session)`; `feedback` to its telemetry."""
        with self._lock:
            steps = {
                name: controller.tick(*inputs.get(name, (None, 0)), feedback.get(name), now)
                for name, controller in self.controllers.items()
            }
            latched = [name for name, step in steps.items() if step.state in (STOPPED, FAULTED)]
            if latched and len(latched) < len(steps):
                cause = steps[latched[0]]
                reason = f"{latched[0] or 'arm'} {cause.state}: {cause.reason}"
                for name, controller in self.controllers.items():
                    if name not in latched:
                        controller.stop(reason)
                        steps[name] = controller.tick(None, 0, feedback.get(name), now)
            return steps
