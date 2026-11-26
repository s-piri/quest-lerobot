"""Latest-only controller input with host time and session ownership."""

from dataclasses import dataclass, replace
from math import sqrt
from threading import Lock
from time import monotonic

from .contracts import finite, timestamp, unit_interval, vector

HANDS = ("left", "right")
HAND_FIELDS = frozenset({"position", "orientation", "grip", "gripper", "scale"})
# Optional beside them: "follow". False previews a target without moving the arm.
OPTIONAL_FIELDS = frozenset({"follow"})
POSE_FIELDS = frozenset({"type", "version", "seq"}) | HAND_FIELDS
FRAME_FIELDS = frozenset({"type", "version", "seq", *HANDS})


def _header(payload, version: int) -> int:
    if payload["type"] != "pose" or type(payload["version"]) is not int:
        raise ValueError("invalid pose type/version")
    if payload["version"] != version:
        raise ValueError("unsupported pose version")
    seq = payload["seq"]
    if type(seq) is not int or not 0 <= seq < 2**53:
        raise ValueError("sequence must be a non-negative JSON-safe integer")
    return seq


@dataclass(frozen=True)
class ControllerSample:
    sequence: int
    position: tuple[float, ...]
    orientation: tuple[float, ...]
    grip: bool
    gripper: float
    scale: float
    received_at: float
    follow: bool = True  # False: the target may move, but the follower stays put

    @classmethod
    def checked(cls, sequence: int, hand, received_at: float):
        """Validate one controller's fields, whichever message carried them."""
        if type(sequence) is not int or not 0 <= sequence < 2**53:
            raise ValueError("sequence must be a non-negative JSON-safe integer")
        if type(hand["grip"]) is not bool:
            raise ValueError("grip must be a boolean")
        follow = hand.get("follow", True)
        if type(follow) is not bool:
            raise ValueError("follow must be a boolean")
        orientation = vector(hand["orientation"], 4, "orientation")
        if abs(sqrt(sum(q * q for q in orientation)) - 1) > 0.001:
            raise ValueError("orientation must be a unit xyzw quaternion")
        return cls(
            sequence,
            vector(hand["position"], 3, "position"),
            orientation,
            hand["grip"],
            unit_interval(hand["gripper"], "gripper"),
            unit_interval(hand["scale"], "scale"),
            timestamp(received_at),
            follow,
        )

    @classmethod
    def parse(cls, payload, received_at: float):
        """A version-1 message: one unnamed controller."""
        if not isinstance(payload, dict) or set(payload) - OPTIONAL_FIELDS != POSE_FIELDS:
            raise ValueError("pose message must contain exactly the documented fields")
        return cls.checked(_header(payload, 1), payload, received_at)


def parse_frame(payload, received_at: float, bimanual: bool) -> dict:
    """Every controller in one message. A hand that is not tracked is None."""
    if not bimanual:
        return {"": ControllerSample.parse(payload, received_at)}
    if not isinstance(payload, dict) or set(payload) != FRAME_FIELDS:
        raise ValueError("pose message must contain exactly the documented fields")
    sequence = _header(payload, 2)
    frame = {}
    for hand in HANDS:
        fields = payload[hand]
        if fields is None:
            frame[hand] = None
        elif not isinstance(fields, dict) or set(fields) - OPTIONAL_FIELDS != HAND_FIELDS:
            raise ValueError(f"{hand} must be null or contain exactly the documented fields")
        else:
            frame[hand] = ControllerSample.checked(sequence, fields, received_at)
    return frame


class InputBuffer:
    """One controller by default; `bimanual` holds a left and a right hand.

    Each hand has its own release-before-press gate. One sequence and one
    receipt time cover the whole message, so both hands go stale together.
    """

    def __init__(self, *, clock=monotonic, timeout: float = 0.2, bimanual: bool = False):
        self.timeout = finite(timeout, "timeout")
        if self.timeout <= 0:
            raise ValueError("timeout must be positive")
        if type(bimanual) is not bool:
            raise ValueError("bimanual must be a boolean")
        self.bimanual = bimanual
        self._hands = HANDS if bimanual else ("",)
        self._clock = clock
        self._lock = Lock()
        self._generation = 0
        self._session: int | None = None
        self._clear()

    def _clear(self):
        self._frame = dict.fromkeys(self._hands)
        self._last: tuple[int, float] | None = None  # newest sequence and its receipt time
        self._released = dict.fromkeys(self._hands, False)

    @property
    def session(self) -> int:
        with self._lock:
            return self._session or 0

    def open_session(self) -> int:
        with self._lock:
            if self._session is not None:
                raise RuntimeError("an input session already owns the stream")
            self._generation += 1
            self._session = self._generation
            self._clear()
            return self._session

    def close_session(self, session: int):
        with self._lock:
            if self._session == session:
                self._session = None
                self._clear()

    def submit(self, session: int, payload):
        with self._lock:
            if self._session is None or session != self._session:
                raise ValueError("input session no longer owns the stream")
            now = self._clock()
            frame = parse_frame(payload, now, self.bimanual)
            sequence = payload["seq"]
            if self._last is not None:
                if sequence <= self._last[0]:
                    raise ValueError("sequence must increase within a session")
                age = now - self._last[1]
                if age < 0:
                    raise ValueError("host clock moved backwards")
                if age > self.timeout:
                    self._released = dict.fromkeys(self._hands, False)
            for hand, sample in frame.items():
                if sample is None:
                    self._released[hand] = False  # a hand that reappears must release first
                    continue
                if not sample.grip:
                    self._released[hand] = True
                frame[hand] = replace(sample, grip=sample.grip and self._released[hand])
            self._frame = frame
            self._last = (sequence, now)

    def frame(self) -> tuple[dict, int]:
        """Atomically pair every hand's sample with the session that owns it."""
        with self._lock:
            session = self._session or 0
            if self._last is None:
                return dict.fromkeys(self._hands), session
            age = timestamp(self._clock()) - self._last[1]
            if not 0 <= age <= self.timeout:
                self._released = dict.fromkeys(self._hands, False)
                return dict.fromkeys(self._hands), session
            return dict(self._frame), session

    def snapshot(self) -> tuple[ControllerSample | None, int]:
        """The single controller's sample and its session."""
        if self.bimanual:
            raise RuntimeError("a bimanual buffer has no single sample; use frame()")
        frame, session = self.frame()
        return frame[""], session

    def latest(self) -> ControllerSample | None:
        return self.snapshot()[0]


def input_action(sample: ControllerSample | None, session: int, prefix: str = "quest.") -> dict:
    """Controller features intentionally cannot be mistaken for follower .pos keys."""
    active = sample is not None
    values = {
        "active": active,
        "session": session,
        "seq": sample.sequence if active else -1,
        "received_at": sample.received_at if active else 0.0,
        "grip": sample.grip if active else False,
        "follow": sample.follow if active else False,
        "gripper": sample.gripper if active else 0.0,
        "scale": sample.scale if active else 0.0,
        **dict(zip(("x", "y", "z"), sample.position if active else (0.0, 0.0, 0.0), strict=True)),
        **dict(
            zip(
                ("qx", "qy", "qz", "qw"),
                sample.orientation if active else (0.0, 0.0, 0.0, 1.0),
                strict=True,
            )
        ),
    }
    return {prefix + key: value for key, value in values.items()}


def frame_action(frame: dict, session: int) -> dict:
    """`quest.*` for one controller, or `quest.left.*` and `quest.right.*` for two."""
    if "" in frame:
        return input_action(frame[""], session)
    action = {}
    for hand in HANDS:
        action |= input_action(frame[hand], session, f"quest.{hand}.")
    return action


def action_sample(action, prefix: str = "quest.") -> tuple[ControllerSample | None, int]:
    """Rebuild one validated controller sample from the features exported above."""
    try:
        values = {key: action[prefix + key] for key in input_action(None, 0, "")}
    except (KeyError, TypeError) as exc:
        raise ValueError(f"action must contain the {prefix}* input features") from exc
    session = values["session"]
    if type(session) is not int or session < 0 or type(values["active"]) is not bool:
        raise ValueError(f"{prefix}session and {prefix}active are malformed")
    if not values["active"]:
        return None, session
    hand = {
        "position": [values[k] for k in ("x", "y", "z")],
        "orientation": [values[k] for k in ("qx", "qy", "qz", "qw")],
        "grip": values["grip"],
        "follow": values["follow"],
        "gripper": values["gripper"],
        "scale": values["scale"],
    }
    return ControllerSample.checked(values["seq"], hand, values["received_at"]), session
