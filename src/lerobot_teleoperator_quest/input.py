"""Latest-only controller input with host time and session ownership."""

from dataclasses import dataclass, replace
from math import sqrt
from threading import Lock
from time import monotonic

from .contracts import finite, timestamp, unit_interval, vector

POSE_FIELDS = frozenset(
    {
        "type",
        "version",
        "seq",
        "position",
        "orientation",
        "grip",
        "gripper",
        "scale",
    }
)


@dataclass(frozen=True)
class ControllerSample:
    sequence: int
    position: tuple[float, ...]
    orientation: tuple[float, ...]
    grip: bool
    gripper: float
    scale: float
    received_at: float

    @classmethod
    def parse(cls, payload, received_at: float):
        if not isinstance(payload, dict) or set(payload) != POSE_FIELDS:
            raise ValueError("pose message must contain exactly the documented fields")
        if payload["type"] != "pose" or type(payload["version"]) is not int:
            raise ValueError("invalid pose type/version")
        if payload["version"] != 1:
            raise ValueError("unsupported pose version")
        seq = payload["seq"]
        if type(seq) is not int or not 0 <= seq < 2**53:
            raise ValueError("sequence must be a non-negative JSON-safe integer")
        if type(payload["grip"]) is not bool:
            raise ValueError("grip must be a boolean")
        orientation = vector(payload["orientation"], 4, "orientation")
        if abs(sqrt(sum(q * q for q in orientation)) - 1) > 0.001:
            raise ValueError("orientation must be a unit xyzw quaternion")
        return cls(
            seq,
            vector(payload["position"], 3, "position"),
            orientation,
            payload["grip"],
            unit_interval(payload["gripper"], "gripper"),
            unit_interval(payload["scale"], "scale"),
            timestamp(received_at),
        )


class InputBuffer:
    def __init__(self, *, clock=monotonic, timeout: float = 0.2):
        self.timeout = finite(timeout, "timeout")
        if self.timeout <= 0:
            raise ValueError("timeout must be positive")
        self._clock = clock
        self._lock = Lock()
        self._generation = 0
        self._session: int | None = None
        self._sample: ControllerSample | None = None
        self._released = False

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
            self._sample = None
            self._released = False
            return self._session

    def close_session(self, session: int):
        with self._lock:
            if self._session == session:
                self._session = None
                self._sample = None
                self._released = False

    def submit(self, session: int, payload):
        with self._lock:
            if self._session is None or session != self._session:
                raise ValueError("input session no longer owns the stream")
            sample = ControllerSample.parse(payload, self._clock())
            if self._sample is not None:
                if sample.sequence <= self._sample.sequence:
                    raise ValueError("sequence must increase within a session")
                age = sample.received_at - self._sample.received_at
                if age < 0:
                    raise ValueError("host clock moved backwards")
                if age > self.timeout:
                    self._released = False
            if not sample.grip:
                self._released = True
            self._sample = replace(sample, grip=sample.grip and self._released)

    def latest(self) -> ControllerSample | None:
        return self.snapshot()[0]

    def snapshot(self) -> tuple[ControllerSample | None, int]:
        """Atomically pair the sample with the session that owns it."""
        with self._lock:
            if self._sample is None:
                return None, self._session or 0
            age = timestamp(self._clock()) - self._sample.received_at
            if not 0 <= age <= self.timeout:
                self._released = False
                return None, self._session or 0
            return self._sample, self._session or 0


def input_action(sample: ControllerSample | None, session: int) -> dict:
    """Controller features intentionally cannot be mistaken for follower .pos keys."""
    active = sample is not None
    return {
        "quest.active": active,
        "quest.session": session,
        "quest.seq": sample.sequence if active else -1,
        "quest.received_at": sample.received_at if active else 0.0,
        "quest.grip": sample.grip if active else False,
        "quest.gripper": sample.gripper if active else 0.0,
        "quest.scale": sample.scale if active else 0.0,
        **dict(
            zip(
                ("quest.x", "quest.y", "quest.z"),
                sample.position if active else (0.0, 0.0, 0.0),
                strict=True,
            )
        ),
        **dict(
            zip(
                ("quest.qx", "quest.qy", "quest.qz", "quest.qw"),
                sample.orientation if active else (0.0, 0.0, 0.0, 1.0),
                strict=True,
            )
        ),
    }
