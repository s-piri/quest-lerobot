"""Owned, immutable values: measurements are never synthesized from commands."""

from dataclasses import dataclass
from math import isfinite
from numbers import Real


def finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def vector(value, size: int, name: str) -> tuple[float, ...]:
    try:
        items = tuple(value)
    except TypeError as exc:
        raise ValueError(f"{name} must contain {size} numbers") from exc
    if len(items) != size:
        raise ValueError(f"{name} must contain {size} numbers")
    return tuple(finite(v, name) for v in items)


def unit_interval(value: object, name: str) -> float:
    value = finite(value, name)
    if not 0 <= value <= 1:
        raise ValueError(f"{name} must be between 0 and 1")
    return value


def timestamp(value: object) -> float:
    value = finite(value, "host receipt timestamp")
    if value < 0:
        raise ValueError("host receipt timestamp must be non-negative")
    return value


@dataclass(frozen=True)
class So101State:
    """Calibrated follower coordinates expressed in radians, NOT URDF coordinates."""

    joints_rad: tuple[float, ...]
    gripper: float

    def __post_init__(self):
        object.__setattr__(self, "joints_rad", vector(self.joints_rad, 5, "joints_rad"))
        object.__setattr__(self, "gripper", unit_interval(self.gripper, "gripper"))

    def to_dict(self) -> dict:
        return {"joints_rad": list(self.joints_rad), "gripper": self.gripper}


@dataclass(frozen=True)
class Telemetry:
    measured: So101State
    requested: So101State | None
    applied: So101State | None
    received_at: float

    def __post_init__(self):
        if not isinstance(self.measured, So101State):
            raise ValueError("measured state is required")
        for name in ("requested", "applied"):
            item = getattr(self, name)
            if item is not None and not isinstance(item, So101State):
                raise ValueError(f"{name} must be a state or None")
        object.__setattr__(self, "received_at", timestamp(self.received_at))

    def to_dict(self) -> dict:
        return {
            "measured": self.measured.to_dict(),
            "requested": None if self.requested is None else self.requested.to_dict(),
            "applied": None if self.applied is None else self.applied.to_dict(),
            "received_at": self.received_at,
        }
