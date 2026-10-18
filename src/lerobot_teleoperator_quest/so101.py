"""Explicit official-follower contracts; no serial access or calibration writes."""

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass
from math import degrees, radians

from .contracts import So101State, finite, vector

FOLLOWER_KEYS = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
)


def to_follower_action(state: So101State) -> dict[str, float]:
    values = (*map(degrees, state.joints_rad), state.gripper * 100)
    return dict(zip(FOLLOWER_KEYS, vector(values, 6, "follower command"), strict=True))


def from_follower_observation(observation: Mapping) -> So101State:
    if not isinstance(observation, Mapping) or not set(FOLLOWER_KEYS) <= observation.keys():
        raise ValueError("observation must include all six SO-101 .pos keys")
    values = [finite(observation[k], k) for k in FOLLOWER_KEYS]
    return So101State(tuple(map(radians, values[:5])), values[5] / 100)


def from_follower_action(action: Mapping) -> So101State:
    if not isinstance(action, Mapping) or set(action) != set(FOLLOWER_KEYS):
        raise ValueError("action must contain exactly the six SO-101 .pos keys")
    return from_follower_observation(action)


def calibration_fingerprint(calibration: Mapping) -> str:
    """Canonical JSON hash, not evidence that a physical mapping was verified."""
    if not isinstance(calibration, Mapping) or not calibration:
        raise ValueError("calibration must be a non-empty mapping")
    canonical = json.dumps(
        dict(calibration),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class UrdfMapping:
    """Operator-verified mapping bound to one calibration file's fingerprint.

    q_urdf = sign * radians(follower_degrees - zero_degrees).
    Creating this object does not mechanically verify the arm; callers must
    supply measurements and compare against the CURRENT official calibration.
    """

    signs: tuple[float, ...]
    zero_degrees: tuple[float, ...]
    calibration: str

    def __post_init__(self):
        signs = vector(self.signs, 5, "signs")
        if any(s not in (-1, 1) for s in signs):
            raise ValueError("signs must be +1 or -1")
        object.__setattr__(self, "signs", signs)
        object.__setattr__(self, "zero_degrees", vector(self.zero_degrees, 5, "zero_degrees"))
        if (
            not isinstance(self.calibration, str)
            or len(self.calibration) != 64
            or any(c not in "0123456789abcdef" for c in self.calibration)
        ):
            raise ValueError("mapping requires a calibration SHA-256 fingerprint")

    def _check(self, calibration: str):
        if calibration != self.calibration:
            raise ValueError("calibration does not match this URDF mapping")

    def to_urdf(self, state: So101State, calibration: str) -> tuple[float, ...]:
        self._check(calibration)
        result = tuple(
            sign * (q - radians(zero))
            for sign, q, zero in zip(self.signs, state.joints_rad, self.zero_degrees, strict=True)
        )
        return vector(result, 5, "URDF joints")

    def from_urdf(self, q, gripper: float, calibration: str) -> So101State:
        self._check(calibration)
        q = vector(q, 5, "URDF joints")
        return So101State(
            tuple(
                sign * angle + radians(zero)
                for sign, angle, zero in zip(self.signs, q, self.zero_degrees, strict=True)
            ),
            gripper,
        )
