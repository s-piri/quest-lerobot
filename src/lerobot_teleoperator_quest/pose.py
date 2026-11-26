"""Metres and unit xyzw quaternions for Cartesian targets. Standard library only."""

from dataclasses import dataclass
from math import atan2, cos, sin, sqrt

from .contracts import vector

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def unit_quaternion(value, name: str) -> tuple[float, ...]:
    q = vector(value, 4, name)
    norm = sqrt(sum(c * c for c in q))
    if abs(norm - 1) > 0.001:
        raise ValueError(f"{name} must be a unit xyzw quaternion")
    return tuple(c / norm for c in q)


def multiply(a, b) -> tuple[float, ...]:
    """Hamilton product: the rotation b followed by the rotation a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def conjugate(q) -> tuple[float, ...]:
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q, v) -> tuple[float, ...]:
    return multiply(multiply(q, (*v, 0.0)), conjugate(q))[:3]


def scale_rotation(q, factor: float) -> tuple[float, ...]:
    """The same axis with its shortest-path angle multiplied by factor."""
    x, y, z, w = q if q[3] >= 0 else tuple(-c for c in q)
    length = sqrt(x * x + y * y + z * z)
    if length < 1e-12:
        return IDENTITY
    half = atan2(length, w) * factor
    k = sin(half) / length
    return (x * k, y * k, z * k, cos(half))


@dataclass(frozen=True)
class Pose:
    position: tuple[float, ...]
    orientation: tuple[float, ...]

    def __post_init__(self):
        object.__setattr__(self, "position", vector(self.position, 3, "position"))
        object.__setattr__(self, "orientation", unit_quaternion(self.orientation, "orientation"))


def compose(outer: Pose, inner: Pose) -> Pose:
    """The pose `inner`, given in the frame of `outer`, in the frame `outer` is given in."""
    moved = rotate(outer.orientation, inner.position)
    return Pose(
        tuple(p + m for p, m in zip(outer.position, moved, strict=True)),
        multiply(outer.orientation, inner.orientation),
    )


def inverse(pose: Pose) -> Pose:
    """The frame `pose` is given in, as seen from `pose`."""
    back = conjugate(pose.orientation)
    return Pose(tuple(-c for c in rotate(back, pose.position)), back)


def rotation_matrix(q) -> tuple[tuple[float, ...], ...]:
    """Rows of the 3x3 matrix that rotates a vector as `rotate(q, v)` does."""
    columns = [rotate(q, axis) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return tuple(tuple(column[row] for column in columns) for row in range(3))
