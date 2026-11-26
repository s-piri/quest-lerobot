"""Forward and inverse kinematics of a serial arm read from a URDF. Standard library only.

Joint values here are URDF radians and poses are in the model's base frame.
Neither is the follower's calibrated coordinates: `UrdfMapping` converts, and
only a bench-verified mapping makes this model describe a real arm.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from functools import cache
from hashlib import sha256
from importlib.resources import files
from math import atan2, cos, sin, sqrt
from xml.etree import ElementTree

from .contracts import finite, vector
from .pose import IDENTITY, Pose, conjugate, multiply, rotate, unit_quaternion

SO101_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
SO101_MODEL = "so101_new_calib.urdf"
# TheRobotStudio/SO-ARM100 at 141aea7bcac2744d92998e30268c413202a56f96, unmodified.
SO101_SHA256 = "651ea00a3efe02a3c984b21a1b52273bd1b320ed635b3eae92fd52e99d0a393b"


def _about(axis, angle: float) -> tuple[float, ...]:
    half = angle / 2
    return (*(a * sin(half) for a in axis), cos(half))


def _rpy(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """URDF fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    q = multiply(_about((0, 0, 1), yaw), _about((0, 1, 0), pitch))
    return multiply(q, _about((1, 0, 0), roll))


def _numbers(text: str | None, name: str) -> tuple[float, ...]:
    try:
        return vector([float(part) for part in (text or "0 0 0").split()], 3, name)
    except ValueError as exc:
        raise ValueError(f"{name} must be three numbers") from exc


def _cross(a, b) -> tuple[float, ...]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(v) -> float:
    return sqrt(sum(c * c for c in v))


def _limited(v, limit: float) -> tuple[float, ...]:
    length = _norm(v)
    return tuple(v) if length <= limit else tuple(c * limit / length for c in v)


def _rotation_vector(q) -> tuple[float, ...]:
    """Axis times angle, the short way round."""
    x, y, z, w = q if q[3] >= 0 else tuple(-c for c in q)
    length = sqrt(x * x + y * y + z * z)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    k = 2 * atan2(length, w) / length
    return (x * k, y * k, z * k)


def _solve3(a, b) -> tuple[float, ...]:
    """Solve a 3x3 system by elimination with partial pivoting."""
    m = [[*row, value] for row, value in zip(a, b, strict=True)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        for row in range(col + 1, 3):
            factor = m[row][col] / m[col][col]
            for k in range(col, 4):
                m[row][k] -= factor * m[col][k]
    x = [0.0, 0.0, 0.0]
    for row in (2, 1, 0):
        x[row] = (m[row][3] - sum(m[row][k] * x[k] for k in range(row + 1, 3))) / m[row][row]
    return tuple(x)


def _damped_gram(rows, damping: float):
    """J Jt + damping^2 I for a Jacobian given as three rows."""
    return [
        [
            sum(a * b for a, b in zip(rows[i], rows[j], strict=True))
            + (damping * damping if i == j else 0.0)
            for j in range(3)
        ]
        for i in range(3)
    ]


def _times(rows, v) -> tuple[float, ...]:
    return tuple(sum(a * b for a, b in zip(row, v, strict=True)) for row in rows)


def _transposed_times(rows, y) -> tuple[float, ...]:
    return tuple(sum(row[k] * y[i] for i, row in enumerate(rows)) for k in range(len(rows[0])))


@dataclass(frozen=True)
class _Joint:
    name: str
    origin: Pose  # the joint frame in its parent link
    axis: tuple[float, ...] | None  # None for a fixed joint
    lower: float
    upper: float


class UrdfKinematics:
    """One chain of a URDF, from `base` to `tip`, driven by the named revolute joints."""

    def __init__(
        self,
        urdf: str,
        *,
        base: str,
        tip: str,
        joints: tuple[str, ...],
        tolerance: float = 0.001,
        iterations: int = 80,
    ):
        root = ElementTree.fromstring(urdf)
        if root.tag != "robot":
            raise ValueError("a URDF document has a <robot> root")
        by_child = {}
        for element in root.findall("joint"):
            child = element.find("child")
            if child is not None:
                by_child[child.get("link")] = element
        chain = []
        link = tip
        while link != base:
            element = by_child.get(link)
            if element is None:
                raise ValueError(f"no chain of joints leads from {base} to {tip}")
            chain.append(self._joint(element))
            link = element.find("parent").get("link")
            if len(chain) > len(by_child):
                raise ValueError("the URDF joints form a loop")
        self._chain = tuple(reversed(chain))
        moving = tuple(joint.name for joint in self._chain if joint.axis is not None)
        if moving != tuple(joints):
            raise ValueError(f"expected the moving joints {joints}, but the chain has {moving}")
        self.joints = moving
        self.lower = tuple(j.lower for j in self._chain if j.axis is not None)
        self.upper = tuple(j.upper for j in self._chain if j.axis is not None)
        self.tolerance = finite(tolerance, "tolerance")
        if self.tolerance <= 0 or type(iterations) is not int or iterations < 1:
            raise ValueError("tolerance and iterations must be positive")
        self.iterations = iterations

    @staticmethod
    def _joint(element) -> _Joint:
        name = element.get("name")
        pose = _origin(element, name)
        kind = element.get("type")
        if kind == "fixed":
            return _Joint(name, pose, None, 0.0, 0.0)
        if kind != "revolute":
            raise ValueError(
                f"joint {name} is {kind}; only revolute and fixed joints are supported"
            )
        axis = _numbers(element.find("axis").get("xyz"), f"{name} axis")
        length = _norm(axis)
        if length < 1e-9:
            raise ValueError(f"joint {name} has no axis")
        limit = element.find("limit")
        if limit is None:
            raise ValueError(f"joint {name} has no limits")
        lower = finite(float(limit.get("lower")), f"{name} lower limit")
        upper = finite(float(limit.get("upper")), f"{name} upper limit")
        if lower >= upper:
            raise ValueError(f"joint {name} has empty limits")
        return _Joint(name, pose, tuple(a / length for a in axis), lower, upper)

    def _forward(self, joints):
        """The tip pose, and each moving joint's origin and axis in the base frame."""
        orientation, position = IDENTITY, (0.0, 0.0, 0.0)
        values = iter(joints)
        frames = []
        for joint in self._chain:
            moved = rotate(orientation, joint.origin.position)
            position = tuple(p + m for p, m in zip(position, moved, strict=True))
            orientation = multiply(orientation, joint.origin.orientation)
            if joint.axis is not None:
                frames.append((position, rotate(orientation, joint.axis)))
                orientation = multiply(orientation, _about(joint.axis, next(values)))
        return (position, orientation), frames

    def fk(self, joints) -> Pose:
        joints = vector(joints, len(self.joints), "URDF joints")
        (position, orientation), _ = self._forward(joints)
        return Pose(position, unit_quaternion(orientation, "tip orientation"))

    def ik(self, target: Pose, seed) -> tuple[float, ...] | None:
        """Reach the target position; turn toward its orientation, staying near the seed.

        An arm with fewer than six joints cannot reach every orientation, so
        position is solved first and orientation only in the freedom left over.
        That freedom is weighed against distance from the seed: without it, a
        small unreachable turn can swing the joints a long way for no real gain.
        Called every tick with the previous command as seed, orientation closes
        in over a few ticks. None means the position is out of reach from here.
        """
        if not isinstance(target, Pose):
            raise ValueError("the target must be a pose")
        start = [
            min(max(value, low), high)
            for value, low, high in zip(
                vector(seed, len(self.joints), "seed"), self.lower, self.upper, strict=True
            )
        ]
        q = list(start)
        # First both tasks together, then position alone: the orientation task can
        # settle slightly off target near a singular pose, and position must win.
        for turning in (True, False):
            for _ in range(self.iterations):
                (position, orientation), frames = self._forward(q)
                reach = tuple(t - p for t, p in zip(target.position, position, strict=True))
                if not turning and _norm(reach) <= self.tolerance / 10:
                    break
                columns = [
                    (
                        _cross(axis, tuple(p - o for p, o in zip(position, origin, strict=True))),
                        axis,
                    )
                    for origin, axis in frames
                ]
                linear = [[column[0][i] for column in columns] for i in range(3)]
                gram = _damped_gram(linear, 0.01)
                step = _transposed_times(linear, _solve3(gram, _limited(reach, 0.05)))
                if turning:
                    turn = _rotation_vector(multiply(target.orientation, conjugate(orientation)))
                    angular = [[column[1][i] for column in columns] for i in range(3)]
                    step = self._with_orientation(
                        linear, angular, gram, reach, turn, step, start, q
                    )
                moved = [
                    min(max(value + delta, low), high)
                    for value, delta, low, high in zip(
                        q, _limited(step, 0.3), self.lower, self.upper, strict=True
                    )
                ]
                change = max(abs(a - b) for a, b in zip(moved, q, strict=True))
                q = moved
                if change < 1e-9:
                    break
        (position, _), _ = self._forward(q)
        error = _norm(tuple(t - p for t, p in zip(target.position, position, strict=True)))
        return tuple(q) if error <= self.tolerance else None

    @staticmethod
    def _with_orientation(linear, angular, gram, reach, turn, first, start, q):
        """Add a turn toward the target in the freedom the position task leaves over."""
        # Barely damped: whatever this projection let through would push the
        # position off target, and a heavier damping lets through millimetres.
        exact = _damped_gram(linear, 1e-4)

        def spare(v):
            """The part of a joint motion that leaves the position where it is."""
            back = _transposed_times(linear, _solve3(exact, _times(linear, v)))
            return tuple(a - b for a, b in zip(v, back, strict=True))

        # Orientation acts only in that spare freedom, pulled back toward the seed.
        free = [spare(row) for row in angular]
        home = spare(tuple(a - b for a, b in zip(start, q, strict=True)))
        left = tuple(
            t - a - h
            for t, a, h in zip(
                _limited(turn, 0.3), _times(angular, first), _times(free, home), strict=True
            )
        )
        toward = _transposed_times(free, _solve3(_damped_gram(free, 0.3), left))
        second = tuple(h + t for h, t in zip(home, toward, strict=True))
        # Position has the last word: it also cancels what the damping let through.
        wanted = tuple(
            r - s for r, s in zip(_limited(reach, 0.05), _times(linear, second), strict=True)
        )
        first = _transposed_times(linear, _solve3(gram, wanted))
        return tuple(a + b for a, b in zip(first, second, strict=True))


def _origin(element, name: str) -> Pose:
    origin = element.find("origin")
    xyz = _numbers(None if origin is None else origin.get("xyz"), f"{name} origin xyz")
    rpy = _numbers(None if origin is None else origin.get("rpy"), f"{name} origin rpy")
    return Pose(xyz, _rpy(*rpy))


def scene(urdf: str) -> dict:
    """The whole joint tree and every link's meshes, as plain data for a renderer.

    Joints are listed parents first. Poses are a position and an xyzw quaternion:
    a joint's is its frame in the parent link, a visual's is the mesh in its link.
    """
    root = ElementTree.fromstring(urdf)
    if root.tag != "robot":
        raise ValueError("a URDF document has a <robot> root")
    colors = {}
    for material in root.findall("material"):
        color = material.find("color")
        if color is not None:
            rgba = [float(part) for part in color.get("rgba").split()]
            colors[material.get("name")] = vector(rgba, 4, "material colour")
    pending = []
    for element in root.findall("joint"):
        joint = UrdfKinematics._joint(element)
        pending.append(
            (element.find("parent").get("link"), element.find("child").get("link"), joint)
        )
    children = {child for _, child, _ in pending}
    bases = sorted({parent for parent, _, _ in pending} - children)
    if len(bases) != 1:
        raise ValueError("the URDF must have exactly one base link")
    placed, joints = {bases[0]}, []
    while pending:
        ready = [item for item in pending if item[0] in placed]
        if not ready:
            raise ValueError("the URDF joints do not form a tree from the base")
        for parent, child, joint in ready:
            if child in placed:
                raise ValueError(f"link {child} has more than one parent joint")
            placed.add(child)
            joints.append(
                {
                    "name": joint.name,
                    "parent": parent,
                    "child": child,
                    "position": list(joint.origin.position),
                    "orientation": list(joint.origin.orientation),
                    "axis": None if joint.axis is None else list(joint.axis),
                    "lower": joint.lower,
                    "upper": joint.upper,
                }
            )
        pending = [item for item in pending if item not in ready]
    visuals = []
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError(f"link {link.get('name')} has a visual that is not a mesh")
            pose = _origin(visual, f"{link.get('name')} visual")
            material = visual.find("material")
            name = None if material is None else material.get("name")
            visuals.append(
                {
                    "link": link.get("name"),
                    "mesh": mesh.get("filename").replace("\\", "/").rsplit("/", 1)[-1],
                    "position": list(pose.position),
                    "orientation": list(pose.orientation),
                    "color": list(colors.get(name, (0.7, 0.7, 0.7, 1.0))),
                }
            )
    return {"base": bases[0], "joints": joints, "visuals": visuals}


def link_poses(tree: dict, values: Mapping[str, float]) -> dict[str, Pose]:
    """Every link's pose in the base frame. Joints without a value sit at zero."""
    poses = {tree["base"]: Pose((0.0, 0.0, 0.0), IDENTITY)}
    for joint in tree["joints"]:
        parent = poses[joint["parent"]]
        moved = rotate(parent.orientation, joint["position"])
        position = tuple(p + m for p, m in zip(parent.position, moved, strict=True))
        orientation = multiply(parent.orientation, joint["orientation"])
        if joint["axis"] is not None:
            angle = finite(values.get(joint["name"], 0.0), joint["name"])
            orientation = multiply(orientation, _about(joint["axis"], angle))
        poses[joint["child"]] = Pose(position, unit_quaternion(orientation, "link orientation"))
    return poses


@cache
def so101() -> UrdfKinematics:
    """The packaged, unmodified upstream SO-101 model, from its base to the gripper frame."""
    data = files(__package__).joinpath("models", SO101_MODEL).read_bytes()
    if sha256(data).hexdigest() != SO101_SHA256:
        raise ValueError("the packaged SO-101 model is not the recorded upstream file")
    return UrdfKinematics(
        data.decode("utf-8"), base="base_link", tip="gripper_frame_link", joints=SO101_JOINTS
    )


@cache
def so101_scene() -> dict:
    """The packaged SO-101 model's joint tree and meshes. Treat the result as read-only."""
    data = files(__package__).joinpath("models", SO101_MODEL).read_bytes()
    if sha256(data).hexdigest() != SO101_SHA256:
        raise ValueError("the packaged SO-101 model is not the recorded upstream file")
    return scene(data.decode("utf-8"))
