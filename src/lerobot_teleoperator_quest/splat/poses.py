"""Camera poses for a wrist-camera recording, from the arm's joint readings.

Standard library only. The world is the arm's base frame, in metres, and a
camera pose is the camera frame in that world with OpenCV axes: x right, y
down, z out through the lens.

Poses from joints are a starting guess for reconstruction to refine. Servo
backlash and link flex put them off by millimetres, which is too much for a
sharp splat on their own, but they give real scale and the arm's own frame.
"""

import json
import shutil
from dataclasses import dataclass
from math import acos, radians, sqrt
from pathlib import Path, PurePosixPath

from ..bench import Camera, Setup
from ..contracts import So101State, finite, timestamp
from ..kinematics import SO101_JOINTS, link_poses, so101_scene
from ..pose import Pose, compose, inverse, rotation_matrix
from ..so101 import from_follower_observation

SHOTS_FILE = "shots.jsonl"


@dataclass(frozen=True)
class Shot:
    """One frame of a recording: when, what the follower read, and its image file."""

    time: float  # seconds on the recording's own clock
    state: So101State
    image: str  # relative to the recording folder


def read_recording(folder) -> list[Shot]:
    """Read `shots.jsonl` beside the images it names.

    Each line is `{"time": seconds, "image": "images/000000.png", "follower": {...}}`
    with the follower's six `.pos` readings. Times must increase.
    """
    folder = Path(folder)
    source = folder / SHOTS_FILE
    if not source.is_file():
        raise ValueError(f"{folder} is not a recording: it has no {SHOTS_FILE}")
    shots = []
    for number, line in enumerate(source.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        where = f"{source} line {number}"
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{where} is not JSON: {exc}") from exc
        if not isinstance(record, dict) or set(record) != {"time", "image", "follower"}:
            raise ValueError(f'{where} must contain exactly "time", "image" and "follower"')
        image = record["image"]
        if not isinstance(image, str) or not image:
            raise ValueError(f"{where}: image must be a file name")
        path = PurePosixPath(image.replace("\\", "/"))
        if path.is_absolute() or ".." in path.parts or ":" in image:
            raise ValueError(f"{where}: image must be a path inside the recording folder")
        if not (folder / path).is_file():
            raise ValueError(f"{where}: image {image} is missing")
        try:
            time = timestamp(record["time"])
            state = from_follower_observation(record["follower"])
        except ValueError as exc:
            raise ValueError(f"{where}: {exc}") from exc
        if shots and time <= shots[-1].time:
            raise ValueError(f"{where}: times must increase")
        shots.append(Shot(time, state, str(path)))
    if len(shots) < 2:
        raise ValueError(f"{source} needs at least two frames")
    return shots


def state_at(shots: list[Shot], time: float) -> So101State | None:
    """The reading at a moment between frames, or None outside the recording."""
    if not shots[0].time <= time <= shots[-1].time:
        return None
    low, high = 0, len(shots) - 1
    while high - low > 1:
        middle = (low + high) // 2
        if shots[middle].time <= time:
            low = middle
        else:
            high = middle
    before, after = shots[low], shots[high]
    part = (time - before.time) / (after.time - before.time)

    def between(a: float, b: float) -> float:
        return a + (b - a) * part

    joints = tuple(
        between(a, b) for a, b in zip(before.state.joints_rad, after.state.joints_rad, strict=True)
    )
    return So101State(joints, between(before.state.gripper, after.state.gripper))


def camera_poses(
    shots: list[Shot], setup: Setup, calibration: str, *, camera_delay: float = 0.0
) -> list[Pose | None]:
    """Where the camera was for each image.

    `camera_delay` is how many seconds older the image is than the reading
    stored beside it. A pose is None when no reading covers that moment.
    """
    if setup.camera is None:
        raise ValueError("this setup has no camera: calibrate the wrist camera first")
    camera_delay = finite(camera_delay, "camera_delay")
    tree = so101_scene()
    if setup.camera.link not in {tree["base"], *(joint["child"] for joint in tree["joints"])}:
        raise ValueError(f"the model has no link named {setup.camera.link}")
    poses = []
    for shot in shots:
        state = state_at(shots, shot.time - camera_delay)
        if state is None:
            poses.append(None)
            continue
        joints = setup.mapping.to_urdf(state, calibration)
        link = link_poses(tree, dict(zip(SO101_JOINTS, joints, strict=True)))[setup.camera.link]
        poses.append(compose(link, setup.camera.pose))
    return poses


def _distance(a, b) -> float:
    return sqrt(sum((x - y) ** 2 for x, y in zip(a, b, strict=True)))


def _turn(a, b) -> float:
    return 2 * acos(min(1.0, abs(sum(x * y for x, y in zip(a, b, strict=True)))))


NO_READING = "no joint reading for that moment"
TOO_FAST = "camera moving too fast"
TOO_CLOSE = "too close to the previous view"


def choose(
    shots: list[Shot],
    poses: list[Pose | None],
    *,
    min_move: float = 0.01,
    min_turn: float = radians(3),
    max_speed: float | None = 0.15,
) -> tuple[list[int], dict[str, int]]:
    """Pick the views worth reconstructing from, and count why others were left out.

    A view is kept when the camera has moved `min_move` metres or turned
    `min_turn` radians since the last kept one. Views taken while the camera
    moves faster than `max_speed` metres a second are dropped: they blur, and
    any error in `camera_delay` costs most there.
    """
    kept, dropped = [], {NO_READING: 0, TOO_FAST: 0, TOO_CLOSE: 0}
    last = None
    for index, pose in enumerate(poses):
        if pose is None:
            dropped[NO_READING] += 1
            continue
        if max_speed is not None:
            near = [
                i for i in (index - 1, index + 1) if 0 <= i < len(poses) and poses[i] is not None
            ]
            speeds = [
                _distance(pose.position, poses[i].position) / abs(shots[i].time - shots[index].time)
                for i in near
            ]
            if speeds and max(speeds) > max_speed:
                dropped[TOO_FAST] += 1
                continue
        if last is not None:
            moved = _distance(pose.position, last.position) >= min_move
            turned = _turn(pose.orientation, last.orientation) >= min_turn
            if not moved and not turned:
                dropped[TOO_CLOSE] += 1
                continue
        kept.append(index)
        last = pose
    return kept, dropped


def nerfstudio(camera: Camera, views: list[tuple[str, Pose]]) -> dict:
    """A nerfstudio `transforms.json`: camera-to-world matrices with OpenGL camera axes.

    The trainer must be told not to re-orient, re-centre or re-scale the poses,
    or the splat will no longer sit in the arm's base frame.
    """
    k1, k2, p1, p2, k3 = camera.distortion
    frames = []
    for name, pose in views:
        r = rotation_matrix(pose.orientation)
        c = pose.position
        # OpenCV looks along +z with y down; OpenGL along -z with y up.
        matrix = [[r[row][0], -r[row][1], -r[row][2], c[row]] for row in range(3)]
        frames.append({"file_path": name, "transform_matrix": [*matrix, [0.0, 0.0, 0.0, 1.0]]})
    return {
        "camera_model": "OPENCV",
        "w": camera.width,
        "h": camera.height,
        "fl_x": camera.fx,
        "fl_y": camera.fy,
        "cx": camera.cx,
        "cy": camera.cy,
        "k1": k1,
        "k2": k2,
        "p1": p1,
        "p2": p2,
        "k3": k3,
        "frames": frames,
    }


def colmap(camera: Camera, views: list[tuple[str, Pose]]) -> dict[str, str]:
    """A COLMAP text model with known poses and no points, as file name to text.

    COLMAP stores world-to-camera: a quaternion written w first, then a translation.
    """
    k1, k2, p1, p2, k3 = camera.distortion
    numbers = [camera.fx, camera.fy, camera.cx, camera.cy, k1, k2, p1, p2]
    model = "OPENCV"
    if k3 != 0:
        model, numbers = "FULL_OPENCV", [*numbers, k3, 0.0, 0.0, 0.0]
    cameras = (
        "# Camera list with one line of data per camera:\n"
        "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n"
        f"1 {model} {camera.width} {camera.height} {' '.join(repr(float(n)) for n in numbers)}\n"
    )
    lines = [
        "# Image list with two lines of data per image:",
        "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME",
        "#   POINTS2D[] as (X, Y, POINT3D_ID)",
    ]
    for number, (name, pose) in enumerate(views, start=1):
        if any(c.isspace() for c in name):
            raise ValueError(f"image name {name!r} must not contain spaces")
        seen = inverse(pose)
        x, y, z, w = seen.orientation
        values = " ".join(repr(float(v)) for v in (w, x, y, z, *seen.position))
        lines += [f"{number} {values} 1 {name}", ""]
    points = "# 3D point list: empty, to be triangulated from the known poses.\n"
    return {"cameras.txt": cameras, "images.txt": "\n".join(lines) + "\n", "points3D.txt": points}


def export(
    recording,
    setup: Setup,
    calibration: str,
    out,
    *,
    camera_delay: float = 0.0,
    min_move: float = 0.01,
    min_turn: float = radians(3),
    max_speed: float | None = 0.15,
) -> dict:
    """Write a scene folder: the chosen images, `transforms.json`, a COLMAP model and a report.

    `out` must not already hold anything. Returns the report.
    """
    recording, out = Path(recording), Path(out)
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise ValueError(f"{out} already exists and is not empty; choose a new folder")
    shots = read_recording(recording)
    poses = camera_poses(shots, setup, calibration, camera_delay=camera_delay)
    kept, dropped = choose(shots, poses, min_move=min_move, min_turn=min_turn, max_speed=max_speed)
    if len(kept) < 2:
        raise ValueError(
            f"only {len(kept)} of {len(shots)} frames are usable "
            f"({', '.join(f'{count} {why}' for why, count in dropped.items() if count)})"
        )
    (out / "images").mkdir(parents=True)
    (out / "colmap").mkdir()
    views, names = [], []
    for number, index in enumerate(kept):
        source = recording / shots[index].image
        name = f"{number:06d}{source.suffix.lower()}"
        shutil.copyfile(source, out / "images" / name)
        views.append((f"images/{name}", poses[index]))
        names.append((name, poses[index]))
    (out / "transforms.json").write_text(
        json.dumps(nerfstudio(setup.camera, views), indent=1, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    for name, text in colmap(setup.camera, names).items():
        (out / "colmap" / name).write_text(text, encoding="utf-8")
    report = {
        "world": "the arm's base frame, in metres",
        "calibration": calibration,
        "frames": len(shots),
        "kept": len(kept),
        "dropped": dropped,
        "source_frames": [shots[index].image for index in kept],
        "options": {
            "camera_delay": camera_delay,
            "min_move": min_move,
            "min_turn": min_turn,
            "max_speed": max_speed,
        },
    }
    (out / "report.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    return report
