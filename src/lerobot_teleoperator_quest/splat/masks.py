"""Masks that hide the gripper from reconstruction.

Needs numpy and OpenCV (the `splat` extra). The wrist camera always sees the
gripper it is mounted on. Left in, it would be reconstructed as an object
floating beside every viewpoint, so each exported view gets a mask: white
where the picture is scene, black where it is arm.

The mask is drawn from the arm's meshes through the calibrated camera, then
grown by a margin, because printed parts, the mount and the mesh never agree
exactly. The jaw's angle is a straight-line guess from the gripper reading.
Always look at the check picture before trusting the masks.
"""

import json
from pathlib import Path

import cv2
import numpy as np

from ..bench import Camera
from ..contracts import unit_interval
from ..kinematics import link_poses, so101_scene
from ..pose import Pose, compose, inverse, rotation_matrix

JAW_JOINT = "gripper"
NEAR = 0.004  # metres: triangles closer to the lens than this are not drawn


def read_stl(path) -> np.ndarray:
    """Triangles of a binary STL file as an array of shape (count, 3 corners, 3)."""
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path} is not a binary STL file")
    count = int(np.frombuffer(data, "<u4", 1, 80)[0])
    if len(data) != 84 + count * 50:
        raise ValueError(f"{path} is truncated or not a binary STL file")
    record = np.dtype([("normal", "<f4", 3), ("corners", "<f4", (3, 3)), ("attribute", "<u2")])
    return np.frombuffer(data, record, count, 84)["corners"].astype(np.float64)


def _below(tree: dict, link: str) -> set[str]:
    """The link and everything attached beyond it."""
    names = {link}
    for joint in tree["joints"]:  # parents come first
        if joint["parent"] in names:
            names.add(joint["child"])
    return names


def load_meshes(folder, camera: Camera) -> dict[str, np.ndarray]:
    """The meshes of every part that moves with the camera, by file name."""
    tree = so101_scene()
    wanted = {v["mesh"] for v in tree["visuals"] if v["link"] in _below(tree, camera.link)}
    meshes = {}
    for name in sorted(wanted):
        path = Path(folder) / name
        if not path.is_file():
            raise ValueError(f"mesh {name} is missing from {folder}")
        meshes[name] = read_stl(path)
    return meshes


def _apply(pose: Pose, points: np.ndarray) -> np.ndarray:
    return points @ np.array(rotation_matrix(pose.orientation)).T + np.array(pose.position)


def arm_in_view(camera: Camera, meshes: dict[str, np.ndarray], gripper: float) -> np.ndarray:
    """True for each pixel covered by a part that moves with the camera."""
    gripper = unit_interval(gripper, "gripper")
    tree = so101_scene()
    jaw = next((joint for joint in tree["joints"] if joint["name"] == JAW_JOINT), None)
    angle = 0.0 if jaw is None else jaw["lower"] + gripper * (jaw["upper"] - jaw["lower"])
    poses = link_poses(tree, {JAW_JOINT: angle})
    seen_from_camera = inverse(compose(poses[camera.link], camera.pose))
    lens = np.array([[camera.fx, 0, camera.cx], [0, camera.fy, camera.cy], [0, 0, 1]])
    distortion = np.array(camera.distortion)
    covered = np.zeros((camera.height, camera.width), np.uint8)
    for visual in tree["visuals"]:
        if visual["link"] not in _below(tree, camera.link):
            continue
        part = compose(poses[visual["link"]], Pose(visual["position"], visual["orientation"]))
        corners = _apply(compose(seen_from_camera, part), meshes[visual["mesh"]].reshape(-1, 3))
        triangles = corners.reshape(-1, 3, 3)
        triangles = triangles[(triangles[:, :, 2] > NEAR).all(axis=1)]
        if not len(triangles):
            continue
        pixels, _ = cv2.projectPoints(
            triangles.reshape(-1, 3), np.zeros(3), np.zeros(3), lens, distortion
        )
        # Far outside the picture, coordinates can exceed what the drawing call accepts.
        pixels = np.clip(pixels.reshape(-1, 3, 2), -1e5, 1e5)
        for triangle in np.round(pixels).astype(np.int32):
            cv2.fillConvexPoly(covered, triangle, 255)
    return covered > 0


def gripper_mask(
    camera: Camera, meshes: dict[str, np.ndarray], gripper: float, *, margin: int = 12
) -> np.ndarray:
    """White where reconstruction may look, black on the arm and `margin` pixels around it."""
    if type(margin) is not int or margin < 0:
        raise ValueError("margin must be a whole number of pixels, zero or more")
    covered = arm_in_view(camera, meshes, gripper).astype(np.uint8)
    if margin:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * margin + 1, 2 * margin + 1))
        covered = cv2.dilate(covered, kernel)
    return np.where(covered > 0, 0, 255).astype(np.uint8)


def write_masks(scene, camera: Camera, meshes: dict[str, np.ndarray], *, margin: int = 12) -> dict:
    """Add a mask for every view of an exported scene, and a picture to check them by.

    Masks go in `masks/`, named as COLMAP expects (the image's name with `.png`
    added), and `transforms.json` gains each frame's `mask_path`. Returns how
    much of the picture was hidden, least and most, as fractions.
    """
    scene = Path(scene)
    report = json.loads((scene / "report.json").read_text(encoding="utf-8"))
    transforms = json.loads((scene / "transforms.json").read_text(encoding="utf-8"))
    if len(report.get("gripper", [])) != len(transforms["frames"]):
        raise ValueError(f"{scene} was not written by this version's export; export it again")
    (scene / "masks").mkdir(exist_ok=True)
    cache, hidden = {}, []
    for frame, gripper in zip(transforms["frames"], report["gripper"], strict=True):
        key = round(gripper, 2)  # a scan holds the gripper still, so one drawing serves all
        if key not in cache:
            cache[key] = gripper_mask(camera, meshes, key, margin=margin)
        mask = cache[key]
        name = f"masks/{Path(frame['file_path']).name}.png"
        if not cv2.imwrite(str(scene / name), mask):
            raise OSError(f"could not write {scene / name}")
        frame["mask_path"] = name
        hidden.append(float((mask == 0).mean()))
    (scene / "transforms.json").write_text(
        json.dumps(transforms, indent=1) + "\n", encoding="utf-8"
    )

    # The first view with its masked area tinted red, for a person to judge.
    first = transforms["frames"][0]
    picture = cv2.imread(str(scene / first["file_path"]))
    check = None
    if picture is not None and picture.shape[:2] == (camera.height, camera.width):
        tint = picture.copy()
        tint[cache[round(report["gripper"][0], 2)] == 0] = (0, 0, 255)
        cv2.imwrite(str(scene / "masks_check.png"), cv2.addWeighted(picture, 0.55, tint, 0.45, 0))
        check = "masks_check.png"
    result = {"least_hidden": min(hidden), "most_hidden": max(hidden), "check": check}
    report["masks"] = {"margin": margin, **result}
    (scene / "report.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    return result
