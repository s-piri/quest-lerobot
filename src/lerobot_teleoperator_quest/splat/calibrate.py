"""Calibrating the wrist camera: its lens, then where it sits on the arm.

Needs numpy and OpenCV (the `splat` extra). The input is a recording of still
shots of a printed chessboard that stays put while the arm looks at it from
many directions. Use a board with an odd number of inner corners one way and an
even number the other, such as 9 by 6, so its orientation is never ambiguous.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from ..bench import Camera, Setup
from ..contracts import finite
from ..kinematics import SO101_JOINTS, link_poses, so101_scene
from ..pose import Pose, rotation_matrix
from .poses import read_recording


@dataclass(frozen=True)
class Board:
    """A printed chessboard: inner corners across and down, and the square's side in metres."""

    columns: int
    rows: int
    square: float

    def __post_init__(self):
        if type(self.columns) is not int or type(self.rows) is not int:
            raise ValueError("board corners must be whole numbers")
        if min(self.columns, self.rows) < 3:
            raise ValueError("a board needs at least 3 inner corners each way")
        if (self.columns + self.rows) % 2 == 0:
            raise ValueError(
                "use a board with an odd number of inner corners one way and an even number "
                "the other, such as 9 by 6: otherwise it looks the same upside down"
            )
        if finite(self.square, "board square") <= 0:
            raise ValueError("the board's square size must be positive, in metres")

    def points(self) -> np.ndarray:
        """Inner corners in the board's own frame, row by row, in metres."""
        grid = np.zeros((self.rows * self.columns, 3), np.float32)
        grid[:, :2] = np.mgrid[0 : self.columns, 0 : self.rows].T.reshape(-1, 2) * self.square
        return grid


def find_corners(image: np.ndarray, board: Board) -> np.ndarray | None:
    """The board's inner corners in pixels, row by row, or None if it is not fully in view."""
    grey = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        grey, (board.columns, board.rows), flags=cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_EXHAUSTIVE
    )
    return corners.reshape(-1, 2).astype(np.float32) if found else None


def _matrix(pose: Pose) -> tuple[np.ndarray, np.ndarray]:
    return np.array(rotation_matrix(pose.orientation)), np.array(pose.position).reshape(3, 1)


def _pose(rotation: np.ndarray, translation: np.ndarray) -> Pose:
    r = np.asarray(rotation, dtype=float)
    w = np.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        q = (
            (r[2, 1] - r[1, 2]) / (4 * w),
            (r[0, 2] - r[2, 0]) / (4 * w),
            (r[1, 0] - r[0, 1]) / (4 * w),
            w,
        )
    else:  # a half turn: take the largest diagonal term
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(max(0.0, 1 + r[i, i] - r[j, j] - r[k, k])) * 2
        q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
        q[i], q[j], q[k] = s / 4, (r[j, i] + r[i, j]) / s, (r[k, i] + r[i, k]) / s
    norm = float(np.linalg.norm(q))
    return Pose([float(v) for v in np.ravel(translation)], [float(c) / norm for c in q])


def calibrate_lens(corners: list[np.ndarray], board: Board, size: tuple[int, int]):
    """Focal lengths, centre and distortion from views of the board.

    Returns `(fx, fy, cx, cy, distortion)`, the reprojection error in pixels,
    and the board's pose in each camera view as `(rotation, translation)`.
    """
    if len(corners) < 8:
        raise ValueError(
            f"the lens needs at least 8 views of the board; {len(corners)} were usable"
        )
    objects = [board.points()] * len(corners)
    views = [c.reshape(-1, 1, 2) for c in corners]
    error, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        objects, views, size, None, None
    )
    lens = (
        float(matrix[0, 0]),
        float(matrix[1, 1]),
        float(matrix[0, 2]),
        float(matrix[1, 2]),
        tuple(float(v) for v in distortion.ravel()[:5]),
    )
    boards = [
        (cv2.Rodrigues(r)[0], t.reshape(3, 1)) for r, t in zip(rotations, translations, strict=True)
    ]
    return lens, float(error), boards


def calibrate_mount(links: list[Pose], boards: list[tuple[np.ndarray, np.ndarray]]):
    """Where the camera sits on its link, from the link's pose and the board's pose per view.

    Returns the camera frame in the link frame, and how far apart the views
    then place the board in the arm's base frame: the largest distance in
    metres from their average. A board that did not move should come out in
    one place, so this is the honest measure of the result.
    """
    if len(links) != len(boards) or len(links) < 4:
        raise ValueError("the mount needs at least 4 views, each with a link pose")
    link_r, link_t = zip(*(_matrix(pose) for pose in links), strict=True)
    board_r, board_t = zip(*boards, strict=True)
    rotation, translation = cv2.calibrateHandEye(
        list(link_r), list(link_t), list(board_r), list(board_t), method=cv2.CALIB_HAND_EYE_PARK
    )
    places = [
        lr @ (rotation @ bt + translation) + lt
        for lr, lt, bt in zip(link_r, link_t, board_t, strict=True)
    ]
    centre = np.mean(places, axis=0)
    spread = max(float(np.linalg.norm(place - centre)) for place in places)
    return _pose(rotation, translation), spread


def calibrate(
    recording, setup: Setup, calibration: str, board: Board, *, link: str = "gripper_link"
):
    """Calibrate the camera from a recording of still shots of a board.

    Returns the camera and a report: how many shots showed the whole board,
    the lens error in pixels, and the board's spread in metres.
    """
    recording = Path(recording)
    tree = so101_scene()
    if link not in {tree["base"], *(joint["child"] for joint in tree["joints"])}:
        raise ValueError(f"the model has no link named {link}")
    shots = read_recording(recording)
    corners, links, size, unreadable = [], [], None, 0
    for shot in shots:
        image = cv2.imread(str(recording / shot.image), cv2.IMREAD_GRAYSCALE)
        if image is None:
            unreadable += 1
            continue
        if size is None:
            size = (image.shape[1], image.shape[0])
        elif size != (image.shape[1], image.shape[0]):
            raise ValueError("the shots are not all the same size")
        found = find_corners(image, board)
        if found is None:
            continue
        joints = setup.mapping.to_urdf(shot.state, calibration)
        corners.append(found)
        links.append(link_poses(tree, dict(zip(SO101_JOINTS, joints, strict=True)))[link])
    if size is None:
        raise ValueError("none of the shots could be read as an image")
    (fx, fy, cx, cy, distortion), error, boards = calibrate_lens(corners, board, size)
    mount, spread = calibrate_mount(links, boards)
    camera = Camera(link, mount, size[0], size[1], fx, fy, cx, cy, distortion)
    report = {
        "shots": len(shots),
        "used": len(corners),
        "unreadable": unreadable,
        "board_not_found": len(shots) - len(corners) - unreadable,
        "lens_error_pixels": error,
        "board_spread_metres": spread,
    }
    return camera, report
