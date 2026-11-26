"""Camera calibration against synthetic photographs of a board with a known answer."""

import json
import random
from math import acos, degrees

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

from test_control import CAL, OTHER_CAL
from test_splat_poses import MAPPING

from lerobot_teleoperator_quest.bench import Setup
from lerobot_teleoperator_quest.contracts import So101State
from lerobot_teleoperator_quest.kinematics import SO101_JOINTS, _about, link_poses, so101_scene
from lerobot_teleoperator_quest.pose import Pose, compose, inverse, multiply, rotation_matrix
from lerobot_teleoperator_quest.so101 import to_follower_action
from lerobot_teleoperator_quest.splat.__main__ import main
from lerobot_teleoperator_quest.splat.calibrate import (
    Board,
    calibrate,
    calibrate_lens,
    calibrate_mount,
    find_corners,
)

BOARD = Board(9, 6, 0.02)
SIZE = (640, 480)
LENS = (610.0, 606.0, 318.0, 244.0)
# The answer the calibration must find: where the camera really sits on the gripper link.
MOUNT = Pose((0.012, 0.047, -0.021), multiply(_about((0, 0, 1), 0.35), _about((1, 0, 0), -0.6)))
PIXELS_PER_SQUARE = 80


def board_texture():
    """A flat picture of the board with a white border, and its pixel-to-metres matrix."""
    squares = (BOARD.columns + 1, BOARD.rows + 1)
    border = 1
    width = (squares[0] + 2 * border) * PIXELS_PER_SQUARE
    height = (squares[1] + 2 * border) * PIXELS_PER_SQUARE
    picture = np.full((height, width), 255, np.uint8)
    for column in range(squares[0]):
        for row in range(squares[1]):
            if (column + row) % 2 == 0:
                x = (column + border) * PIXELS_PER_SQUARE
                y = (row + border) * PIXELS_PER_SQUARE
                picture[y : y + PIXELS_PER_SQUARE, x : x + PIXELS_PER_SQUARE] = 0
    # The first inner corner is one square in from the first drawn square.
    scale = BOARD.square / PIXELS_PER_SQUARE
    origin = (border + 1) * PIXELS_PER_SQUARE
    to_metres = np.array([[scale, 0, -origin * scale], [0, scale, -origin * scale], [0, 0, 1]])
    return picture, to_metres


def photograph(board_in_camera: Pose):
    """What an ideal camera with LENS sees of the board in that pose."""
    picture, to_metres = board_texture()
    rotation = np.array(rotation_matrix(board_in_camera.orientation))
    position = np.array(board_in_camera.position)
    lens = np.array([[LENS[0], 0, LENS[2]], [0, LENS[1], LENS[3]], [0, 0, 1]])
    plane = np.column_stack([rotation[:, 0], rotation[:, 1], position])
    return cv2.warpPerspective(
        picture, lens @ plane @ to_metres, SIZE, flags=cv2.INTER_LINEAR, borderValue=160
    )


def link_pose(state):
    joints = MAPPING.to_urdf(state, CAL)
    return link_poses(so101_scene(), dict(zip(SO101_JOINTS, joints, strict=True)))["gripper_link"]


def in_view(board_in_camera: Pose) -> bool:
    lens = np.array([[LENS[0], 0, LENS[2]], [0, LENS[1], LENS[3]], [0, 0, 1]])
    rotation = np.array(rotation_matrix(board_in_camera.orientation))
    corners = BOARD.points().astype(float)
    # Include the outer squares, which the detector needs to see as well.
    outer = np.array(
        [
            [-BOARD.square, -BOARD.square, 0],
            [BOARD.columns * BOARD.square, BOARD.rows * BOARD.square, 0],
        ]
    )
    seen = (rotation @ np.vstack([corners, outer]).T).T + np.array(board_in_camera.position)
    if (seen[:, 2] < 0.12).any() or rotation[2, 2] > -0.5:  # too close, or seen too edge-on
        return False
    pixels = (lens @ (seen / seen[:, 2:3]).T).T
    return bool(
        (pixels[:, 0] > 25).all()
        and (pixels[:, 0] < SIZE[0] - 25).all()
        and (pixels[:, 1] > 25).all()
        and (pixels[:, 1] < SIZE[1] - 25).all()
    )


@pytest.fixture(scope="module")
def shoot():
    """Still shots of a board that stays put, from arm poses that keep it in view."""
    rng = random.Random(5)
    start = So101State((0.1, 0.3, 0.2, 0.9, 0.2), 0.5)
    first = compose(link_pose(start), MOUNT)
    # Lay the board a third of a metre in front of the first view, facing the camera.
    facing = _about((1, 0, 0), 3.141592653589793)
    board = compose(first, Pose((-0.08, 0.05, 0.34), facing))
    shots = []
    for _ in range(4000):
        if len(shots) == 14:
            break
        joints = tuple(j + rng.uniform(-0.3, 0.3) for j in start.joints_rad)
        low, high = so101_scene_limits()
        if any(
            not lo < MAPPING.to_urdf(So101State(joints, 0.5), CAL)[i] < hi
            for i, (lo, hi) in enumerate(zip(low, high, strict=True))
        ):
            continue
        state = So101State(joints, 0.5)
        seen = compose(inverse(compose(link_pose(state), MOUNT)), board)
        if in_view(seen):
            shots.append((state, seen))
    assert len(shots) == 14, "the synthetic arm could not find enough views of the board"
    return shots


@pytest.fixture(scope="module")
def corners(shoot):
    found = [find_corners(photograph(seen), BOARD) for _, seen in shoot]
    assert all(c is not None for c in found)
    return found


def so101_scene_limits():
    joints = [joint for joint in so101_scene()["joints"] if joint["name"] in SO101_JOINTS]
    return [joint["lower"] for joint in joints], [joint["upper"] for joint in joints]


def angle(a, b):
    return 2 * acos(min(1.0, abs(sum(x * y for x, y in zip(a, b, strict=True)))))


def test_the_board_is_found_in_a_photograph_and_not_in_a_blank_one(shoot):
    corners = find_corners(photograph(shoot[0][1]), BOARD)
    assert corners.shape == (54, 2)
    colour = cv2.cvtColor(photograph(shoot[0][1]), cv2.COLOR_GRAY2BGR)
    assert find_corners(colour, BOARD) is not None
    assert find_corners(np.full((480, 640), 128, np.uint8), BOARD) is None


def test_the_lens_and_the_mount_are_recovered_from_photographs(shoot, corners):
    (fx, fy, cx, cy, distortion), error, boards = calibrate_lens(corners, BOARD, SIZE)
    assert error < 0.3  # pixels
    assert fx == pytest.approx(LENS[0], rel=0.02) and fy == pytest.approx(LENS[1], rel=0.02)
    assert cx == pytest.approx(LENS[2], abs=8) and cy == pytest.approx(LENS[3], abs=8)
    assert max(abs(v) for v in distortion[:2]) < 0.1  # the synthetic lens has none

    mount, spread = calibrate_mount([link_pose(state) for state, _ in shoot], boards)
    assert (
        sum((a - b) ** 2 for a, b in zip(mount.position, MOUNT.position, strict=True)) ** 0.5
        < 0.004
    )
    assert degrees(angle(mount.orientation, MOUNT.orientation)) < 1.0
    assert spread < 0.004


def test_a_wrong_mapping_shows_up_as_a_board_that_will_not_stay_put(shoot, corners):
    _, _, boards = calibrate_lens(corners, BOARD, SIZE)
    # Link poses computed as if the elbow read 12 degrees differently than it does.
    skewed = [
        link_pose(
            So101State(
                (s.joints_rad[0], s.joints_rad[1], s.joints_rad[2] * 1.6 + 0.2, *s.joints_rad[3:]),
                0.5,
            )
        )
        for s, _ in shoot
    ]
    _, spread = calibrate_mount(skewed, boards)
    assert spread > 0.01


def write_shots(folder, shots, blanks=0):
    (folder / "images").mkdir(parents=True)
    lines = []
    for index, (state, seen) in enumerate(shots):
        name = f"images/{index:04d}.png"
        blank = index < blanks
        picture = np.full((SIZE[1], SIZE[0]), 128, np.uint8) if blank else photograph(seen)
        assert cv2.imwrite(str(folder / name), picture)
        lines.append(
            json.dumps({"time": float(index), "image": name, "follower": to_follower_action(state)})
        )
    (folder / "shots.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return folder


def test_a_recording_of_stills_becomes_a_camera_for_the_setup(tmp_path, shoot):
    folder = write_shots(tmp_path / "board", shoot, blanks=2)
    camera, report = calibrate(folder, Setup(MAPPING), CAL, BOARD)
    assert (report["shots"], report["used"], report["board_not_found"]) == (14, 12, 2)
    assert report["lens_error_pixels"] < 0.3 and report["board_spread_metres"] < 0.005
    assert (camera.link, camera.width, camera.height) == ("gripper_link", 640, 480)
    assert camera.fx == pytest.approx(LENS[0], rel=0.02)
    assert degrees(angle(camera.pose.orientation, MOUNT.orientation)) < 1.5
    # The result drops straight into a setup file and survives it.
    setup = Setup(MAPPING, camera=camera)
    setup.save(tmp_path / "with_camera.json")
    assert Setup.load(tmp_path / "with_camera.json").camera == camera

    with pytest.raises(ValueError, match="calibration does not match"):
        calibrate(folder, Setup(MAPPING), OTHER_CAL, BOARD)
    with pytest.raises(ValueError, match="no link named"):
        calibrate(folder, Setup(MAPPING), CAL, BOARD, link="nowhere")


def test_too_few_views_are_refused_with_the_count(tmp_path, shoot):
    folder = write_shots(tmp_path / "few", shoot[:9], blanks=3)
    with pytest.raises(ValueError, match="at least 8 views of the board; 6 were usable"):
        calibrate(folder, Setup(MAPPING), CAL, BOARD)
    with pytest.raises(ValueError, match="at least 4 views"):
        calibrate_mount([link_pose(shoot[0][0])] * 3, [(np.eye(3), np.zeros((3, 1)))] * 3)


@pytest.mark.parametrize(
    "columns, rows, square",
    [
        (9, 7, 0.02),
        (8, 6, 0.02),
        (2, 5, 0.02),
        (9, 6, 0),
        (9, 6, -0.02),
        (9.0, 6, 0.02),
        (9, 6, float("nan")),
    ],
)
def test_boards_that_cannot_work_are_refused(columns, rows, square):
    with pytest.raises(ValueError):
        Board(columns, rows, square)


def test_the_calibrate_command_writes_a_new_setup_and_says_how_good_it_is(tmp_path, shoot, capsys):
    folder = write_shots(tmp_path / "board", shoot, blanks=1)
    calibration = {"shoulder_pan": {"id": 1, "homing_offset": 12}}  # what CAL fingerprints
    (tmp_path / "cal.json").write_text(json.dumps(calibration), encoding="utf-8")
    Setup(MAPPING).save(tmp_path / "setup.json")
    options = [
        str(folder),
        "--setup",
        str(tmp_path / "setup.json"),
        "--calibration",
        str(tmp_path / "cal.json"),
        "--square",
        "0.02",
    ]
    assert main(["calibrate", *options, "--out", str(tmp_path / "with_camera.json")]) == 0
    printed = capsys.readouterr().out
    assert "used 13 of 14 shots; the whole board was not found in 1." in printed
    assert "pixels of error" in printed and "mm of each other" in printed
    assert "<- high" not in printed
    done = Setup.load(tmp_path / "with_camera.json")
    assert done.mapping == MAPPING and done.camera.link == "gripper_link"
    assert Setup.load(tmp_path / "setup.json").camera is None  # the original is untouched

    assert main(["calibrate", *options, "--out", str(tmp_path / "with_camera.json")]) == 1
    assert "already exists" in capsys.readouterr().err
    assert main(["calibrate", *options, "--board", "9by6", "--out", str(tmp_path / "x.json")]) == 1
    assert "like 9x6" in capsys.readouterr().err
    assert main(["calibrate", *options, "--board", "8x6", "--out", str(tmp_path / "x.json")]) == 1
    assert "upside down" in capsys.readouterr().err
    assert not (tmp_path / "x.json").exists()
