import json
import subprocess
import sys
from math import radians
from pathlib import Path

import pytest
from test_control import CAL, OTHER_CAL

from lerobot_teleoperator_quest.bench import Camera, Setup
from lerobot_teleoperator_quest.contracts import So101State
from lerobot_teleoperator_quest.kinematics import SO101_JOINTS, _about, link_poses, so101_scene
from lerobot_teleoperator_quest.pose import Pose, compose, conjugate, rotate
from lerobot_teleoperator_quest.so101 import UrdfMapping, to_follower_action
from lerobot_teleoperator_quest.splat.__main__ import main
from lerobot_teleoperator_quest.splat.poses import (
    NO_READING,
    TOO_CLOSE,
    TOO_FAST,
    Shot,
    camera_poses,
    choose,
    colmap,
    export,
    nerfstudio,
    read_recording,
    state_at,
)

MAPPING = UrdfMapping((1, -1, 1, 1, 1), (0, 10, 0, 0, 0), CAL)
CAMERA = Camera(
    "gripper_link",
    Pose((0.01, 0.03, -0.02), _about((1, 0, 0), 0.4)),
    640,
    480,
    600.0,
    605.0,
    322.0,
    238.0,
)
SETUP = Setup(MAPPING, camera=CAMERA)


def sweep(count, step=0.02):
    """A slow scan: every joint creeps a little further each frame."""
    return [
        So101State((0.1 + step * i, 0.2 + step * i / 2, 0.3, 0.1 * i * step, 0.0), 0.5)
        for i in range(count)
    ]


def make_recording(folder, states, times=None, suffix=".png"):
    (folder / "images").mkdir(parents=True)
    lines = []
    for index, state in enumerate(states):
        name = f"images/frame{index:04d}{suffix}"
        (folder / name).write_bytes(f"image {index}".encode())
        time = index / 30 if times is None else times[index]
        lines.append(
            json.dumps({"time": time, "image": name, "follower": to_follower_action(state)})
        )
    (folder / "shots.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return folder


def expected_pose(state):
    joints = MAPPING.to_urdf(state, CAL)
    link = link_poses(so101_scene(), dict(zip(SO101_JOINTS, joints, strict=True)))["gripper_link"]
    return compose(link, CAMERA.pose)


def pixel(camera, point):
    """Project a point given in the camera frame with OpenCV axes."""
    return (
        camera.fx * point[0] / point[2] + camera.cx,
        camera.fy * point[1] / point[2] + camera.cy,
    )


def test_a_recording_is_read_back_as_written(tmp_path):
    states = sweep(5)
    shots = read_recording(make_recording(tmp_path / "scan", states))
    assert [shot.image for shot in shots] == [f"images/frame{i:04d}.png" for i in range(5)]
    assert shots[3].time == pytest.approx(0.1)
    for shot, state in zip(shots, states, strict=True):
        assert shot.state.joints_rad == pytest.approx(state.joints_rad)


@pytest.mark.parametrize(
    "damage",
    [
        lambda lines: lines[:1],
        lambda lines: [*lines[:2], "{not json"],
        lambda lines: [*lines, json.dumps({"time": 9, "image": "images/frame0000.png"})],
        lambda lines: [*lines, json.dumps(json.loads(lines[0]) | {"time": 9, "note": "x"})],
        lambda lines: [*lines, json.dumps(json.loads(lines[0]) | {"time": 0.01})],
        lambda lines: [
            *lines,
            json.dumps(json.loads(lines[0]) | {"time": 9, "image": "images/gone.png"}),
        ],
        lambda lines: [
            *lines,
            json.dumps(json.loads(lines[0]) | {"time": 9, "image": "../shots.jsonl"}),
        ],
        lambda lines: [*lines, json.dumps(json.loads(lines[0]) | {"time": 9, "image": "C:/x.png"})],
        lambda lines: [*lines, json.dumps(json.loads(lines[0]) | {"time": 9, "image": ""})],
        lambda lines: [
            *lines,
            json.dumps(json.loads(lines[0]) | {"time": 9, "follower": {"a": 1}}),
        ],
        lambda lines: [*lines, json.dumps(json.loads(lines[0]) | {"time": -1})],
    ],
)
def test_a_damaged_recording_is_refused_with_its_line(tmp_path, damage):
    folder = make_recording(tmp_path / "scan", sweep(3))
    lines = (folder / "shots.jsonl").read_text(encoding="utf-8").splitlines()
    (folder / "shots.jsonl").write_text("\n".join(damage(lines)) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="shots.jsonl"):
        read_recording(folder)


def test_a_folder_without_shots_is_not_a_recording(tmp_path):
    with pytest.raises(ValueError, match="not a recording"):
        read_recording(tmp_path)


def test_readings_between_frames_are_interpolated():
    shots = [Shot(float(i), state, f"{i}.png") for i, state in enumerate(sweep(3, step=0.1))]
    half = state_at(shots, 0.5)
    assert half.joints_rad == pytest.approx((0.15, 0.225, 0.3, 0.005, 0.0))
    assert state_at(shots, 2.0) == shots[2].state
    assert state_at(shots, 0.0) == shots[0].state
    assert state_at(shots, -0.01) is None
    assert state_at(shots, 2.01) is None


def test_camera_poses_follow_the_mapping_the_model_and_the_mount(tmp_path):
    states = sweep(6)
    shots = read_recording(make_recording(tmp_path / "scan", states))
    poses = camera_poses(shots, SETUP, CAL)
    for pose, state in zip(poses, states, strict=True):
        wanted = expected_pose(state)
        assert pose.position == pytest.approx(wanted.position, abs=1e-9)
        assert pose.orientation == pytest.approx(wanted.orientation, abs=1e-9)
    with pytest.raises(ValueError, match="calibration does not match"):
        camera_poses(shots, SETUP, OTHER_CAL)
    with pytest.raises(ValueError, match="no camera"):
        camera_poses(shots, Setup(MAPPING), CAL)
    elsewhere = Camera("no_such_link", CAMERA.pose, 640, 480, 600.0, 600.0, 320.0, 240.0)
    with pytest.raises(ValueError, match="no link named"):
        camera_poses(shots, Setup(MAPPING, camera=elsewhere), CAL)


def test_a_late_image_is_paired_with_the_earlier_reading(tmp_path):
    states = sweep(6)
    shots = read_recording(make_recording(tmp_path / "scan", states))
    late = camera_poses(shots, SETUP, CAL, camera_delay=1 / 60)  # half a frame
    assert late[0] is None  # nothing was read before the recording began
    for index in range(1, 6):
        between = state_at(shots, shots[index].time - 1 / 60)
        assert late[index].position == pytest.approx(expected_pose(between).position, abs=1e-9)
    early = camera_poses(shots, SETUP, CAL, camera_delay=-1 / 60)
    assert early[-1] is None and early[0] is not None


def test_both_formats_put_a_world_point_on_the_same_pixel(tmp_path):
    states = sweep(8, step=0.05)
    shots = read_recording(make_recording(tmp_path / "scan", states))
    poses = camera_poses(shots, SETUP, CAL)
    views = [(f"{i:06d}.png", pose) for i, pose in enumerate(poses)]
    scene = nerfstudio(CAMERA, views)
    lines = [
        line for line in colmap(CAMERA, views)["images.txt"].splitlines() if line[:1].isdigit()
    ]
    assert len(lines) == len(views) == len(scene["frames"])
    for (name, pose), line, frame in zip(views, lines, scene["frames"], strict=True):
        # A point a quarter of a metre in front of this camera, a little off axis.
        world = compose(pose, Pose((0.03, -0.02, 0.25), (0, 0, 0, 1))).position
        offset = tuple(w - c for w, c in zip(world, pose.position, strict=True))
        direct = pixel(CAMERA, rotate(conjugate(pose.orientation), offset))
        assert direct == pytest.approx(pixel(CAMERA, (0.03, -0.02, 0.25)))

        # COLMAP: X_camera = R(q) X_world + t, with q written w first.
        fields = line.split()
        assert fields[0] == str(views.index((name, pose)) + 1) and fields[-2:] == ["1", name]
        qw, qx, qy, qz, tx, ty, tz = map(float, fields[1:8])
        seen = tuple(
            r + t for r, t in zip(rotate((qx, qy, qz, qw), world), (tx, ty, tz), strict=True)
        )
        assert pixel(CAMERA, seen) == pytest.approx(direct, abs=1e-6)

        # nerfstudio: camera-to-world with OpenGL axes, so invert and flip y and z.
        m = frame["transform_matrix"]
        assert m[3] == [0.0, 0.0, 0.0, 1.0]
        offset = [world[row] - m[row][3] for row in range(3)]
        gl = [sum(m[row][axis] * offset[row] for row in range(3)) for axis in range(3)]
        assert gl[2] < 0  # in front of an OpenGL camera
        assert pixel(CAMERA, (gl[0], -gl[1], -gl[2])) == pytest.approx(direct, abs=1e-6)


def test_the_camera_model_is_written_for_both_tools():
    views = [("a.png", expected_pose(sweep(1)[0]))]
    scene = nerfstudio(CAMERA, views)
    assert (scene["camera_model"], scene["w"], scene["h"]) == ("OPENCV", 640, 480)
    assert (scene["fl_x"], scene["fl_y"], scene["cx"], scene["cy"]) == (600.0, 605.0, 322.0, 238.0)
    plain = colmap(CAMERA, views)["cameras.txt"].splitlines()[-1].split()
    assert plain[:4] == ["1", "OPENCV", "640", "480"] and len(plain) == 12
    bent = Camera.from_dict(CAMERA.to_dict() | {"distortion": [0.1, -0.05, 0.001, 0.002, 0.01]})
    full = colmap(bent, views)["cameras.txt"].splitlines()[-1].split()
    assert full[1] == "FULL_OPENCV" and len(full) == 16
    assert [float(v) for v in full[8:13]] == [0.1, -0.05, 0.001, 0.002, 0.01]
    assert nerfstudio(bent, views)["k3"] == 0.01
    with pytest.raises(ValueError, match="spaces"):
        colmap(CAMERA, [("my image.png", views[0][1])])
    assert set(colmap(CAMERA, views)) == {"cameras.txt", "images.txt", "points3D.txt"}


def test_views_are_chosen_for_movement_and_dropped_for_speed():
    still = [sweep(1)[0]] * 4
    moving = sweep(6, step=0.02)
    jump = [So101State((1.2, 0.2, 0.3, 0.0, 0.0), 0.5)]  # a lunge between two frames
    states = [*still, *moving, *jump]
    shots = [Shot(i / 30, state, f"{i}.png") for i, state in enumerate(states)]
    poses = [expected_pose(state) for state in states]
    poses[2] = None

    kept, dropped = choose(shots, poses, min_move=0.004, min_turn=radians(1), max_speed=0.3)
    assert kept[0] == 0
    assert dropped[NO_READING] == 1
    assert dropped[TOO_CLOSE] >= 2  # the frames where nothing moved
    assert dropped[TOO_FAST] == 2  # the lunge and the frame it leapt from
    assert 10 not in kept and 9 not in kept
    assert len(kept) + sum(dropped.values()) == len(states)

    everything, none = choose(
        shots, [p or poses[0] for p in poses], min_move=0, min_turn=0, max_speed=None
    )
    assert everything == list(range(len(states))) and sum(none.values()) == 0


def test_export_writes_a_self_contained_scene(tmp_path):
    states = sweep(12)
    folder = make_recording(tmp_path / "scan", states, suffix=".JPG")
    out = tmp_path / "scene"
    report = export(folder, SETUP, CAL, out, min_move=0.004, max_speed=None)
    assert report["frames"] == 12 and report["kept"] >= 6
    assert report["world"] == "the arm's base frame, in metres"
    assert json.loads((out / "report.json").read_text(encoding="utf-8")) == report

    scene = json.loads((out / "transforms.json").read_text(encoding="utf-8"))
    assert [frame["file_path"] for frame in scene["frames"]] == [
        f"images/{i:06d}.jpg" for i in range(report["kept"])
    ]
    for frame, source in zip(scene["frames"], report["source_frames"], strict=True):
        assert (out / frame["file_path"]).read_bytes() == (folder / source).read_bytes()
    images = (out / "colmap" / "images.txt").read_text(encoding="utf-8")
    assert f"{report['kept']} " in images and "000000.jpg" in images and "images/" not in images
    assert (out / "colmap" / "points3D.txt").exists()

    with pytest.raises(ValueError, match="not empty"):
        export(folder, SETUP, CAL, out)
    (tmp_path / "empty").mkdir()
    export(
        folder, SETUP, CAL, tmp_path / "empty", min_move=0.004, max_speed=None
    )  # an empty folder is fine


def test_export_says_why_too_few_frames_are_usable(tmp_path):
    folder = make_recording(tmp_path / "scan", [sweep(1)[0]] * 6)
    with pytest.raises(ValueError, match=r"only 1 of 6 frames are usable \(5 too close"):
        export(folder, SETUP, CAL, tmp_path / "scene")
    assert not (tmp_path / "scene").exists()
    with pytest.raises(ValueError, match="no camera"):
        export(folder, Setup(MAPPING), CAL, tmp_path / "scene")


def test_importing_the_package_does_not_import_the_splat_tools():
    source = Path(__file__).resolve().parents[1] / "src"
    probe = (
        "import sys, lerobot_teleoperator_quest as package; "
        "assert not any(name.startswith(package.__name__ + '.splat') for name in sys.modules)"
    )
    result = subprocess.run(
        [sys.executable, "-c", probe], cwd=source, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr


def test_the_export_command_checks_the_calibration_and_reports_what_it_dropped(tmp_path, capsys):
    calibration = {"shoulder_pan": {"id": 1, "homing_offset": 12}}  # what CAL fingerprints
    (tmp_path / "cal.json").write_text(json.dumps(calibration), encoding="utf-8")
    SETUP.save(tmp_path / "setup.json")
    folder = make_recording(tmp_path / "scan", [sweep(1)[0]] * 3 + sweep(10))
    common = ["--setup", str(tmp_path / "setup.json"), "--calibration", str(tmp_path / "cal.json")]
    slow = ["--min-move", "0.004", "--any-speed"]

    assert main(["export", str(folder), *common, "--out", str(tmp_path / "scene"), *slow]) == 0
    printed = capsys.readouterr().out
    assert "kept" in printed and "of 13 frames" in printed
    assert "too close to the previous view" in printed
    assert "Few views were kept" in printed  # ten views is nowhere near enough for a splat
    assert "arm's base frame" in printed
    assert (tmp_path / "scene" / "transforms.json").is_file()

    assert main(["export", str(folder), *common, "--out", str(tmp_path / "scene")]) == 1
    assert "not empty" in capsys.readouterr().err

    (tmp_path / "other.json").write_text(json.dumps({"shoulder_pan": {"id": 2}}), encoding="utf-8")
    wrong = ["--setup", str(tmp_path / "setup.json"), "--calibration", str(tmp_path / "other.json")]
    assert main(["export", str(folder), *wrong, "--out", str(tmp_path / "scene2")]) == 1
    assert "different calibration file" in capsys.readouterr().err
    assert not (tmp_path / "scene2").exists()

    Setup(MAPPING).save(tmp_path / "bare.json")
    bare = ["--setup", str(tmp_path / "bare.json"), "--calibration", str(tmp_path / "cal.json")]
    assert main(["export", str(folder), *bare, "--out", str(tmp_path / "scene3")]) == 1
    assert "no camera" in capsys.readouterr().err
    assert (
        main(["export", str(tmp_path / "nowhere"), *common, "--out", str(tmp_path / "scene4")]) == 1
    )
    assert "not a recording" in capsys.readouterr().err
