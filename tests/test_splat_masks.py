"""Gripper masks, checked against the arm's own meshes projected by independent arithmetic."""

import json
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

from test_control import CAL
from test_splat_poses import MAPPING, make_recording, sweep

from lerobot_teleoperator_quest.bench import Camera, Setup
from lerobot_teleoperator_quest.kinematics import _about, link_poses, so101_scene
from lerobot_teleoperator_quest.pose import Pose, compose, inverse, rotate
from lerobot_teleoperator_quest.splat.__main__ import main
from lerobot_teleoperator_quest.splat.masks import (
    arm_in_view,
    gripper_mask,
    load_meshes,
    read_stl,
    write_masks,
)
from lerobot_teleoperator_quest.splat.poses import export

ASSETS = Path(__file__).resolve().parents[1] / "client" / "assets"
# On the gripper link, above the motor and off to one side, looking along the jaws (its -z).
LOOKING_AT_JAWS = Camera(
    "gripper_link",
    Pose((0.0, 0.04, 0.05), _about((1, 0, 0), 3.141592653589793)),
    640,
    480,
    300.0,
    300.0,
    320.0,
    240.0,
)
LOOKING_AWAY = Camera(
    "gripper_link", Pose((0.0, 0.04, 0.05), (0, 0, 0, 1)), 640, 480, 300.0, 300.0, 320.0, 240.0
)


@pytest.fixture(scope="module")
def meshes():
    return load_meshes(ASSETS, LOOKING_AT_JAWS)


def test_only_the_parts_that_move_with_the_camera_are_loaded(meshes):
    assert set(meshes) == {
        "sts3215_03a_v1.stl",
        "wrist_roll_follower_so101_v1.stl",
        "moving_jaw_so101_v1.stl",
    }
    for triangles in meshes.values():
        assert triangles.ndim == 3 and triangles.shape[1:] == (3, 3)
    on_the_forearm = Camera(
        "lower_arm_link", LOOKING_AT_JAWS.pose, 640, 480, 300.0, 300.0, 320.0, 240.0
    )
    assert len(load_meshes(ASSETS, on_the_forearm)) > len(meshes)  # the wrist and gripper too
    with pytest.raises(ValueError, match="is missing"):
        load_meshes(ASSETS.parent, LOOKING_AT_JAWS)


def test_a_damaged_mesh_file_is_refused(tmp_path):
    (tmp_path / "short.stl").write_bytes(b"x" * 20)
    with pytest.raises(ValueError, match="not a binary STL"):
        read_stl(tmp_path / "short.stl")
    data = (ASSETS / "moving_jaw_so101_v1.stl").read_bytes()
    (tmp_path / "cut.stl").write_bytes(data[:-10])
    with pytest.raises(ValueError, match="truncated"):
        read_stl(tmp_path / "cut.stl")
    assert read_stl(ASSETS / "moving_jaw_so101_v1.stl").shape == (2500, 3, 3)


def test_mesh_corners_land_where_plain_arithmetic_puts_them(meshes):
    """The same projection done one corner at a time with the package's pose maths."""
    camera, tree = LOOKING_AT_JAWS, so101_scene()
    covered = arm_in_view(camera, meshes, 0.5)
    assert 0.02 < covered.mean() < 0.9  # the gripper is in view and does not fill it

    jaw = next(joint for joint in tree["joints"] if joint["name"] == "gripper")
    poses = link_poses(tree, {"gripper": jaw["lower"] + 0.5 * (jaw["upper"] - jaw["lower"])})
    seen_from_camera = inverse(compose(poses[camera.link], camera.pose))
    checked = 0
    for visual in tree["visuals"]:
        if visual["mesh"] not in meshes or visual["link"] not in (
            "gripper_link",
            "moving_jaw_so101_v1_link",
        ):
            continue
        part = compose(
            seen_from_camera,
            compose(poses[visual["link"]], Pose(visual["position"], visual["orientation"])),
        )
        for triangle in meshes[visual["mesh"]][::23]:
            corners = [
                tuple(
                    r + p
                    for r, p in zip(rotate(part.orientation, tuple(c)), part.position, strict=True)
                )
                for c in triangle
            ]
            if min(c[2] for c in corners) <= 0.02:
                continue
            # The middle of a triangle is inside it, wherever its edges round to.
            middle = [sum(c[axis] for c in corners) / 3 for axis in range(3)]
            u = round(camera.fx * middle[0] / middle[2] + camera.cx)
            v = round(camera.fy * middle[1] / middle[2] + camera.cy)
            if 0 <= u < camera.width and 0 <= v < camera.height:
                assert covered[v, u], (visual["mesh"], u, v)
                checked += 1
    assert checked > 20


def test_the_jaw_moves_the_mask_and_a_camera_facing_away_sees_no_arm(meshes):
    closed, opened = (
        arm_in_view(LOOKING_AT_JAWS, meshes, 0.0),
        arm_in_view(LOOKING_AT_JAWS, meshes, 1.0),
    )
    assert (closed != opened).mean() > 0.005
    assert not arm_in_view(LOOKING_AWAY, meshes, 0.5).any()
    with pytest.raises(ValueError):
        arm_in_view(LOOKING_AT_JAWS, meshes, 1.5)


def test_a_mask_is_black_on_the_arm_and_the_margin_only_grows_it(meshes):
    covered = arm_in_view(LOOKING_AT_JAWS, meshes, 0.5)
    tight = gripper_mask(LOOKING_AT_JAWS, meshes, 0.5, margin=0)
    assert tight.dtype == np.uint8 and set(np.unique(tight)) == {0, 255}
    assert ((tight == 0) == covered).all()
    wide = gripper_mask(LOOKING_AT_JAWS, meshes, 0.5, margin=12)
    assert (wide[tight == 0] == 0).all() and (wide == 0).sum() > (tight == 0).sum()
    assert (gripper_mask(LOOKING_AWAY, meshes, 0.5) == 255).all()
    for bad in (-1, 2.5):
        with pytest.raises(ValueError):
            gripper_mask(LOOKING_AT_JAWS, meshes, 0.5, margin=bad)


def scene_with_pictures(tmp_path, camera=LOOKING_AT_JAWS):
    folder = make_recording(tmp_path / "scan", sweep(8))
    for image in (folder / "images").iterdir():
        assert cv2.imwrite(str(image), np.full((480, 640, 3), 90, np.uint8))
    setup = Setup(MAPPING, camera=camera)
    export(folder, setup, CAL, tmp_path / "scene", min_move=0.004, max_speed=None)
    return folder, setup


def test_every_exported_view_gets_a_mask_both_tools_can_find(tmp_path, meshes):
    _, setup = scene_with_pictures(tmp_path)
    scene = tmp_path / "scene"
    result = write_masks(scene, setup.camera, meshes)
    assert 0.02 < result["least_hidden"] <= result["most_hidden"] < 0.9
    transforms = json.loads((scene / "transforms.json").read_text(encoding="utf-8"))
    for frame in transforms["frames"]:
        name = Path(frame["file_path"]).name
        assert frame["mask_path"] == f"masks/{name}.png"  # COLMAP's naming: image name plus .png
        mask = cv2.imread(str(scene / frame["mask_path"]), cv2.IMREAD_GRAYSCALE)
        assert mask.shape == (480, 640) and set(np.unique(mask)) == {0, 255}
    check = cv2.imread(str(scene / "masks_check.png"))
    assert check.shape == (480, 640, 3)
    hidden = (
        cv2.imread(str(scene / transforms["frames"][0]["mask_path"]), cv2.IMREAD_GRAYSCALE) == 0
    )
    assert check[hidden][:, 2].mean() > check[~hidden][:, 2].mean() + 40  # red over the gripper
    report = json.loads((scene / "report.json").read_text(encoding="utf-8"))
    assert report["masks"]["margin"] == 12 and report["masks"]["check"] == "masks_check.png"

    del report["gripper"]
    (scene / "report.json").write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(ValueError, match="export it again"):
        write_masks(scene, setup.camera, meshes)


def test_the_export_command_makes_masks_and_says_how_to_check_them(tmp_path, capsys):
    folder, setup = scene_with_pictures(tmp_path)
    (tmp_path / "cal.json").write_text(json.dumps({"shoulder_pan": {"id": 1, "homing_offset": 12}}))
    setup.save(tmp_path / "setup.json")
    options = [
        "export",
        str(folder),
        "--setup",
        str(tmp_path / "setup.json"),
        "--calibration",
        str(tmp_path / "cal.json"),
        "--min-move",
        "0.004",
        "--any-speed",
    ]
    assert main([*options, "--out", str(tmp_path / "plain")]) == 0
    printed = capsys.readouterr().out
    assert "No masks were made" in printed and "--meshes" in printed
    assert not (tmp_path / "plain" / "masks").exists()

    assert main([*options, "--out", str(tmp_path / "masked"), "--meshes", str(ASSETS)]) == 0
    printed = capsys.readouterr().out
    assert "Masks hide the gripper" in printed and "masks_check.png" in printed
    assert "<-" not in printed
    assert (tmp_path / "masked" / "masks" / "000000.png.png").is_file()

    Setup(MAPPING, camera=LOOKING_AWAY).save(tmp_path / "away.json")
    away = [*options[:3], str(tmp_path / "away.json"), *options[4:]]
    assert main([*away, "--out", str(tmp_path / "away"), "--meshes", str(ASSETS)]) == 0
    assert "nothing was hidden" in capsys.readouterr().out

    assert main([*options, "--out", str(tmp_path / "bad"), "--meshes", str(tmp_path)]) == 1
    assert "is missing" in capsys.readouterr().err
