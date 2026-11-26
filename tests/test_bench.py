import json
import random
from math import degrees, radians

import pytest
from test_control import CAL, OTHER_CAL, REST, limits

from lerobot_teleoperator_quest.bench import REFERENCE_POSES, Camera, Setup, fit_mapping, main
from lerobot_teleoperator_quest.pose import IDENTITY, Pose
from lerobot_teleoperator_quest.so101 import FOLLOWER_KEYS, UrdfMapping

MAPPING = UrdfMapping((1, -1, 1, -1, 1), (3.0, -12.5, 0.0, 41.0, -7.25), CAL)
CAMERA = Camera(
    "gripper_link", Pose((0.01, 0.02, -0.03), IDENTITY), 640, 480, 600.0, 601.5, 320.5, 239.0
)

# Zero, then each joint alone by 45 degrees: the procedure in docs/bench.md.
POSES = [[0.0] * 5] + [[radians(45) if j == i else 0.0 for j in range(5)] for i in range(5)]


def readings(mapping, pose, noise=0.0, rng=None):
    exact = [
        z + s * degrees(q)
        for z, s, q in zip(mapping.zero_degrees, mapping.signs, pose, strict=True)
    ]
    return [value + (rng.uniform(-noise, noise) if rng else 0.0) for value in exact]


def test_a_full_setup_survives_the_file_exactly(tmp_path):
    setup = Setup(MAPPING, limits(), (0, 0, 0.7071067811865476, 0.7071067811865476), CAMERA)
    path = tmp_path / "left.json"
    setup.save(path)
    loaded = Setup.load(path)
    assert loaded == setup
    assert loaded.calibration == CAL
    assert json.loads(path.read_text(encoding="utf-8"))["version"] == 1
    # The loaded mapping is the same object in effect: it refuses another calibration.
    assert loaded.mapping.to_urdf(REST, CAL) == MAPPING.to_urdf(REST, CAL)
    with pytest.raises(ValueError, match="calibration does not match"):
        loaded.mapping.to_urdf(REST, OTHER_CAL)


def test_a_setup_may_hold_only_the_mapping(tmp_path):
    path = tmp_path / "arm.json"
    Setup(MAPPING).save(path)
    loaded = Setup.load(path)
    assert (loaded.limits, loaded.xr_to_base, loaded.camera) == (None, None, None)
    assert loaded.mapping == MAPPING


def change(setup_dict, path, value):
    data = json.loads(json.dumps(setup_dict))
    target = data
    for key in path[:-1]:
        target = target[key]
    if value is KeyError:
        del target[path[-1]]
    else:
        target[path[-1]] = value
    return data


@pytest.mark.parametrize(
    "path, value",
    [
        (["version"], 2),
        (["version"], True),
        (["version"], KeyError),
        (["extra"], 1),
        (["calibration"], "not a fingerprint"),
        (["mapping", "signs"], [1, 1, 1, 1, 0.5]),
        (["mapping", "zero_degrees"], [0, 0, 0, 0]),
        (["mapping", "unit"], "degrees"),
        (["limits", "joint_speed"], [1, 1, 1, 1, -1]),
        (["limits", "max_step_s"], KeyError),
        (["limits", "note"], "x"),
        (["xr_to_base"], [0, 0, 0, 0]),
        (["xr_to_base"], [0, 0, 1]),
        (["camera", "width"], 640.0),
        (["camera", "height"], 0),
        (["camera", "fx"], -1),
        (["camera", "cy"], float("nan")),
        (["camera", "orientation"], [0, 0, 0, 3]),
        (["camera", "distortion"], [0.1, 0.2]),
        (["camera", "link"], ""),
        (["camera", "position"], KeyError),
        (["camera", "serial"], "abc"),
    ],
)
def test_a_damaged_setup_is_refused(path, value):
    good = Setup(MAPPING, limits(), IDENTITY, CAMERA).to_dict()
    assert Setup.from_dict(good) == Setup(MAPPING, limits(), IDENTITY, CAMERA)
    with pytest.raises(ValueError):
        Setup.from_dict(change(good, path, value))


def test_files_that_are_not_setups_are_refused(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(ValueError, match="not a setup file"):
        Setup.load(path)
    path.write_text("[]", encoding="utf-8")
    with pytest.raises(ValueError):
        Setup.load(path)
    with pytest.raises(ValueError):
        Setup(None)
    with pytest.raises(ValueError):
        Setup(MAPPING, limits="wide")
    with pytest.raises(ValueError):
        Setup(MAPPING, camera={"link": "gripper_link"})


def test_camera_distortion_defaults_to_none_and_is_kept():
    assert CAMERA.distortion == (0.0,) * 5
    data = CAMERA.to_dict()
    del data["distortion"]
    assert Camera.from_dict(data) == CAMERA
    bent = Camera.from_dict(CAMERA.to_dict() | {"distortion": [0.1, -0.05, 0.001, 0.002, 0.01]})
    assert bent.distortion == (0.1, -0.05, 0.001, 0.002, 0.01)


def test_fitting_recovers_directions_and_zeros_from_exact_poses():
    samples = [(pose, readings(MAPPING, pose)) for pose in POSES]
    fitted, worst = fit_mapping(samples, CAL)
    assert fitted.signs == MAPPING.signs
    assert fitted.zero_degrees == pytest.approx(MAPPING.zero_degrees)
    assert fitted.calibration == CAL
    assert worst == pytest.approx((0, 0, 0, 0, 0), abs=1e-9)


def test_fitting_tolerates_hand_posing_and_reports_how_well_it_fits():
    rng = random.Random(3)
    samples = [(pose, readings(MAPPING, pose, 4.0, rng)) for pose in POSES]
    fitted, worst = fit_mapping(samples, CAL)
    assert fitted.signs == MAPPING.signs
    for got, wanted in zip(fitted.zero_degrees, MAPPING.zero_degrees, strict=True):
        assert abs(got - wanted) < 4.0
    assert all(0 < w <= 8.0 for w in worst)


def test_one_badly_matched_pose_shows_in_that_joints_disagreement():
    samples = [(pose, readings(MAPPING, pose)) for pose in POSES]
    samples[0][1][2] += 12.0  # the elbow was 12 degrees off when "zero" was recorded
    _, worst = fit_mapping(samples, CAL)
    assert worst[2] > 5
    assert max(worst[:2] + worst[3:]) < 1e-9


def test_fitting_refuses_what_the_poses_cannot_settle():
    still = [(pose, readings(MAPPING, pose)) for pose in POSES[:5]]  # wrist_roll never moved
    with pytest.raises(ValueError, match="joint 5 was not posed"):
        fit_mapping(still, CAL)
    with pytest.raises(ValueError, match="at least two poses"):
        fit_mapping(still[:1], CAL)
    # Readings that do not follow the joint at all: neither direction fits.
    noise = [(pose, [0.0, 0.0, 0.0, 0.0, 0.0]) for pose in POSES]
    with pytest.raises(ValueError, match="do not settle its direction"):
        fit_mapping(noise, CAL)
    with pytest.raises(ValueError):
        fit_mapping([(POSES[0], [0.0] * 4), (POSES[1], [0.0] * 5)], CAL)
    with pytest.raises(ValueError):
        fit_mapping([1, 2], CAL)
    with pytest.raises(ValueError, match="fingerprint"):
        fit_mapping([(pose, readings(MAPPING, pose)) for pose in POSES], "not a fingerprint")


def write_samples(path, mapping, named=True):
    records = []
    for pose in POSES:
        follower = readings(mapping, pose)
        if named:
            follower = dict(zip(FOLLOWER_KEYS, [*follower, 50.0], strict=True))
        records.append({"model": pose, "follower": follower})
    path.write_text(json.dumps(records), encoding="utf-8")


def fit_command(folder, out="left.json"):
    return main(
        [
            "fit",
            "--samples",
            str(folder / "samples.json"),
            "--calibration",
            str(folder / "cal.json"),
            "--out",
            str(folder / out),
        ]
    )


def test_reference_poses_are_zero_then_each_joint_alone():
    assert [list(pose) for pose in REFERENCE_POSES] == POSES


def test_the_fit_command_writes_a_setup_bound_to_the_calibration_file(tmp_path, capsys):
    calibration = {"shoulder_pan": {"id": 1, "homing_offset": 12}}
    (tmp_path / "cal.json").write_text(json.dumps(calibration), encoding="utf-8")
    write_samples(tmp_path / "samples.json", MAPPING)
    assert fit_command(tmp_path) == 0
    printed = capsys.readouterr().out
    assert "not a verification" in printed
    assert "shoulder_lift.pos" in printed and "direction -1" in printed
    assert "gripper.pos" not in printed  # the gripper is not a mapped joint
    setup = Setup.load(tmp_path / "left.json")
    assert setup.calibration == CAL  # the same content test_control fingerprints
    assert setup.mapping.signs == MAPPING.signs
    assert setup.mapping.zero_degrees == pytest.approx(MAPPING.zero_degrees)

    # A second run must not silently replace a setup someone may be relying on.
    assert fit_command(tmp_path) == 1
    assert "already exists" in capsys.readouterr().err

    left, cal = str(tmp_path / "left.json"), str(tmp_path / "cal.json")
    assert main(["show", left, "--calibration", cal]) == 0
    assert "matches that calibration" in capsys.readouterr().out
    other = tmp_path / "other.json"
    other.write_text(json.dumps({"shoulder_pan": {"id": 1, "homing_offset": 13}}), encoding="utf-8")
    assert main(["show", left, "--calibration", str(other)]) == 1
    assert "different calibration file" in capsys.readouterr().err
    assert main(["show", left]) == 0


def test_the_fit_command_explains_bad_input_instead_of_tracing(tmp_path, capsys):
    (tmp_path / "cal.json").write_text(json.dumps({"a": 1}), encoding="utf-8")
    cases = {
        "not a list": "{}",
        "extra field": json.dumps([{"model": [0] * 5, "follower": [0] * 5, "note": "x"}]),
        "missing joint": json.dumps(
            [{"model": [0] * 5, "follower": {"shoulder_pan.pos": 1.0}}] * 2
        ),
        "one pose": json.dumps([{"model": [0] * 5, "follower": [0] * 5}]),
        "not json": "{",
    }
    for name, text in cases.items():
        (tmp_path / "samples.json").write_text(text, encoding="utf-8")
        assert fit_command(tmp_path, "out.json") == 1, name
        assert capsys.readouterr().err.startswith("error: "), name
        assert not (tmp_path / "out.json").exists(), name
    assert main(["show", str(tmp_path / "missing.json")]) == 1
    assert capsys.readouterr().err.startswith("error: ")
    write_samples(tmp_path / "samples.json", MAPPING, named=False)  # plain lists work too
    assert fit_command(tmp_path, "out.json") == 0
