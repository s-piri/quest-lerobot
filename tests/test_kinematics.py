import json
import random
from hashlib import sha256
from importlib.resources import files
from math import acos
from pathlib import Path

import pytest
from test_control import CAL, TICK, feedback, limits, sample

from lerobot_teleoperator_quest.contracts import So101State
from lerobot_teleoperator_quest.control import ENGAGED, Controller
from lerobot_teleoperator_quest.kinematics import (
    SO101_JOINTS,
    SO101_SHA256,
    UrdfKinematics,
    _about,
    link_poses,
    scene,
    so101,
    so101_scene,
)
from lerobot_teleoperator_quest.pose import IDENTITY, Pose, multiply
from lerobot_teleoperator_quest.so101 import FOLLOWER_KEYS, UrdfMapping

REFERENCE = json.loads(
    (Path(__file__).parent / "fixtures" / "so101_fk.json").read_text(encoding="utf-8")
)


def angle(a, b):
    return 2 * acos(min(1.0, abs(sum(x * y for x, y in zip(a, b, strict=True)))))


def distance(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b, strict=True)) ** 0.5


def inside(model, rng, margin):
    return [
        rng.uniform(low + margin, high - margin)
        for low, high in zip(model.lower, model.upper, strict=True)
    ]


def test_the_packaged_model_is_the_recorded_upstream_file():
    data = files("lerobot_teleoperator_quest").joinpath("models", "so101_new_calib.urdf")
    assert sha256(data.read_bytes()).hexdigest() == SO101_SHA256
    model = so101()
    assert model.joints == SO101_JOINTS
    assert tuple(f"{name}.pos" for name in model.joints) == FOLLOWER_KEYS[:5]
    assert model.lower == pytest.approx((-1.91986, -1.74533, -1.69, -1.65806, -2.74385))
    assert model.upper == pytest.approx((1.91986, 1.74533, 1.69, 1.65806, 2.84121))


def test_forward_kinematics_matches_an_independent_library():
    """The reference poses come from yourdfpy; see scripts/make_fk_fixtures.py."""
    model = so101()
    assert len(REFERENCE["cases"]) >= 40
    for case in REFERENCE["cases"]:
        pose = model.fk(case["joints"])
        assert pose.position == pytest.approx(case["position"], abs=1e-9)
        assert angle(pose.orientation, case["orientation"]) < 1e-6


def test_inverse_returns_to_a_reachable_pose_from_a_nearby_start():
    model, rng = so101(), random.Random(11)
    for _ in range(150):
        joints = inside(model, rng, 0.05)
        seed = [
            min(max(value + rng.uniform(-0.1, 0.1), low), high)
            for value, low, high in zip(joints, model.lower, model.upper, strict=True)
        ]
        target = model.fk(joints)
        solved = model.ik(target, seed)
        assert solved is not None
        assert all(
            low <= v <= high for v, low, high in zip(solved, model.lower, model.upper, strict=True)
        )
        reached = model.fk(solved)
        assert distance(reached.position, target.position) <= model.tolerance
        assert angle(reached.orientation, target.orientation) < 0.05


def test_repeated_solves_close_in_on_position_and_orientation():
    """As in teleop: each tick starts from the previous answer."""
    model = so101()
    target = model.fk([0.35, -0.3, 0.5, 0.7, 0.4])
    joints = [0.3, -0.4, 0.6, 0.5, -0.2]
    turns = []
    for _ in range(8):
        joints = model.ik(target, joints)
        turns.append(angle(model.fk(joints).orientation, target.orientation))
    assert turns == sorted(turns, reverse=True)
    assert turns[-1] < 1e-3
    assert distance(model.fk(joints).position, target.position) < 1e-6


def test_small_steps_solve_with_proportionate_joint_motion():
    model, rng = so101(), random.Random(12)
    changes, refused = [], 0
    for _ in range(300):
        joints = inside(model, rng, 0.2)
        here = model.fk(joints)
        target = Pose(
            tuple(p + rng.uniform(-0.005, 0.005) for p in here.position), here.orientation
        )
        solved = model.ik(target, joints)
        if solved is None:
            refused += 1  # some random poses sit at the edge of reach
            continue
        assert distance(model.fk(solved).position, target.position) <= model.tolerance
        changes.append(max(abs(a - b) for a, b in zip(solved, joints, strict=True)))
    changes.sort()
    assert refused <= 5  # measured 2 of 300 with this seed
    assert changes[len(changes) // 2] < 0.06
    assert changes[int(len(changes) * 0.95)] < 0.15


def test_position_wins_over_an_orientation_the_arm_cannot_reach():
    model, rng = so101(), random.Random(13)
    for _ in range(60):
        joints = inside(model, rng, 0.3)
        here = model.fk(joints)
        axis = [rng.gauss(0, 1) for _ in range(3)]
        length = sum(a * a for a in axis) ** 0.5
        twisted = multiply(_about([a / length for a in axis], 1.0), here.orientation)
        solved = model.ik(Pose(here.position, twisted), joints)
        assert solved is not None
        assert distance(model.fk(solved).position, here.position) <= model.tolerance
        # A one-radian turn may need about that much wrist motion, but never a wild swing.
        assert max(abs(a - b) for a, b in zip(solved, joints, strict=True)) < 1.7


def test_out_of_reach_is_refused_not_approximated():
    model = so101()
    assert model.ik(Pose((2.0, 0, 0), IDENTITY), [0.0] * 5) is None
    assert model.ik(Pose((0, 0, -1.0), IDENTITY), [0.3, 0.2, -0.4, 0.1, 0.0]) is None


@pytest.mark.parametrize("seed", [[0.0] * 4, [float("nan")] * 5, "wrong", None])
def test_malformed_arguments_are_rejected(seed):
    model = so101()
    with pytest.raises(ValueError):
        model.ik(model.fk([0.0] * 5), seed)
    with pytest.raises(ValueError):
        model.ik((0, 0, 0), [0.0] * 5)
    with pytest.raises(ValueError):
        model.fk(seed)


def test_every_link_matches_the_independent_library_with_the_jaw_moved():
    tree = so101_scene()
    assert tree["base"] == "base_link"
    assert [joint["name"] for joint in tree["joints"]][:5] == list(SO101_JOINTS)
    assert len(REFERENCE["links"]) >= 16
    for case in REFERENCE["links"]:
        poses = link_poses(tree, case["values"])
        assert set(poses) == set(case["links"])
        for name, wanted in case["links"].items():
            assert poses[name].position == pytest.approx(wanted["position"], abs=1e-9)
            assert angle(poses[name].orientation, wanted["orientation"]) < 1e-6
    # The chain used for control and the tree used for drawing agree on the tool.
    joints = REFERENCE["cases"][5]["joints"]
    tool = link_poses(tree, dict(zip(SO101_JOINTS, joints, strict=True)))["gripper_frame_link"]
    assert tool.position == pytest.approx(so101().fk(joints).position, abs=1e-12)


def test_the_headsets_model_is_built_from_the_packaged_one():
    """Rebuild with scripts/build_client_assets.py if this fails."""
    assets = Path(__file__).resolve().parents[1] / "client" / "assets"
    manifest = json.loads((assets / "so101.json").read_text(encoding="utf-8"))
    tree = so101_scene()
    assert manifest["source"]["model_sha256"] == SO101_SHA256
    for key in ("base", "joints", "visuals"):
        assert manifest[key] == json.loads(json.dumps(tree[key]))
    used = {visual["mesh"] for visual in tree["visuals"]}
    assert set(manifest["meshes"]) == used
    for name, triangles in manifest["meshes"].items():
        assert (assets / name).stat().st_size == 84 + 50 * triangles
        assert triangles <= 2500


def test_a_scene_needs_one_base_and_a_tree():
    with pytest.raises(ValueError, match="one base"):
        scene(urdf(joint("first", "a", "b") + joint("second", "c", "b")))
    with pytest.raises(ValueError):
        scene(urdf(joint("first", "a", "b") + joint("second", "a", "b")))
    with pytest.raises(ValueError):
        scene("<notrobot/>")
    small = scene(urdf(joint("second", "b", "c") + joint("first", "a", "b")))
    assert [j["name"] for j in small["joints"]] == ["first", "second"]  # parents first
    assert small["visuals"] == []
    assert link_poses(small, {"first": 0.4})["c"].position == pytest.approx((0, 0, 0.2))
    with pytest.raises(ValueError):
        link_poses(small, {"first": float("nan")})


LINKS = "".join(f'<link name="{name}"/>' for name in ("a", "b", "c"))


def urdf(joints):
    return f"<robot name='t'>{LINKS}{joints}</robot>"


def joint(
    name, parent, child, kind="revolute", limit='<limit lower="-1" upper="1"/>', axis="0 0 1"
):
    return (
        f'<joint name="{name}" type="{kind}"><origin xyz="0 0 0.1" rpy="0 0 0"/>'
        f'<parent link="{parent}"/><child link="{child}"/><axis xyz="{axis}"/>{limit}</joint>'
    )


def test_a_small_chain_is_read_in_base_to_tip_order():
    model = UrdfKinematics(
        urdf(joint("second", "b", "c", axis="0 1 0") + joint("first", "a", "b")),
        base="a",
        tip="c",
        joints=("first", "second"),
    )
    assert model.fk([0, 0]).position == pytest.approx((0, 0, 0.2))
    assert model.fk([1.2, 0]).position == pytest.approx((0, 0, 0.2))  # spinning about its own axis
    assert model.fk([0, 0.5]).orientation == pytest.approx(_about((0, 1, 0), 0.5))


@pytest.mark.parametrize(
    "document, joints",
    [
        ("<notrobot/>", ("first",)),
        (urdf(joint("first", "a", "b")), ("first",)),  # nothing reaches the tip
        (urdf(joint("first", "a", "b") + joint("second", "b", "c")), ("second", "first")),
        (urdf(joint("first", "a", "b") + joint("second", "b", "c")), ("first",)),
        (urdf(joint("first", "a", "b") + joint("second", "b", "c", kind="prismatic")), ("first",)),
        (urdf(joint("first", "a", "b") + joint("second", "b", "c", limit="")), ("first", "second")),
        (
            urdf(joint("first", "a", "b", axis="0 0 0") + joint("second", "b", "c")),
            ("first", "second"),
        ),
        (
            urdf(
                joint("first", "a", "b", limit='<limit lower="1" upper="1"/>')
                + joint("s", "b", "c")
            ),
            ("first", "s"),
        ),
        (urdf(joint("first", "c", "b") + joint("second", "b", "c")), ("first", "second")),  # a loop
    ],
)
def test_unsupported_or_inconsistent_models_are_rejected(document, joints):
    with pytest.raises(ValueError):
        UrdfKinematics(document, base="a", tip="c", joints=joints)


def test_the_gate_drives_the_real_model_to_a_cartesian_target():
    """A stand-in mapping, NOT a verified one: this exercises the software path only."""
    model = so101()
    unverified = UrdfMapping((1, 1, 1, 1, 1), (0, 0, 0, 0, 0), CAL)
    wide = limits(
        joint_min=model.lower,
        joint_max=model.upper,
        workspace_min=(-1, -1, -1),
        workspace_max=(1, 1, 1),
    )
    gate = Controller(
        kinematics=model, mapping=unverified, limits=wide, xr_to_base=IDENTITY, feedback_timeout=0.1
    )
    rest = So101State((0.2, -0.3, 0.5, 0.4, 0.1), 0.5)
    start = model.fk(rest.joints_rad)
    gate.arm(feedback(1.0, rest), CAL, 1.0)
    gate.tick(sample(0, 1.0, grip=False), 1, feedback(1.0, rest), 1.0)
    step = gate.tick(sample(1, 1.01), 1, feedback(1.01, rest), 1.01)
    assert (step.state, step.command) == (ENGAGED, rest)
    for i in range(2, 60):
        at = 1.0 + i * TICK
        step = gate.tick(sample(i, at, position=(0.02, -0.01, 0.015)), 1, feedback(at, rest), at)
    assert step.reason == "tracking"
    reached = model.fk(step.command.joints_rad)
    wanted = tuple(s + d for s, d in zip(start.position, (0.02, -0.01, 0.015), strict=True))
    assert distance(reached.position, wanted) < 0.0015
    assert (
        max(abs(a - b) for a, b in zip(step.command.joints_rad, rest.joints_rad, strict=True)) < 0.3
    )
