import json
import math

import pytest

from lerobot_teleoperator_quest.contracts import So101State, Telemetry
from lerobot_teleoperator_quest.so101 import (
    FOLLOWER_KEYS,
    UrdfMapping,
    calibration_fingerprint,
    from_follower_action,
    from_follower_observation,
    to_follower_action,
)


def state(q=0.0, gripper=0.0):
    return So101State((q,) * 5, gripper)


def test_feedback_is_measured_not_requested_even_with_jammed_gripper():
    measured, requested, applied = state(0.1, 0.2), state(0.3, 0.9), state(0.25, 0.8)
    data = json.loads(json.dumps(Telemetry(measured, requested, applied, 12.5).to_dict()))
    assert data["measured"]["joints_rad"] == [0.1] * 5
    assert data["requested"]["joints_rad"] == [0.3] * 5
    assert data["applied"]["joints_rad"] == [0.25] * 5
    assert data["measured"]["gripper"] == 0.2
    assert data["requested"]["gripper"] == 0.9
    assert data["received_at"] == 12.5
    assert Telemetry(measured, None, None, 0).to_dict()["applied"] is None


def test_values_are_owned_and_serialization_is_a_copy():
    original = [0.0] * 5
    s = So101State(original, 0)
    original[0] = 3
    assert s.joints_rad == (0.0,) * 5
    data = Telemetry(s, None, None, 1).to_dict()
    data["measured"]["joints_rad"][0] = 4
    assert s.joints_rad[0] == 0
    with pytest.raises(AttributeError):
        s.gripper = 1


@pytest.mark.parametrize(
    "joints,g",
    [
        ([0] * 4, 0),
        ([0] * 6, 0),
        ([math.nan] * 5, 0),
        ([math.inf] * 5, 0),
        ([True] * 5, 0),
        ([0] * 5, True),
        ([0] * 5, -0.01),
        ([0] * 5, 1.01),
        ([0] * 5, math.nan),
        ([10**400] * 5, 0),
    ],
)
def test_reject_bad_state(joints, g):
    with pytest.raises(ValueError):
        So101State(joints, g)


@pytest.mark.parametrize("timestamp", [-1, math.nan, math.inf, True])
def test_reject_bad_feedback_time(timestamp):
    with pytest.raises(ValueError):
        Telemetry(state(), None, None, timestamp)


def test_official_degree_and_gripper_contract():
    s = So101State((0, math.pi / 2, -math.pi / 2, math.pi, -math.pi), 0.75)
    action = to_follower_action(s)
    assert tuple(action) == FOLLOWER_KEYS
    assert list(action.values()) == pytest.approx([0, 90, -90, 180, -180, 75])
    assert from_follower_action(action).joints_rad == pytest.approx(s.joints_rad)
    assert from_follower_action(action).gripper == 0.75


def test_observations_allow_camera_fields_but_actions_are_strict():
    obs = to_follower_action(state()) | {"front": object()}
    assert from_follower_observation(obs) == state()
    with pytest.raises(ValueError):
        from_follower_action(obs)
    obs.pop("shoulder_pan.pos")
    with pytest.raises(ValueError):
        from_follower_observation(obs)


@pytest.mark.parametrize("value", [math.nan, math.inf, True, "10", None])
def test_reject_invalid_follower_value(value):
    action = to_follower_action(state())
    action["elbow_flex.pos"] = value
    with pytest.raises(ValueError):
        from_follower_action(action)


def test_urdf_mapping_explicit_and_bound_to_calibration():
    fp = calibration_fingerprint({"wrist": {"homing_offset": 123, "id": 5}})
    assert fp == calibration_fingerprint({"wrist": {"id": 5, "homing_offset": 123}})
    mapping = UrdfMapping((1, -1, 1, -1, 1), (10, 20, 0, 0, 0), fp)
    q = mapping.to_urdf(state(), fp)
    assert q == pytest.approx(tuple(math.radians(x) for x in [-10, 20, 0, 0, 0]))
    assert mapping.from_urdf(q, 0.3, fp).joints_rad == pytest.approx((0,) * 5)
    with pytest.raises(ValueError, match="calibration"):
        mapping.to_urdf(state(), "f" * 64)


@pytest.mark.parametrize(
    "signs,zeros,fp",
    [
        ((1,) * 4, (0,) * 5, "a" * 64),
        ((0,) * 5, (0,) * 5, "a" * 64),
        ((True,) * 5, (0,) * 5, "a" * 64),
        ((1,) * 5, (math.nan,) * 5, "a" * 64),
        ((1,) * 5, (0,) * 5, ""),
    ],
)
def test_refuse_unverified_mapping(signs, zeros, fp):
    with pytest.raises(ValueError):
        UrdfMapping(signs, zeros, fp)


def test_conversion_must_not_manufacture_infinite_commands():
    with pytest.raises(ValueError, match="finite"):
        to_follower_action(state(1e308))
    mapping = UrdfMapping((1,) * 5, (-1e308,) * 5, "a" * 64)
    with pytest.raises(ValueError, match="finite"):
        mapping.to_urdf(state(1.79e308), "a" * 64)
