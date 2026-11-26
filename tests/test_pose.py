from math import cos, pi, sin

import pytest

from lerobot_teleoperator_quest.control import Anchor
from lerobot_teleoperator_quest.pose import (
    IDENTITY,
    Pose,
    conjugate,
    multiply,
    rotate,
    scale_rotation,
)


def about(axis, angle):
    return (*(a * sin(angle / 2) for a in axis), cos(angle / 2))


Z90 = about((0, 0, 1), pi / 2)


def test_rotation_follows_the_right_hand_rule():
    assert rotate(Z90, (1, 0, 0)) == pytest.approx((0, 1, 0))
    assert rotate(conjugate(Z90), (0, 1, 0)) == pytest.approx((1, 0, 0))
    assert multiply(Z90, conjugate(Z90)) == pytest.approx(IDENTITY)


def test_product_applies_the_right_operand_first():
    x90 = about((1, 0, 0), pi / 2)
    assert rotate(multiply(Z90, x90), (0, 1, 0)) == pytest.approx((0, 0, 1))
    assert rotate(multiply(x90, Z90), (0, 1, 0)) == pytest.approx((-1, 0, 0))


def test_scaling_a_rotation_keeps_its_axis_and_takes_the_short_way():
    assert scale_rotation(Z90, 0.5) == pytest.approx(about((0, 0, 1), pi / 4))
    assert scale_rotation(Z90, 0) == pytest.approx(IDENTITY)
    assert scale_rotation(IDENTITY, 3) == IDENTITY
    long_way = tuple(-c for c in about((0, 0, 1), pi / 2))
    assert scale_rotation(long_way, 0.5) == pytest.approx(about((0, 0, 1), pi / 4))


@pytest.mark.parametrize(
    "position, orientation",
    [
        ((0, 0), IDENTITY),
        ((0, 0, float("nan")), IDENTITY),
        ((0, 0, 0), (0, 0, 0, 0)),
        ((0, 0, 0), (0, 0, 0, 2)),
        ((0, 0, 0), (0, 0, 1)),
        ((0, 0, True), IDENTITY),
    ],
)
def test_pose_rejects_malformed_values(position, orientation):
    with pytest.raises(ValueError):
        Pose(position, orientation)


def test_pose_is_normalized_and_immutable():
    pose = Pose([1, 2, 3], (0, 0, 0, 1.0005))
    assert pose.position == (1.0, 2.0, 3.0)
    assert pose.orientation == pytest.approx(IDENTITY)
    with pytest.raises(AttributeError):
        pose.position = (0, 0, 0)


def test_anchor_maps_controller_motion_into_the_base_frame():
    anchor = Anchor(Pose((1, 1, 1), IDENTITY), Pose((0.2, 0, 0.1), IDENTITY), 0.5, 1.0)
    moved = anchor.target(Pose((1.1, 1, 1), IDENTITY), Z90)
    assert moved.position == pytest.approx((0.2, 0.05, 0.1))
    assert moved.orientation == pytest.approx(IDENTITY)


def test_anchor_rotation_is_relative_to_the_grip_and_expressed_in_the_base_frame():
    grip = about((0, 1, 0), 0.3)
    anchor = Anchor(Pose((0, 0, 0), grip), Pose((0, 0, 0), Z90), 1.0, 1.0)
    assert anchor.target(Pose((0, 0, 0), grip), Z90).orientation == pytest.approx(Z90)

    # A turn about the controller frame's x axis is a turn about base y here.
    turned = multiply(about((1, 0, 0), 0.4), grip)
    expected = multiply(about((0, 1, 0), 0.4), Z90)
    assert anchor.target(Pose((0, 0, 0), turned), Z90).orientation == pytest.approx(expected)

    halved = Anchor(anchor.controller, anchor.tool, 1.0, 0.5)
    expected = multiply(about((0, 1, 0), 0.2), Z90)
    assert halved.target(Pose((0, 0, 0), turned), Z90).orientation == pytest.approx(expected)
