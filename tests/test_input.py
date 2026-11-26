import math

import pytest

from lerobot_teleoperator_quest.input import (
    InputBuffer,
    action_sample,
    frame_action,
    input_action,
)


def pose(seq=0, grip=False, **changes):
    return (
        dict(
            type="pose",
            version=1,
            seq=seq,
            position=[1, 2, 3],
            orientation=[0, 0, 0, 1],
            grip=grip,
            gripper=0.2,
            scale=0.25,
        )
        | changes
    )


def buffer():
    now = [10.0]
    return InputBuffer(clock=lambda: now[0]), now


def test_disarmed_until_release_then_press_and_host_time_is_used():
    buf, now = buffer()
    session = buf.open_session()
    buf.submit(session, pose(grip=True))
    assert not buf.latest().grip
    buf.submit(session, pose(1))
    buf.submit(session, pose(2, True))
    assert buf.latest().grip
    assert buf.latest().received_at == now[0]
    assert input_action(buf.latest(), session)["quest.active"] is True


def test_timeout_disconnect_reconnect_and_old_session_fencing():
    buf, now = buffer()
    first = buf.open_session()
    buf.submit(first, pose())
    now[0] += 0.201
    assert buf.latest() is None
    assert input_action(None, first)["quest.grip"] is False
    buf.close_session(first)
    second = buf.open_session()
    assert second != first
    with pytest.raises(ValueError, match="session"):
        buf.submit(first, pose(1))
    buf.submit(second, pose(0, True))
    assert not buf.latest().grip
    buf.close_session(first)
    assert buf.latest() is not None  # stale cleanup cannot close new owner
    buf.close_session(second)
    assert buf.latest() is None


def test_stale_stream_requires_a_new_release_after_recovery():
    buf, now = buffer()
    session = buf.open_session()
    buf.submit(session, pose())
    buf.submit(session, pose(1, True))
    now[0] += 0.3
    # No intervening latest() required to notice the stale gap.
    buf.submit(session, pose(2, True))
    assert not buf.latest().grip
    buf.submit(session, pose(3))
    buf.submit(session, pose(4, True))
    assert buf.latest().grip


def test_single_owner_sequence_order_and_monotonic_clock():
    buf, now = buffer()
    session = buf.open_session()
    with pytest.raises(RuntimeError):
        buf.open_session()
    buf.submit(session, pose(4))
    for seq in (4, 3):
        with pytest.raises(ValueError, match="sequence"):
            buf.submit(session, pose(seq))
    now[0] = 9
    assert buf.latest() is None
    with pytest.raises(ValueError, match="clock"):
        buf.submit(session, pose(5))


@pytest.mark.parametrize(
    "patch",
    [
        {"version": 2},
        {"version": True},
        {"seq": True},
        {"seq": -1},
        {"seq": 1.5},
        {"seq": 2**53},
        {"position": [0, 0]},
        {"position": [math.nan, 0, 0]},
        {"orientation": [0, 0, 0, 0]},
        {"orientation": [0, 0, 0, 2]},
        {"grip": 1},
        {"gripper": 2},
        {"scale": -1},
        {"scale": True},
        {"t": 999999},
        {"type": "target"},
    ],
)
def test_reject_invalid_samples(patch):
    buf, _ = buffer()
    session = buf.open_session()
    with pytest.raises(ValueError):
        buf.submit(session, pose(**patch))
    assert buf.latest() is None


def test_payload_is_owned_and_missing_keys_rejected():
    buf, _ = buffer()
    session = buf.open_session()
    payload = pose()
    buf.submit(session, payload)
    payload["position"][0] = 100
    assert buf.latest().position == (1, 2, 3)
    payload = pose(1)
    payload.pop("gripper")
    with pytest.raises(ValueError):
        buf.submit(session, payload)


@pytest.mark.parametrize("timeout", [0, -1, math.inf, math.nan, True])
def test_timeout_must_be_finite_positive(timeout):
    with pytest.raises(ValueError):
        InputBuffer(timeout=timeout)


def frame(seq=0, left=None, right=None):
    return {"type": "pose", "version": 2, "seq": seq, "left": left, "right": right}


def hand(grip=False, **changes):
    return (
        dict(position=[1, 2, 3], orientation=[0, 0, 0, 1], grip=grip, gripper=0.2, scale=0.25)
        | changes
    )


def bimanual():
    now = [10.0]
    return InputBuffer(clock=lambda: now[0], bimanual=True), now


def test_each_hand_has_its_own_release_gate():
    buf, _ = bimanual()
    session = buf.open_session()
    buf.submit(session, frame(0, hand(), hand(True)))
    buf.submit(session, frame(1, hand(True), hand(True)))
    hands, owner = buf.frame()
    assert owner == session
    assert hands["left"].grip and not hands["right"].grip
    buf.submit(session, frame(2, hand(True), hand()))
    buf.submit(session, frame(3, hand(True), hand(True)))
    hands = buf.frame()[0]
    assert hands["left"].grip and hands["right"].grip
    assert hands["left"].sequence == hands["right"].sequence == 3


def test_a_hand_that_loses_tracking_must_release_again():
    buf, _ = bimanual()
    session = buf.open_session()
    buf.submit(session, frame(0, hand(), hand()))
    buf.submit(session, frame(1, hand(True), hand(True)))
    buf.submit(session, frame(2, None, hand(True)))
    hands = buf.frame()[0]
    assert hands["left"] is None and hands["right"].grip
    buf.submit(session, frame(3, hand(True), hand(True)))
    hands = buf.frame()[0]
    assert not hands["left"].grip and hands["right"].grip
    buf.submit(session, frame(4))  # neither hand tracked is still a valid, ordered frame
    assert buf.frame()[0] == {"left": None, "right": None}
    with pytest.raises(ValueError, match="sequence"):
        buf.submit(session, frame(4))


def test_both_hands_go_stale_together_and_must_release_again():
    buf, now = bimanual()
    session = buf.open_session()
    buf.submit(session, frame(0, hand(), hand()))
    buf.submit(session, frame(1, hand(True), hand(True)))
    now[0] += 0.201
    assert buf.frame() == ({"left": None, "right": None}, session)
    buf.submit(session, frame(2, hand(True), hand(True)))
    hands = buf.frame()[0]
    assert not hands["left"].grip and not hands["right"].grip


def test_modes_do_not_accept_each_others_messages_or_accessors():
    buf, _ = bimanual()
    session = buf.open_session()
    with pytest.raises(ValueError):
        buf.submit(session, pose())
    with pytest.raises(RuntimeError, match="frame"):
        buf.snapshot()
    with pytest.raises(RuntimeError, match="frame"):
        buf.latest()
    single, _ = buffer()
    with pytest.raises(ValueError):
        single.submit(single.open_session(), frame(0, hand(), hand()))
    with pytest.raises(ValueError):
        InputBuffer(bimanual=1)


@pytest.mark.parametrize(
    "payload",
    [
        frame() | {"version": 1},
        frame() | {"version": 3},
        frame() | {"extra": 1},
        {k: v for k, v in frame().items() if k != "right"},
        frame(-1),
        frame(True),
        frame(0, left=[]),
        frame(0, left=hand() | {"t": 1}),
        frame(0, left={k: v for k, v in hand().items() if k != "scale"}),
        frame(0, left=hand(grip=1)),
        frame(0, right=hand(orientation=[0, 0, 0, 2])),
        frame(0, right=hand(position=[0, math.inf, 0])),
        frame(0, hand(), hand(gripper=-0.1)),
        [],
    ],
)
def test_bimanual_rejects_malformed_frames(payload):
    buf, _ = bimanual()
    session = buf.open_session()
    with pytest.raises(ValueError):
        buf.submit(session, payload)
    assert buf.frame()[0] == {"left": None, "right": None}


def test_exported_features_rebuild_the_same_sample_for_one_hand():
    buf, _ = buffer()
    session = buf.open_session()
    buf.submit(session, pose())
    buf.submit(session, pose(1, True))
    action = frame_action(*buf.frame())
    assert action == input_action(buf.latest(), session)
    assert action_sample(action) == (buf.latest(), session)
    buf.close_session(session)
    assert action_sample(frame_action(*buf.frame())) == (None, 0)


def test_exported_features_rebuild_each_hand_separately():
    buf, _ = bimanual()
    session = buf.open_session()
    buf.submit(session, frame(0, hand(position=[4, 5, 6]), None))
    hands, _ = buf.frame()
    action = frame_action(hands, session)
    assert all(key.startswith(("quest.left.", "quest.right.")) for key in action)
    assert not any(key.endswith(".pos") for key in action)
    assert action_sample(action, "quest.left.") == (hands["left"], session)
    assert action_sample(action, "quest.right.") == (None, session)
    with pytest.raises(ValueError, match=r"quest\.\* input features"):
        action_sample(action)


@pytest.mark.parametrize(
    "patch",
    [
        {"quest.session": True},
        {"quest.session": -1},
        {"quest.active": 1},
        {"quest.seq": 1.5},
        {"quest.x": math.nan},
        {"quest.qw": 3.0},
        {"quest.grip": 1},
        {"quest.gripper": 2},
        {"quest.received_at": -1},
        {"quest.scale": None},
    ],
)
def test_malformed_exported_features_are_rejected(patch):
    buf, _ = buffer()
    session = buf.open_session()
    buf.submit(session, pose())
    with pytest.raises(ValueError):
        action_sample(frame_action(*buf.frame()) | patch)
    with pytest.raises(ValueError):
        action_sample(None)
