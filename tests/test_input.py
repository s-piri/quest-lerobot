import math

import pytest

from lerobot_teleoperator_quest.input import InputBuffer, input_action


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
