import importlib.util
import json

import pytest

pytestmark = pytest.mark.integration
if any(importlib.util.find_spec(name) is None for name in ("lerobot", "websockets")):
    pytest.skip("install .[lerobot] for the real integration gate", allow_module_level=True)

from lerobot.teleoperators import make_teleoperator_from_config
from test_input import frame, hand, pose
from websockets.exceptions import ConnectionClosed
from websockets.sync.client import connect

from lerobot_teleoperator_quest import Quest, QuestConfig
from lerobot_teleoperator_quest.so101 import FOLLOWER_KEYS


@pytest.fixture
def quest(tmp_path):
    device = make_teleoperator_from_config(QuestConfig(port=0, calibration_dir=tmp_path))
    yield device
    device.disconnect()


def auth(ws, token):
    ws.send(json.dumps({"type": "auth", "token": token}))
    return json.loads(ws.recv(timeout=2))


def test_real_factory_concrete_plugin_and_authentication(quest):
    assert isinstance(quest, Quest)
    assert quest.config.type == "quest"
    assert quest.is_calibrated  # input device, not the arm's calibration
    with pytest.raises(RuntimeError):
        quest.get_action()
    quest.connect()
    assert quest.is_connected
    with pytest.raises(RuntimeError):
        quest.connect()
    with connect(quest.url) as ws:
        assert auth(ws, quest.access_token)["type"] == "ready"
        ws.send(json.dumps(pose()))
        assert json.loads(ws.recv(timeout=2))["seq"] == 0
        ws.send(json.dumps(pose(1, True)))
        assert json.loads(ws.recv(timeout=2))["seq"] == 1
        action = quest.get_action()
        assert action["quest.grip"] is True
        assert set(action) == set(quest.action_features)
        assert not set(FOLLOWER_KEYS) & set(action)
        quest.send_feedback(dict.fromkeys(FOLLOWER_KEYS, 0.0))
        assert quest.feedback["measured"]["gripper"] == 0
        assert quest.feedback["requested"] is None
        assert quest.feedback["applied"] is None
    quest.disconnect()
    assert not quest.is_connected
    quest.disconnect()  # idempotent


def test_bad_auth_and_second_owner_are_rejected(quest):
    quest.connect()
    with connect(quest.url) as bad:
        bad.send(json.dumps({"type": "auth", "token": "wrong"}))
        with pytest.raises(ConnectionClosed):
            bad.recv(timeout=2)
    with connect(quest.url) as first, connect(quest.url) as second:
        assert auth(first, quest.access_token)["type"] == "ready"
        second.send(json.dumps({"type": "auth", "token": quest.access_token}))
        with pytest.raises(ConnectionClosed):
            second.recv(timeout=2)
        first.send(json.dumps(pose()))
        first.recv(timeout=2)
        assert quest.get_action()["quest.active"]


def test_malformed_input_closes_and_invalidates_state(quest):
    quest.connect()
    with connect(quest.url) as ws:
        auth(ws, quest.access_token)
        ws.send(json.dumps(pose()))
        ws.recv(timeout=2)
        ws.send(json.dumps(pose(0)))  # duplicate sequence
        with pytest.raises(ConnectionClosed):
            ws.recv(timeout=2)
    assert not quest.get_action()["quest.active"]


def test_shutdown_closes_an_active_socket_and_can_restart(quest):
    quest.connect()
    token = quest.access_token
    with connect(quest.url) as ws:
        auth(ws, token)
        quest.disconnect()
        with pytest.raises(ConnectionClosed):
            ws.recv(timeout=2)
    quest.connect()
    assert quest.access_token != token
    assert not quest.get_action()["quest.active"]


def test_shutdown_closes_unauthenticated_connections_and_clears_feedback(quest):
    quest.connect()
    quest.send_feedback(dict.fromkeys(FOLLOWER_KEYS, 0.0))
    with connect(quest.url) as ws:
        quest.disconnect()
        with pytest.raises(ConnectionClosed):
            ws.recv(timeout=2)
    quest.connect()
    assert quest.feedback is None


def test_remote_listener_requires_tls():
    with pytest.raises(ValueError, match="TLS"):
        QuestConfig(host="0.0.0.0")


def test_credential_is_not_serialized_in_config(quest):
    from dataclasses import asdict

    assert "token" not in asdict(quest.config)
    assert quest.access_token not in repr(quest.config)


@pytest.mark.parametrize(
    "options",
    [
        {"port": -1},
        {"port": True},
        {"timeout_s": 0},
        {"timeout_s": float("nan")},
        {"host": ""},
        {"origins": []},
        {"origins": [1]},
        {"cert": "cert.pem"},
    ],
)
def test_invalid_config_is_rejected(options):
    with pytest.raises(ValueError):
        QuestConfig(**options)


def test_unlisted_browser_origin_is_rejected(quest):
    from websockets.exceptions import InvalidStatus

    quest.connect()
    with pytest.raises(InvalidStatus):
        with connect(quest.url, origin="https://unlisted.example"):
            pass


def test_shutdown_closes_a_socket_before_websocket_handshake(quest):
    import socket
    import time
    from urllib.parse import urlparse

    quest.connect()
    url = urlparse(quest.url)
    with socket.create_connection((url.hostname, url.port), timeout=2) as raw:
        # Give the accept loop time to hand off the socket without sending a
        # handshake. The assertion tests EOF, not an internal socket counter.
        time.sleep(0.05)
        quest.disconnect()
        raw.settimeout(0.5)
        try:
            assert raw.recv(1) == b""
        except ConnectionResetError:
            pass


@pytest.fixture
def pair(tmp_path):
    config = QuestConfig(port=0, calibration_dir=tmp_path, bimanual=True)
    device = make_teleoperator_from_config(config)
    yield device
    device.disconnect()


def test_bimanual_plugin_speaks_version_two_and_exports_both_hands(pair):
    assert all(key.startswith(("quest.left.", "quest.right.")) for key in pair.action_features)
    assert set(pair.feedback_features) == {
        f"{arm}_{key}" for arm in ("left", "right") for key in FOLLOWER_KEYS
    }
    pair.connect()
    with connect(pair.url) as ws:
        ready = auth(ws, pair.access_token)
        assert (ready["type"], ready["version"]) == ("ready", 2)
        ws.send(json.dumps(frame(0, hand(), None)))
        assert json.loads(ws.recv(timeout=2)) == {"type": "accepted", "seq": 0}
        ws.send(json.dumps(frame(1, hand(True), hand(True))))
        ws.recv(timeout=2)
        action = pair.get_action()
        assert set(action) == set(pair.action_features)
        assert all(type(action[key]) is kind for key, kind in pair.action_features.items())
        assert action["quest.left.grip"] is True
        assert action["quest.right.active"] is True
        assert action["quest.right.grip"] is False  # it appeared already pressed
        assert not any(key.endswith(".pos") for key in action)

        observation = {key: 0.0 for key in pair.feedback_features} | {"left_gripper.pos": 40.0}
        pair.send_feedback(observation)
        assert pair.feedback["left"]["measured"]["gripper"] == pytest.approx(0.4)
        assert pair.feedback["right"]["measured"]["gripper"] == 0
        assert pair.feedback["right"]["requested"] is None
        with pytest.raises(ValueError):
            pair.send_feedback(dict.fromkeys(FOLLOWER_KEYS, 0.0))  # one arm's keys only


def test_bimanual_plugin_rejects_a_single_controller_message(pair):
    pair.connect()
    with connect(pair.url) as ws:
        auth(ws, pair.access_token)
        ws.send(json.dumps(frame(0, hand(), hand())))
        ws.recv(timeout=2)
        ws.send(json.dumps(pose(1)))
        with pytest.raises(ConnectionClosed):
            ws.recv(timeout=2)
    action = pair.get_action()
    assert not action["quest.left.active"] and not action["quest.right.active"]


def test_single_plugin_rejects_a_two_hand_message_and_reports_version_one(quest):
    quest.connect()
    with connect(quest.url) as ws:
        assert auth(ws, quest.access_token)["version"] == 1
        ws.send(json.dumps(frame(0, hand(), hand())))
        with pytest.raises(ConnectionClosed):
            ws.recv(timeout=2)
    assert not quest.get_action()["quest.active"]


def test_bimanual_must_be_a_boolean():
    with pytest.raises(ValueError, match="bimanual"):
        QuestConfig(bimanual=1)


def exchange(ws, payload):
    """Send one sample; return the state messages that arrived before its acknowledgment."""
    ws.send(json.dumps(payload))
    states = []
    while True:
        reply = json.loads(ws.recv(timeout=2))
        if reply["type"] == "accepted":
            assert reply["seq"] == payload["seq"]
            return states
        states.append(reply)


def test_state_reaches_the_headset_after_an_acknowledgment_newest_only(quest):
    with pytest.raises(RuntimeError):
        quest.send_state({"type": "state", "arms": {}})
    quest.connect()
    with connect(quest.url) as ws:
        auth(ws, quest.access_token)
        # Queued while the listener waits for a sample: only the newest is kept.
        quest.send_state({"type": "state", "arms": {"": {"state": "armed"}}})
        newest = {"type": "state", "arms": {"": {"state": "engaged", "goal": [0.1] * 6}}}
        quest.send_state(newest)
        assert exchange(ws, pose()) == []  # it follows an acknowledgment, never precedes one
        assert exchange(ws, pose(1)) == [newest]
        assert exchange(ws, pose(2)) == []  # delivered once
        assert exchange(ws, pose(3)) == []


@pytest.mark.parametrize(
    "state",
    [
        None,
        [],
        {"arms": {}},
        {"type": "pose"},
        {"type": "state", "value": float("nan")},
        {"type": "state", "value": float("inf")},
        {"type": "state", "value": object()},
        {"type": "state", "value": "x" * 9000},
    ],
)
def test_malformed_state_is_rejected_before_it_is_queued(quest, state):
    quest.connect()
    with pytest.raises(ValueError):
        quest.send_state(state)
    with connect(quest.url) as ws:
        auth(ws, quest.access_token)
        ws.send(json.dumps(pose()))
        assert json.loads(ws.recv(timeout=2))["type"] == "accepted"
        ws.send(json.dumps(pose(1)))
        assert json.loads(ws.recv(timeout=2)) == {"type": "accepted", "seq": 1}


def test_a_queued_state_does_not_survive_a_restart(quest):
    quest.connect()
    quest.send_state({"type": "state", "arms": {}})
    quest.disconnect()
    quest.connect()
    with connect(quest.url) as ws:
        auth(ws, quest.access_token)
        ws.send(json.dumps(pose()))
        assert json.loads(ws.recv(timeout=2))["type"] == "accepted"
        ws.send(json.dumps(pose(1)))
        assert json.loads(ws.recv(timeout=2)) == {"type": "accepted", "seq": 1}


def test_a_ping_delivers_state_without_counting_as_input(tmp_path):
    config = QuestConfig(port=0, calibration_dir=tmp_path, timeout_s=0.3)
    quest = make_teleoperator_from_config(config)
    quest.connect()
    try:
        with connect(quest.url) as ws:
            auth(ws, quest.access_token)
            assert exchange(ws, pose()) == []
            assert quest.get_action()["quest.active"] is True

            # No sample is flowing, yet the picture still reaches the headset.
            state = {"type": "state", "arms": {"": {"state": "armed"}}}
            quest.send_state(state)
            ws.send(json.dumps({"type": "ping"}))
            assert json.loads(ws.recv(timeout=2)) == state

            # Pings for longer than the input timeout do not keep the input alive.
            import time

            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                ws.send(json.dumps({"type": "ping"}))
                time.sleep(0.05)
            assert quest.get_action()["quest.active"] is False
            assert exchange(ws, pose(1)) == []  # a ping is not acknowledged and uses no sequence

            ws.send(json.dumps({"type": "ping", "seq": 2}))  # not a ping: malformed input
            with pytest.raises(ConnectionClosed):
                ws.recv(timeout=2)
    finally:
        quest.disconnect()


def test_pairing_link_carries_this_runs_listener_and_token(quest, pair):
    with pytest.raises(RuntimeError):
        quest.pairing_link("https://robot.local/client/")
    quest.connect()
    from urllib.parse import parse_qs, urlsplit

    link = quest.pairing_link("https://robot.local/client/")
    parts = urlsplit(link)
    assert (parts.scheme, parts.netloc, parts.path) == ("https", "robot.local", "/client/")
    assert parse_qs(parts.fragment) == {"url": [quest.url], "token": [quest.access_token]}

    named = parse_qs(
        urlsplit(quest.pairing_link("https://robot.local/", host="robot.local")).fragment
    )
    port = quest.url.rsplit(":", 1)[1]
    assert named["url"] == [f"ws://robot.local:{port}"]
    for page in ("", None, "https://robot.local/#old"):
        with pytest.raises(ValueError):
            quest.pairing_link(page)
    with pytest.raises(ValueError):
        quest.pairing_link("https://robot.local/", host="")

    pair.connect()
    assert parse_qs(urlsplit(pair.pairing_link("https://robot.local/")).fragment)["mode"] == [
        "both"
    ]
    # The token is still absent from anything a CLI would log.
    from dataclasses import asdict

    assert quest.access_token not in repr(asdict(quest.config))
