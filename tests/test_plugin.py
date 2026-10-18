import importlib.util
import json

import pytest

pytestmark = pytest.mark.integration
if any(importlib.util.find_spec(name) is None for name in ("lerobot", "websockets")):
    pytest.skip("install .[lerobot] for the real integration gate", allow_module_level=True)

from lerobot.teleoperators import make_teleoperator_from_config
from test_input import pose
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
