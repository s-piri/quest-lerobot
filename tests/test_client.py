"""The browser client's message builders, run in Node against the real listener."""

import importlib.util
import json
import shutil
import subprocess
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.integration
if any(importlib.util.find_spec(name) is None for name in ("lerobot", "websockets")):
    pytest.skip("install .[lerobot] for the real integration gate", allow_module_level=True)

from lerobot.teleoperators import make_teleoperator_from_config

from lerobot_teleoperator_quest import QuestConfig

CLIENT = Path(__file__).resolve().parents[1] / "client"
NODE = shutil.which("node")


def node_major() -> int:
    if NODE is None:
        return 0
    version = subprocess.run([NODE, "--version"], capture_output=True, text=True).stdout
    return int(version.strip().lstrip("v").split(".")[0])


needs_node = pytest.mark.skipif(
    node_major() < 22, reason="the client checks need Node 22 or newer for its built-in WebSocket"
)


@needs_node
def test_message_builders_pass_their_own_tests():
    result = subprocess.run(
        [
            NODE,
            "--test",
            *(str(CLIENT / f"{name}.test.mjs") for name in ("protocol", "arm", "guide")),
        ],
        capture_output=True,
        encoding="utf-8",
        errors="replace",
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@needs_node
@pytest.mark.parametrize("mode", ["left", "right", "both"])
def test_client_messages_drive_the_real_listener(tmp_path, mode):
    # A long input timeout: this checks the wire format, not the watchdog.
    config = QuestConfig(port=0, calibration_dir=tmp_path, bimanual=mode == "both", timeout_s=30)
    quest = make_teleoperator_from_config(config)
    quest.connect()
    process = subprocess.Popen(
        [NODE, str(CLIENT / "smoke.mjs"), quest.url, quest.access_token, mode],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        encoding="utf-8",
        errors="replace",
    )
    try:
        line = process.stdout.readline()  # the script exits by itself after 15 s
        assert line, process.stderr.read()
        summary = json.loads(line)
        assert summary["sent"] == 3
        assert summary["version"] == (2 if mode == "both" else 1)

        action = quest.get_action()
        prefixes = ["quest.left.", "quest.right."] if mode == "both" else ["quest."]
        for prefix in prefixes:
            assert action[prefix + "active"] is True
            assert action[prefix + "session"] == summary["session"]
            assert action[prefix + "seq"] == 2
            assert action[prefix + "grip"] is True  # released first, then pressed
            assert action[prefix + "x"] == pytest.approx(0.3)
            assert action[prefix + "gripper"] == pytest.approx(0.75)  # trigger a quarter pulled
            assert action[prefix + "scale"] == pytest.approx(0.5)
        process.stdin.write("\n")
        process.stdin.flush()
        assert process.wait(timeout=20) == 0, process.stderr.read()

        # The client left: its input must be invalidated, not left active for 30 s.
        deadline = time.monotonic() + 5
        while any(quest.get_action()[prefix + "active"] for prefix in prefixes):
            assert time.monotonic() < deadline, "input stayed active after the client closed"
            time.sleep(0.02)
    finally:
        process.kill()
        process.wait(timeout=20)
        quest.disconnect()
