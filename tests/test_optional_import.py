"""Core contracts must work even when only part of the integration is installed."""

import subprocess
import sys
from pathlib import Path


def test_core_import_with_lerobot_but_without_websockets():
    source = Path(__file__).resolve().parents[1] / "src"
    probe = """
import importlib.util
from unittest.mock import patch
original = importlib.util.find_spec
def available(name, *args, **kwargs):
    return None if name == 'websockets' else original(name, *args, **kwargs)
with patch.object(importlib.util, 'find_spec', side_effect=available):
    import lerobot_teleoperator_quest as package
    from lerobot_teleoperator_quest.contracts import So101State
    assert So101State((0,)*5, 0).gripper == 0
    assert not hasattr(package, 'Quest')
"""
    result = subprocess.run(
        [sys.executable, "-c", probe],
        cwd=source,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
