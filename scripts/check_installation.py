"""Install the built wheel outside the checkout and check its public boundary."""

import argparse
import os
import subprocess
import tempfile
import venv
from pathlib import Path

CORE_PROBE = """
import importlib.util
import json
import sys
from pathlib import Path
import lerobot_teleoperator_quest as package
from lerobot_teleoperator_quest.contracts import So101State, Telemetry
assert Path(package.__file__).resolve().is_relative_to(Path(sys.prefix).resolve())
assert importlib.util.find_spec('lerobot') is None
assert 'torch' not in sys.modules
sample = Telemetry(So101State((0,)*5, 0), So101State((1,)*5, 1), None, 1)
data = json.loads(json.dumps(sample.to_dict()))
assert data['measured'] != data['requested']
from lerobot_teleoperator_quest.kinematics import so101
assert len(so101().fk((0,) * 5).position) == 3, 'the packaged SO-101 model did not load'
print('PASS: installed core wheel, no LeRobot/Torch, packaged model, distinct measurements')
"""

INTEGRATION_PROBE = """
import sys
from pathlib import Path
from importlib.metadata import distribution, version
assert version('lerobot') == '0.7.1'
assert 'lerobot_teleoperator_quest' not in sys.modules
from lerobot.utils.import_utils import register_third_party_plugins
register_third_party_plugins()
assert 'lerobot_teleoperator_quest' in sys.modules, 'official discovery missed the wheel'
import lerobot_teleoperator_quest as package
from lerobot.teleoperators import TeleoperatorConfig, make_teleoperator_from_config
assert distribution('lerobot_teleoperator_quest').metadata['Name'] == 'lerobot_teleoperator_quest'
assert Path(package.__file__).resolve().is_relative_to(Path(sys.prefix).resolve())
assert TeleoperatorConfig.get_choice_class('quest') is package.QuestConfig
config = package.QuestConfig(port=0, calibration_dir=Path.cwd() / 'input-calibration')
device = make_teleoperator_from_config(config)
assert isinstance(device, package.Quest)
assert not device.is_connected
assert not (set(device.action_features) & {'shoulder_pan.pos', 'gripper.pos'})
device.disconnect()
print('PASS: installed wheel discovered and instantiated by actual official LeRobot')
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wheel", required=True, type=Path)
    parser.add_argument("--integration", action="store_true")
    args = parser.parse_args()
    wheel = args.wheel.resolve(strict=True)
    checks = Path(__file__).resolve().parent.parent / ".checks"
    checks.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="wheel-", dir=checks) as directory:
        root = Path(directory)
        venv.EnvBuilder(with_pip=True, system_site_packages=args.integration).create(root / "env")
        binary = root / "env" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
        subprocess.run(
            [
                str(binary),
                "-m",
                "pip",
                "install",
                "--no-deps",
                "--ignore-installed",
                "--no-cache-dir",
                "--disable-pip-version-check",
                "--quiet",
                str(wheel),
            ],
            check=True,
            cwd=root,
        )
        subprocess.run(
            [str(binary), "-I", "-c", INTEGRATION_PROBE if args.integration else CORE_PROBE],
            check=True,
            cwd=root,
        )


if __name__ == "__main__":
    main()
