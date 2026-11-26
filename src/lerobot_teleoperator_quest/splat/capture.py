"""Taking a recording: images and joint readings side by side.

Needs numpy and OpenCV (the `splat` extra). It only ever calls
`get_observation()`: nothing here sends a command to the arm. The robot is
whatever the caller connected, typically the official follower with its wrist
camera configured; this module never opens one itself.
"""

import json
from pathlib import Path
from time import monotonic, sleep

import cv2
import numpy as np

from ..so101 import FOLLOWER_KEYS, from_follower_observation
from .poses import SHOTS_FILE


def _shot(robot, folder: Path, camera: str, index: int, time: float, log) -> None:
    observation = robot.get_observation()
    if camera not in observation:
        cameras = sorted(k for k, v in observation.items() if isinstance(v, np.ndarray))
        raise ValueError(f"the robot has no camera named {camera!r}; it has {cameras or 'none'}")
    image = np.asarray(observation[camera])
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError(f"camera {camera!r} did not give an 8-bit colour image")
    from_follower_observation(observation)  # refuse a reading that could not be used later
    name = f"images/{index:06d}.png"
    # LeRobot hands out RGB; OpenCV writes BGR.
    if not cv2.imwrite(str(folder / name), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
        raise OSError(f"could not write {folder / name}")
    follower = {key: float(observation[key]) for key in FOLLOWER_KEYS}
    log.write(json.dumps({"time": time, "image": name, "follower": follower}) + "\n")
    log.flush()  # whatever was taken survives an interruption


def _start(folder) -> Path:
    folder = Path(folder)
    if folder.exists() and (not folder.is_dir() or any(folder.iterdir())):
        raise ValueError(f"{folder} already exists and is not empty; choose a new folder")
    (folder / "images").mkdir(parents=True)
    return folder


def record_stills(robot, folder, camera: str, shots: int, *, wait=input, clock=monotonic) -> int:
    """Take `shots` pictures, waiting for the operator before each.

    For calibration: move the arm, let it settle, then confirm. `wait` is
    called with a prompt and may raise EOFError or KeyboardInterrupt to finish
    early. Returns how many were taken.
    """
    if type(shots) is not int or shots < 1:
        raise ValueError("shots must be a positive whole number")
    folder = _start(folder)
    taken = 0
    with (folder / SHOTS_FILE).open("w", encoding="utf-8") as log:
        for index in range(shots):
            try:
                wait(f"Shot {index + 1} of {shots}: hold the arm still, then press Enter. ")
            except (EOFError, KeyboardInterrupt):
                break
            _shot(robot, folder, camera, index, clock(), log)
            taken += 1
    return taken


def record_scan(
    robot, folder, camera: str, seconds: float, *, rate: float = 10.0, clock=monotonic, pause=sleep
) -> int:
    """Take pictures at `rate` a second for `seconds` while the arm is moved slowly.

    Returns how many were taken. A KeyboardInterrupt ends the scan early and
    keeps what was taken.
    """
    if not seconds > 0 or not rate > 0:
        raise ValueError("seconds and rate must be positive")
    folder = _start(folder)
    taken = 0
    began = clock()
    with (folder / SHOTS_FILE).open("w", encoding="utf-8") as log:
        try:
            while True:
                due = began + taken / rate
                now = clock()
                if due - began >= seconds or now - began >= seconds:
                    break
                if now < due:
                    pause(due - now)
                _shot(robot, folder, camera, taken, clock(), log)
                taken += 1
        except KeyboardInterrupt:
            pass
    return taken
