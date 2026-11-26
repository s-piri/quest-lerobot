"""Recording from a stand-in robot. No real robot or camera is involved."""

import json

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

from test_control import REST

from lerobot_teleoperator_quest.so101 import FOLLOWER_KEYS, to_follower_action
from lerobot_teleoperator_quest.splat.capture import record_scan, record_stills
from lerobot_teleoperator_quest.splat.poses import read_recording


class StandInRobot:
    """Only what recording uses: `get_observation`. Anything else would be a bug."""

    def __init__(self, camera="wrist"):
        self.camera = camera
        self.calls = 0

    def get_observation(self):
        self.calls += 1
        picture = np.zeros((48, 64, 3), np.uint8)
        picture[..., 0] = 200  # red in RGB, which LeRobot cameras return
        picture[0, 0] = (0, self.calls, 255)
        reading = to_follower_action(REST) | {"shoulder_pan.pos": float(self.calls)}
        return {**reading, self.camera: picture, "overhead": picture}

    def __getattr__(self, name):
        raise AssertionError(f"recording must not use robot.{name}")


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def pause(self, seconds):
        assert seconds > 0
        self.now += seconds


def test_stills_wait_for_the_operator_and_read_back_as_a_recording(tmp_path):
    robot, clock, prompts = StandInRobot(), Clock(), []

    def wait(prompt):
        prompts.append(prompt)
        clock.now += 2.0  # the operator repositions the arm

    assert record_stills(robot, tmp_path / "board", "wrist", 3, wait=wait, clock=clock) == 3
    assert prompts[0].startswith("Shot 1 of 3") and len(prompts) == 3
    shots = read_recording(tmp_path / "board")
    assert [shot.time for shot in shots] == [102.0, 104.0, 106.0]
    assert [shot.image for shot in shots] == [f"images/{i:06d}.png" for i in range(3)]
    line = json.loads(
        (tmp_path / "board" / "shots.jsonl").read_text(encoding="utf-8").splitlines()[1]
    )
    assert set(line["follower"]) == set(FOLLOWER_KEYS)
    assert line["follower"]["shoulder_pan.pos"] == 2.0  # the reading taken with that picture

    # Colours survive: LeRobot's red is still red when OpenCV reads the file back as BGR.
    saved = cv2.imread(str(tmp_path / "board" / "images" / "000001.png"))
    assert saved.shape == (48, 64, 3)
    assert tuple(saved[10, 10]) == (0, 0, 200)
    assert tuple(saved[0, 0]) == (255, 2, 0)


def test_the_operator_can_stop_stills_early_and_keep_what_was_taken(tmp_path):
    answers = iter([None, None, EOFError()])
    clock = Clock()

    def wait(_prompt):
        clock.now += 1.0  # the operator takes a moment between shots
        answer = next(answers)
        if answer is not None:
            raise answer

    taken = record_stills(StandInRobot(), tmp_path / "board", "wrist", 10, wait=wait, clock=clock)
    assert taken == 2
    assert len(read_recording(tmp_path / "board")) == 2


def test_a_scan_keeps_its_rate_and_stops_on_time(tmp_path):
    robot, clock = StandInRobot(), Clock()
    taken = record_scan(
        robot, tmp_path / "scan", "wrist", 1.0, rate=10, clock=clock, pause=clock.pause
    )
    assert taken == 10 and robot.calls == 10
    shots = read_recording(tmp_path / "scan")
    assert [round(shot.time - 100.0, 6) for shot in shots] == [round(i / 10, 6) for i in range(10)]


def test_interrupting_a_scan_keeps_what_was_taken(tmp_path):
    robot, clock = StandInRobot(), Clock()

    def pause(seconds):
        if robot.calls == 4:
            raise KeyboardInterrupt
        clock.pause(seconds)

    assert (
        record_scan(robot, tmp_path / "scan", "wrist", 5.0, rate=10, clock=clock, pause=pause) == 4
    )
    assert len(read_recording(tmp_path / "scan")) == 4


def test_recording_refuses_what_it_could_not_use_later(tmp_path):
    with pytest.raises(
        ValueError, match="no camera named 'front'; it has \\['overhead', 'wrist'\\]"
    ):
        record_stills(
            StandInRobot(), tmp_path / "a", "front", 1, wait=lambda _: None, clock=Clock()
        )

    class Grey(StandInRobot):
        def get_observation(self):
            return super().get_observation() | {"wrist": np.zeros((48, 64), np.uint8)}

    with pytest.raises(ValueError, match="8-bit colour image"):
        record_stills(Grey(), tmp_path / "b", "wrist", 1, wait=lambda _: None, clock=Clock())

    class Unread(StandInRobot):
        def get_observation(self):
            observation = super().get_observation()
            del observation["gripper.pos"]
            return observation

    with pytest.raises(ValueError, match="six SO-101"):
        record_stills(Unread(), tmp_path / "c", "wrist", 1, wait=lambda _: None, clock=Clock())

    (tmp_path / "used").mkdir()
    (tmp_path / "used" / "notes.txt").write_text("keep me", encoding="utf-8")
    with pytest.raises(ValueError, match="not empty"):
        record_scan(StandInRobot(), tmp_path / "used", "wrist", 1.0)
    for bad in ({"seconds": 0}, {"seconds": 1.0, "rate": 0}):
        with pytest.raises(ValueError):
            record_scan(
                StandInRobot(), tmp_path / "d", "wrist", bad["seconds"], rate=bad.get("rate", 10)
            )
    with pytest.raises(ValueError):
        record_stills(StandInRobot(), tmp_path / "e", "wrist", 0)
