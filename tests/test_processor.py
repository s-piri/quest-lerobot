import importlib.util
import json
import time
from math import degrees, radians

import pytest

pytestmark = pytest.mark.integration
if any(importlib.util.find_spec(name) is None for name in ("lerobot", "websockets")):
    pytest.skip("install .[lerobot] for the real integration gate", allow_module_level=True)

from fakes import FakeFollower
from lerobot.configs import FeatureType, PipelineFeatureType, PolicyFeature
from lerobot.processor import DataProcessorPipeline
from lerobot.teleoperators import make_teleoperator_from_config
from test_control import CAL, OTHER_CAL, REST, TICK, controller
from test_group import CALS, RIGHT, RIGHT_MAPPING
from test_input import frame, hand, pose
from websockets.sync.client import connect

from lerobot_teleoperator_quest import QuestConfig
from lerobot_teleoperator_quest.contracts import So101State
from lerobot_teleoperator_quest.control import ARMED, DISARMED, ENGAGED, FAULTED, STOPPED, Group
from lerobot_teleoperator_quest.input import InputBuffer, frame_action
from lerobot_teleoperator_quest.processor import QuestControlStep, quest_processor
from lerobot_teleoperator_quest.so101 import FOLLOWER_KEYS, to_follower_action


def press(dx=0.0):
    """One controller, gripped, moved dx metres from where the default pose sits."""
    return pose(grip=True, position=[1 + dx, 2, 3], scale=1.0)


def grasp(dx=0.0):
    """The same for one hand of a two-hand frame."""
    return hand(True, position=[1 + dx, 2, 3], scale=1.0)


def follower_prefix(arm):
    return f"{arm}_" if arm else ""


class Rig:
    """LeRobot's loop shape around fakes: observe, read input, run the pipeline, send."""

    def __init__(self, bimanual=False):
        self.now = 10.0
        self.buffer = InputBuffer(clock=lambda: self.now, bimanual=bimanual)
        self.session = self.buffer.open_session()
        if bimanual:
            self.followers = {"left": FakeFollower(REST), "right": FakeFollower(RIGHT)}
            control = Group({"left": controller(), "right": controller(mapping=RIGHT_MAPPING)})
        else:
            self.followers = {"": FakeFollower(REST)}
            control = controller()
        self.step = QuestControlStep(control, clock=lambda: self.now)
        self.pipeline = quest_processor(self.step)
        self.seq = 0

    def observation(self):
        observation = {"front": "a camera frame"}
        for arm, follower in self.followers.items():
            prefix = follower_prefix(arm)
            observation |= {prefix + k: v for k, v in follower.get_observation().items()}
        return observation

    def tick(self, payload=None, observation=None, ticks=1):
        action = None
        for _ in range(ticks):
            self.now += TICK
            if payload is not None:
                self.buffer.submit(self.session, payload | {"seq": self.seq})
                self.seq += 1
            seen = self.observation() if observation is None else observation
            action = self.pipeline((frame_action(*self.buffer.frame()), seen))
            for arm, follower in self.followers.items():
                prefix = follower_prefix(arm)
                # Split by prefix exactly as the official bimanual follower does.
                own = {k.removeprefix(prefix): v for k, v in action.items() if k.startswith(prefix)}
                follower.send_action(own)
                follower.advance(TICK)
        return action

    def state(self, arm=""):
        return self.step.steps[arm].state


def test_it_is_a_real_lerobot_pipeline_with_follower_action_features():
    rig = Rig()
    assert isinstance(rig.pipeline, DataProcessorPipeline)
    source = {
        key: PolicyFeature(type=FeatureType.ACTION, shape=(1,))
        for key in frame_action({"": None}, 0)
    }
    features = rig.pipeline.transform_features(
        {PipelineFeatureType.ACTION: dict(source), PipelineFeatureType.OBSERVATION: {}}
    )
    assert set(features[PipelineFeatureType.ACTION]) == set(FOLLOWER_KEYS)

    two = Rig(bimanual=True)
    source = frame_action({"left": None, "right": None}, 0)
    features = two.pipeline.transform_features(
        {
            PipelineFeatureType.ACTION: {
                key: PolicyFeature(type=FeatureType.ACTION, shape=(1,)) for key in source
            },
            PipelineFeatureType.OBSERVATION: {},
        }
    )
    assert set(features[PipelineFeatureType.ACTION]) == {
        f"{arm}_{key}" for arm in ("left", "right") for key in FOLLOWER_KEYS
    }


def test_before_arming_it_holds_the_first_measurement_once_and_never_follows_input():
    rig = Rig()
    first = rig.tick(pose(grip=False))
    assert first == pytest.approx(to_follower_action(REST))
    assert set(first) == set(FOLLOWER_KEYS)
    rig.tick(pose(grip=True, position=[0.5, 0, 0]), ticks=5)
    # Someone moves the limp arm: the latched hold must not chase the measurement.
    rig.followers[""].measured = So101State((0.0,) * 5, 0.1)
    rig.followers[""].goal = rig.followers[""].measured
    rig.buffer.close_session(rig.session)
    assert rig.pipeline((frame_action(*rig.buffer.frame()), rig.observation())) == first
    assert rig.state() == DISARMED


def test_arm_release_press_moves_the_follower_and_keeps_measured_apart_from_requested():
    rig = Rig()
    with pytest.raises(RuntimeError, match="fresh measured feedback"):
        rig.step.arm(CAL)  # no observation has passed through yet
    rig.tick(press())
    with pytest.raises(ValueError, match="calibration does not match"):
        rig.step.arm(OTHER_CAL)
    rig.step.arm(CAL)
    rig.tick(press(), ticks=3)
    assert rig.state() == ARMED
    rig.tick(pose(grip=False))
    rig.tick(press())
    assert rig.state() == ENGAGED
    rig.followers[""].speed = 0.5  # a follower slower than the request
    action = rig.tick(press(0.2), ticks=10)
    assert set(action) == set(FOLLOWER_KEYS)
    follower = rig.followers[""]
    assert 0.1 < follower.measured.joints_rad[0] < 0.3  # it really moved, and lags

    record = rig.step.telemetry[""]
    assert record.requested.joints_rad[0] > record.measured.joints_rad[0] + 0.02
    assert record.applied is None
    assert action == pytest.approx(to_follower_action(record.requested))


def test_quiet_input_holds_and_the_loop_keeps_sending_the_same_goal():
    rig = Rig()
    rig.tick(pose())
    rig.step.arm(CAL)
    rig.tick(pose())
    rig.tick(press())
    moving = rig.tick(press(0.8), ticks=15)
    assert moving["shoulder_pan.pos"] == pytest.approx(degrees(0.25))  # on its way to 0.9 rad
    rig.tick(ticks=25)  # the headset goes silent
    assert rig.state() == ARMED
    held = rig.tick()
    # The last sample stays valid for the 200 ms watchdog, then progress ends far short.
    assert degrees(0.25) < held["shoulder_pan.pos"] < degrees(0.25 + 0.22)
    assert rig.tick(ticks=30) == held
    assert rig.followers[""].get_observation() == pytest.approx(held)


def test_a_reused_observation_goes_stale_and_faults_without_new_motion():
    rig = Rig()
    rig.tick(pose())
    rig.step.arm(CAL)
    rig.tick(pose())
    rig.tick(press())
    rig.tick(press(0.3), ticks=5)
    frozen = rig.observation()
    previous = rig.tick(press(0.3), observation=frozen, ticks=9)
    assert rig.state() == ENGAGED  # 80 ms old: inside the 100 ms feedback timeout
    for _ in range(3):
        action = rig.tick(press(0.3), observation=frozen)
        if rig.state() == FAULTED:
            break
        previous = action
    assert rig.state() == FAULTED
    assert action == previous  # the same goal again: nothing new for the servos
    assert rig.tick(press(0.3), ticks=10) == previous  # fresh observations do not resume it
    rig.step.clear()
    rig.tick(press(0.3))
    assert rig.state() == DISARMED


@pytest.mark.parametrize(
    "damage",
    [
        lambda obs: obs.pop("gripper.pos"),
        lambda obs: obs.update({"elbow_flex.pos": float("nan")}),
        lambda obs: obs.update({"gripper.pos": 140.0}),
        lambda obs: obs.update({"wrist_roll.pos": "12"}),
    ],
)
def test_an_unusable_observation_faults_an_armed_gate_and_repeats_the_goal(damage):
    rig = Rig()
    before = rig.tick(pose())
    rig.step.arm(CAL)
    broken = rig.observation()
    damage(broken)
    assert rig.tick(pose(), observation=broken) == before
    assert rig.state() == FAULTED
    assert "" not in rig.step.telemetry


def test_with_no_measurement_and_no_earlier_goal_there_is_nothing_safe_to_return():
    rig = Rig()
    with pytest.raises(RuntimeError, match="no measured state"):
        rig.tick(pose(), observation={"front": "a camera frame"})
    with pytest.raises(ValueError, match="input features"):
        rig.pipeline(({"shoulder_pan.pos": 0.0}, rig.observation()))


def test_stop_and_clear_through_the_step():
    rig = Rig()
    rig.tick(pose())
    rig.step.arm(CAL)
    rig.tick(pose())
    rig.tick(press())
    rig.tick(press(0.3), ticks=10)
    rig.step.stop("operator pressed stop")
    held = rig.tick(press(0.3))
    assert rig.state() == STOPPED
    assert rig.step.steps[""].reason == "operator pressed stop"
    assert rig.tick(pose(grip=False), ticks=5) == held
    rig.step.clear()
    assert rig.tick(press()) == held
    assert rig.state() == DISARMED
    rig.step.arm(CAL)
    rig.tick(pose())
    rig.step.disarm()
    rig.tick(pose())
    assert rig.state() == DISARMED


def test_two_arms_each_follow_their_own_hand_through_one_pipeline():
    rig = Rig(bimanual=True)
    first = rig.tick(frame(left=hand(), right=hand()))
    assert set(first) == {f"{arm}_{key}" for arm in ("left", "right") for key in FOLLOWER_KEYS}
    with pytest.raises(ValueError, match="calibration does not match"):
        rig.step.arm(CAL)  # one fingerprint cannot arm two arms
    rig.step.arm(CALS)
    rig.tick(frame(left=hand(), right=hand()))
    rig.tick(frame(left=grasp(), right=hand()))
    rig.tick(frame(left=grasp(0.2), right=hand()), ticks=20)
    assert (rig.state("left"), rig.state("right")) == (ENGAGED, ARMED)
    assert rig.followers["left"].measured.joints_rad[0] > 0.25
    assert rig.followers["right"].measured.joints_rad == pytest.approx(RIGHT.joints_rad)

    # The right arm's observation disappears: both arms stop.
    broken = rig.observation()
    del broken["right_wrist_flex.pos"]
    rig.tick(frame(left=grasp(0.4), right=hand()), observation=broken)
    assert (rig.state("left"), rig.state("right")) == (STOPPED, FAULTED)
    assert rig.step.steps["left"].reason == "right faulted: measured feedback is stale"
    held = rig.tick(frame(left=grasp(0.4), right=hand()))
    assert rig.tick(frame(left=grasp(0.4), right=grasp()), ticks=20) == held
    assert rig.followers["left"].get_observation()["shoulder_pan.pos"] == pytest.approx(
        held["left_shoulder_pan.pos"]
    )
    assert rig.followers["left"].measured.joints_rad[0] < 0.35  # it never reached +0.4


def test_headset_state_shows_measured_and_goal_in_model_joints():
    rig = Rig()
    assert rig.step.headset_state() == {
        "type": "state",
        "arms": {
            "": {
                "state": DISARMED,
                "reason": DISARMED,
                "base": [0.0, 0.0, 0.0, 1.0],
                "measured": None,
                "goal": None,
            }
        },
    }
    rig.tick(pose())
    assert rig.step.headset_state()["arms"][""]["measured"] is None  # not armed: no calibration
    rig.step.arm(CAL)
    rig.tick(pose())
    arm = rig.step.headset_state()["arms"][""]
    assert (arm["state"], arm["reason"], arm["goal"]) == (ARMED, "ready", None)
    # The mapping in these tests flips joint 1 about a 10 degree zero.
    assert arm["measured"] == pytest.approx([0.1, -(0.2 - radians(10)), 0.3, 0.0, 0.0, 0.5])

    rig.tick(press())
    rig.followers[""].speed = 0.5
    rig.tick(press(0.2) | {"follow": False}, ticks=10)
    arm = rig.step.headset_state()["arms"][""]
    assert (arm["state"], arm["reason"]) == (ENGAGED, "previewing")
    assert arm["goal"][0] == pytest.approx(0.3)
    assert arm["measured"][0] == pytest.approx(0.1)  # the preview did not move the arm
    json.dumps(rig.step.headset_state(), allow_nan=False)

    two = Rig(bimanual=True)
    two.tick(frame(left=hand(), right=hand()))
    two.step.arm(CALS)
    two.tick(frame(left=hand(), right=hand()))
    arms = two.step.headset_state()["arms"]
    assert set(arms) == {"left", "right"}
    assert arms["right"]["measured"][0] == pytest.approx(RIGHT.joints_rad[0])


def test_headset_state_travels_from_the_step_through_the_real_listener(tmp_path):
    quest = make_teleoperator_from_config(QuestConfig(port=0, calibration_dir=tmp_path))
    quest.connect()
    try:
        follower = FakeFollower(REST)
        step = QuestControlStep(controller())
        pipeline = quest_processor(step)
        states = []

        with connect(quest.url) as ws:
            ws.send(json.dumps({"type": "auth", "token": quest.access_token}))
            ws.recv(timeout=2)

            def send(seq, **changes):
                """One sample, then one turn of the host loop, which queues a state."""
                ws.send(json.dumps(pose(seq, **changes)))
                while True:
                    reply = json.loads(ws.recv(timeout=2))
                    if reply["type"] == "accepted":
                        break
                    states.append(reply)
                action = pipeline((quest.get_action(), follower.get_observation()))
                follower.send_action(action)
                follower.advance(0.01)
                quest.send_state(step.headset_state())

            send(0)
            step.arm(CAL)
            send(1)
            send(2, grip=True, scale=1.0)
            for seq in range(3, 14):
                send(seq, grip=True, scale=1.0, position=[1.1, 2, 3], follow=False)
        # Delivery is newest-only, so which states arrive depends on timing; their
        # order and the final picture do not.
        seen = [state["arms"][""] for state in states]
        order = [DISARMED, ARMED, ENGAGED]
        ranks = [order.index(arm["state"]) for arm in seen]
        assert ranks == sorted(ranks)
        last = seen[-1]
        assert (last["state"], last["reason"]) == (ENGAGED, "previewing")
        assert last["goal"][0] == pytest.approx(0.2)
        assert last["measured"][0] == pytest.approx(0.1)
    finally:
        quest.disconnect()


def test_what_the_follower_applied_is_recorded_only_when_it_is_reported():
    rig = Rig()
    rig.tick(pose())
    rig.step.arm(CAL)
    rig.tick(pose())
    rig.tick(press())
    requested = rig.tick(press(0.3), ticks=5)
    before = rig.step.telemetry[""]
    assert before.applied is None  # nothing has been reported for this tick

    # A follower that clips: it moved the shoulder two degrees less than asked.
    clipped = requested | {"shoulder_pan.pos": requested["shoulder_pan.pos"] - 2.0}
    rig.step.applied(clipped)
    record = rig.step.telemetry[""]
    assert degrees(record.applied.joints_rad[0]) == pytest.approx(
        requested["shoulder_pan.pos"] - 2.0
    )
    assert record.applied.joints_rad[0] < record.requested.joints_rad[0]
    assert record.measured == before.measured  # a report changes neither of the others
    assert record.requested == before.requested
    stored = record.to_dict()
    assert len({json.dumps(stored[key]) for key in ("measured", "requested", "applied")}) == 3

    rig.tick(press(0.3))
    assert rig.step.telemetry[""].applied is None  # a new tick, not yet reported

    for bad in (None, [], {"shoulder_pan.pos": 1.0}, requested | {"gripper.pos": float("nan")}):
        with pytest.raises(ValueError):
            rig.step.applied(bad)
    assert rig.step.telemetry[""].applied is None  # a refused report changes nothing


def test_applied_is_recorded_per_arm_and_all_or_nothing():
    two = Rig(bimanual=True)
    sent = two.tick(frame(left=hand(), right=hand()))
    two.step.applied(sent)
    assert two.step.telemetry["left"].applied == two.step.telemetry["left"].requested
    assert two.step.telemetry["right"].applied == two.step.telemetry["right"].requested

    sent = two.tick(frame(left=hand(), right=hand()))
    with pytest.raises(ValueError, match="right_"):
        two.step.applied({key: value for key, value in sent.items() if key.startswith("left_")})
    assert two.step.telemetry["left"].applied is None  # the left arm was not half-recorded


def test_arm_names_must_match_the_official_follower_prefixes():
    with pytest.raises(ValueError, match="left"):
        QuestControlStep(Group({"a": controller(), "b": controller()}))
    with pytest.raises(ValueError, match="left"):
        QuestControlStep(Group({"left": controller()}))


def test_goal_keys_are_exactly_the_official_followers_action_features(tmp_path):
    """Constructing a follower opens no port; nothing here connects."""
    pytest.importorskip("rustypot")
    from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
    from lerobot.robots.so_follower import SOFollower, SOFollowerConfig, SOFollowerRobotConfig

    single = SOFollower(SOFollowerRobotConfig(port="not-a-port", calibration_dir=tmp_path))
    assert not single.is_connected
    assert set(Rig().tick(pose())) == set(single.action_features)

    pair = BiSOFollower(
        BiSOFollowerConfig(
            left_arm_config=SOFollowerConfig(port="not-a-port"),
            right_arm_config=SOFollowerConfig(port="not-a-port-either"),
            calibration_dir=tmp_path,
        )
    )
    assert not pair.left_arm.is_connected and not pair.right_arm.is_connected
    two = Rig(bimanual=True)
    assert set(two.tick(frame(left=hand(), right=hand()))) == set(pair.action_features)
    assert set(two.observation()) - {"front"} == set(pair.observation_features)


def test_real_teleoperator_socket_to_pipeline_for_two_arms(tmp_path):
    """The real listener and its real clock; only the followers are fake."""
    quest = make_teleoperator_from_config(
        QuestConfig(port=0, calibration_dir=tmp_path, bimanual=True)
    )
    quest.connect()
    try:
        followers = {"left": FakeFollower(REST), "right": FakeFollower(RIGHT)}
        step = QuestControlStep(
            Group({"left": controller(), "right": controller(mapping=RIGHT_MAPPING)})
        )
        pipeline = quest_processor(step)

        def loop():
            observation = {}
            for arm, follower in followers.items():
                observation |= {f"{arm}_{k}": v for k, v in follower.get_observation().items()}
            action = pipeline((quest.get_action(), observation))
            for arm, follower in followers.items():
                follower.send_action(
                    {k.removeprefix(f"{arm}_"): v for k, v in action.items() if k.startswith(arm)}
                )
                follower.advance(0.01)
            time.sleep(0.01)

        with connect(quest.url) as ws:
            ws.send(json.dumps({"type": "auth", "token": quest.access_token}))
            assert json.loads(ws.recv(timeout=2)) | {"session": 0} == {
                "type": "ready",
                "version": 2,
                "session": 0,
            }

            def send(seq, left, right):
                ws.send(json.dumps(frame(seq, left, right)))
                assert json.loads(ws.recv(timeout=2)) == {"type": "accepted", "seq": seq}
                loop()

            send(0, hand(), hand())
            step.arm(CALS)
            send(1, hand(), hand())
            send(2, hand(), hand(True))
            for seq in range(3, 20):
                send(seq, hand(), hand(True, position=[1, 2.2, 3]))  # latched scale 0.25
            assert step.steps["right"].state == ENGAGED
            assert step.steps["left"].state == ARMED
            assert followers["right"].measured.joints_rad[1] > RIGHT.joints_rad[1] + 0.02
            assert followers["left"].measured.joints_rad == pytest.approx(REST.joints_rad)
        # The socket closed: input is gone, and the arm holds.
        for _ in range(3):
            loop()
        assert step.steps["right"].state == ARMED
    finally:
        quest.disconnect()
