"""Compose the motion gate with LeRobot's processor API. It opens no robot.

LeRobot's loops send an action on every tick, but the gate can decide to send
nothing. This step then repeats the goal it returned last, which changes
nothing for a position servo. Before it has returned any goal it latches the
first measured state once, which holds the arm where it already is.
"""

from collections.abc import Mapping
from time import monotonic

from lerobot.configs import FeatureType, PipelineFeatureType, PolicyFeature
from lerobot.processor import (
    RobotAction,
    RobotActionProcessorStep,
    RobotObservation,
    RobotProcessorPipeline,
    TransitionKey,
    robot_action_observation_to_transition,
    transition_to_robot_action,
)

from .contracts import So101State, Telemetry
from .control import DISARMED, Controller, Group, Step
from .input import HANDS, action_sample, input_action
from .so101 import FOLLOWER_KEYS, from_follower_observation, to_follower_action


def _prefixes(arm: str) -> tuple[str, str]:
    """Input features, then the official follower's keys (`left_` for a bimanual arm)."""
    return (f"quest.{arm}.", f"{arm}_") if arm else ("quest.", "")


class QuestControlStep(RobotActionProcessorStep):
    """Quest input and the follower's observation in; follower `.pos` goals out.

    Pass one `Controller`, or a `Group` whose arms are named "left" and "right"
    for the official bimanual SO follower. The host application arms, stops and
    clears through this step; nothing here reads a keyboard or a headset button.
    """

    def __init__(self, control: Controller | Group, *, clock=monotonic):
        self.group = control if isinstance(control, Group) else Group({"": control})
        if set(self.group.controllers) not in ({""}, set(HANDS)):
            raise ValueError('arms must be one unnamed controller, or "left" and "right"')
        self._clock = clock
        self._observation = None
        self._observed_at = 0.0
        self._feedback: dict[str, Telemetry | None] = {}
        self._goal: dict[str, So101State] = {}
        self.steps: dict[str, Step] = {}  # each arm's latest decision
        self.telemetry: dict[str, Telemetry] = {}  # measured, and the goal returned beside it

    def arm(self, calibration: str | Mapping[str, str]):
        """Give the current calibration fingerprint: one, or one per arm."""
        if isinstance(calibration, str):
            calibration = {"": calibration}
        self.group.arm(self._feedback, calibration, self._clock())

    def disarm(self):
        self.group.disarm()

    def stop(self, reason: str = "stop requested"):
        self.group.stop(reason)

    def clear(self):
        self.group.clear()

    def headset_state(self) -> dict:
        """What the headset draws, for `Quest.send_state`: each arm's measured joints and
        the goal its hand is asking for, in the model's radians with the gripper 0-1.

        Joints are None until the arm has been armed, because only then is the
        calibration known that turns follower values into model joints.
        """
        arms = {}
        for arm, controller in self.group.controllers.items():
            step = self.steps.get(arm)
            record = self._feedback.get(arm)
            measured = None if record is None else controller.urdf_joints(record.measured)
            goal = None if step is None else controller.urdf_joints(step.goal)
            arms[arm] = {
                "state": DISARMED if step is None else step.state,
                "reason": DISARMED if step is None else step.reason,
                "base": list(controller.xr_to_base),
                "measured": None if measured is None else [*measured, record.measured.gripper],
                "goal": None if goal is None else [*goal, step.goal.gripper],
            }
        return {"type": "state", "arms": arms}

    def action(self, action: RobotAction) -> RobotAction:
        now = self._clock()
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is not self._observation:
            # A new object is the only evidence of a new measurement this API offers.
            self._observation, self._observed_at = observation, now
        inputs, feedback = {}, {}
        for arm in self.group.controllers:
            source, follower = _prefixes(arm)
            inputs[arm] = action_sample(action, source)
            feedback[arm] = self._measure(observation, follower, self._observed_at)
        self._feedback = feedback
        self.steps = self.group.tick(inputs, feedback, now)

        goals: RobotAction = {}
        for arm, step in self.steps.items():
            goal = step.command if step.command is not None else self._goal.get(arm)
            if goal is None and feedback[arm] is not None:
                goal = feedback[arm].measured
            if goal is None:
                raise RuntimeError(f"{arm or 'the arm'} has no measured state and no earlier goal")
            self._goal[arm] = goal
            if feedback[arm] is None:
                self.telemetry.pop(arm, None)
            else:
                self.telemetry[arm] = Telemetry(
                    feedback[arm].measured, goal, None, feedback[arm].received_at
                )
            follower = _prefixes(arm)[1]
            goals |= {follower + key: value for key, value in to_follower_action(goal).items()}
        return goals

    @staticmethod
    def _measure(observation, prefix: str, at: float) -> Telemetry | None:
        """None when there is no usable measurement, which faults an armed gate."""
        if not isinstance(observation, Mapping):
            return None
        try:
            own = {key: observation[prefix + key] for key in FOLLOWER_KEYS}
            return Telemetry(from_follower_observation(own), None, None, at)
        except (KeyError, ValueError):
            return None

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        actions = features[PipelineFeatureType.ACTION]
        for arm in self.group.controllers:
            source, follower = _prefixes(arm)
            for key in input_action(None, 0, source):
                actions.pop(key, None)
            for key in FOLLOWER_KEYS:
                actions[follower + key] = PolicyFeature(type=FeatureType.ACTION, shape=(1,))
        return features


def quest_processor(
    step: QuestControlStep,
) -> RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction]:
    """The pipeline shape LeRobot's loops call with `(teleop_action, observation)`."""
    return RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        steps=[step],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )
