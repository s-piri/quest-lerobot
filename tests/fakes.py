"""Test doubles only. Neither is the official follower or an SO-101 kinematic model."""

from lerobot_teleoperator_quest.contracts import So101State
from lerobot_teleoperator_quest.pose import IDENTITY, Pose
from lerobot_teleoperator_quest.so101 import (
    from_follower_action,
    from_follower_observation,
    to_follower_action,
)


class ToyKinematics:
    """A gantry: the first three joints ARE the tool position. Exercises the gate, not IK."""

    def __init__(self, reach: float = 10.0):
        self.reach = reach
        self.reply = None  # set to force a specific inverse-kinematics result

    def fk(self, joints):
        return Pose(tuple(joints[:3]), IDENTITY)

    def ik(self, target, seed):
        if self.reply is not None:
            return self.reply
        if any(abs(p) > self.reach for p in target.position):
            return None
        return (*target.position, seed[3], seed[4])


class FakeFollower:
    """Position servos that lag behind their goal, with a gripper that can jam."""

    def __init__(self, state: So101State, *, speed=1.0, gripper_speed=1.0, jam_below=None):
        self.measured = state
        self.goal = state
        self.speed = speed
        self.gripper_speed = gripper_speed
        self.jam_below = jam_below
        self.sent = []

    def get_observation(self) -> dict:
        return to_follower_action(self.measured)

    def send_action(self, action: dict) -> dict:
        self.goal = from_follower_action(action)
        self.sent.append(action)
        return action

    def state(self) -> So101State:
        return from_follower_observation(self.get_observation())

    def advance(self, dt: float):
        def toward(now, goal, rate):
            return now + min(max(goal - now, -rate * dt), rate * dt)

        joints = tuple(
            toward(now, goal, self.speed)
            for now, goal in zip(self.measured.joints_rad, self.goal.joints_rad, strict=True)
        )
        gripper = toward(self.measured.gripper, self.goal.gripper, self.gripper_speed)
        if self.jam_below is not None:
            gripper = max(gripper, self.jam_below)
        self.measured = So101State(joints, gripper)
