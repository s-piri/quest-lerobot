from math import radians

import pytest
from fakes import FakeFollower, ToyKinematics

from lerobot_teleoperator_quest.contracts import So101State, Telemetry
from lerobot_teleoperator_quest.control import (
    ARMED,
    DISARMED,
    ENGAGED,
    FAULTED,
    STOPPED,
    Controller,
    Limits,
    Step,
)
from lerobot_teleoperator_quest.input import ControllerSample, InputBuffer
from lerobot_teleoperator_quest.pose import IDENTITY
from lerobot_teleoperator_quest.so101 import (
    UrdfMapping,
    calibration_fingerprint,
    to_follower_action,
)

CAL = calibration_fingerprint({"shoulder_pan": {"id": 1, "homing_offset": 12}})
OTHER_CAL = calibration_fingerprint({"shoulder_pan": {"id": 1, "homing_offset": 13}})
MAPPING = UrdfMapping((1, -1, 1, 1, 1), (0, 10, 0, 0, 0), CAL)
REST = So101State((0.1, 0.2, 0.3, 0.0, 0.0), 0.5)
TICK = 0.01


def limits(**changes):
    values = {
        "joint_min": (-2,) * 5,
        "joint_max": (2,) * 5,
        "joint_speed": (1,) * 5,
        "gripper_min": 0,
        "gripper_max": 1,
        "gripper_speed": 2,
        "workspace_min": (-1,) * 3,
        "workspace_max": (1,) * 3,
        "max_step_s": 0.05,
    }
    return Limits(**{**values, **changes})


def controller(kinematics=None, **changes):
    values = {
        "kinematics": kinematics or ToyKinematics(),
        "mapping": MAPPING,
        "limits": limits(),
        "xr_to_base": IDENTITY,
        "feedback_timeout": 0.1,
    }
    return Controller(**{**values, **changes})


def sample(seq, at, *, position=(0, 0, 0), grip=True, gripper=0.5, scale=1.0):
    return ControllerSample(seq, position, IDENTITY, grip, gripper, scale, at)


def feedback(at, state=REST):
    return Telemetry(state, None, None, at)


def engaged(gate, *, at=1.0, state=REST, session=1):
    """Arm, release, then press: the only route to motion."""
    gate.arm(feedback(at, state), CAL, at)
    gate.tick(sample(0, at, grip=False), session, feedback(at, state), at)
    return gate.tick(sample(1, at + TICK), session, feedback(at + TICK, state), at + TICK)


def test_starts_disarmed_and_sends_nothing_whatever_the_input():
    gate = controller()
    assert gate.state == DISARMED
    for seq, grip in enumerate((False, True, True)):
        step = gate.tick(sample(seq, 1.0, grip=grip), 1, feedback(1.0), 1.0)
        assert (step.state, step.command) == (DISARMED, None)


def test_arming_requires_fresh_feedback_and_the_mapped_calibration():
    gate = controller()
    with pytest.raises(RuntimeError, match="fresh measured feedback"):
        gate.arm(None, CAL, 1.0)
    with pytest.raises(RuntimeError, match="fresh measured feedback"):
        gate.arm(feedback(0.5), CAL, 1.0)
    with pytest.raises(RuntimeError, match="fresh measured feedback"):
        gate.arm(feedback(1.5), CAL, 1.0)  # a timestamp from the future is not fresh
    with pytest.raises(ValueError, match="calibration does not match"):
        gate.arm(feedback(1.0), OTHER_CAL, 1.0)
    assert gate.state == DISARMED
    gate.arm(feedback(1.0), CAL, 1.0)
    with pytest.raises(RuntimeError, match="cannot arm while armed"):
        gate.arm(feedback(1.0), CAL, 1.0)


def test_arming_sends_nothing_and_a_held_grip_cannot_engage():
    gate = controller()
    gate.arm(feedback(1.0), CAL, 1.0)
    for i in range(5):
        at = 1.0 + i * TICK
        step = gate.tick(sample(i, at), 1, feedback(at), at)
        assert (step.state, step.command) == (ARMED, None)
        assert step.reason == "release the grip before pressing it"
    step = gate.tick(sample(5, 1.05, grip=False), 1, feedback(1.05), 1.05)
    assert (step.state, step.command, step.reason) == (ARMED, None, "ready")
    assert gate.tick(sample(6, 1.06), 1, feedback(1.06), 1.06).state == ENGAGED


def test_engaging_anchors_from_the_measured_state_not_an_old_request():
    gate = controller()
    assert engaged(gate).command == REST

    # Drive away, release, and let the arm settle somewhere else entirely.
    for i in range(2, 30):
        at = 1.0 + i * TICK
        gate.tick(sample(i, at, position=(0.3, 0, 0)), 1, feedback(at), at)
    elsewhere = So101State((-0.4, 0.1, 0.2, 0.3, 0.0), 0.2)
    gate.tick(sample(30, 1.30, grip=False), 1, feedback(1.30, elsewhere), 1.30)
    step = gate.tick(sample(31, 1.31, position=(0.3, 0, 0)), 1, feedback(1.31, elsewhere), 1.31)
    assert (step.state, step.command) == (ENGAGED, elsewhere)


def test_controller_motion_becomes_scaled_follower_motion_through_the_mapping():
    gate = controller()
    engaged(gate)
    step = None
    for i in range(2, 12):
        at = 1.0 + i * TICK
        moved = sample(i, at, position=(0.02, 0.04, -0.01), scale=0.5)
        step = gate.tick(moved, 1, feedback(at), at)
    # The scale latched at engagement was 1.0; the later 0.5 detent changes nothing.
    # Joint 1 has sign -1 in the mapping, so +y in the model lowers the follower value.
    assert step.command.joints_rad == pytest.approx((0.12, 0.16, 0.29, 0.0, 0.0))
    assert step.reason == "tracking"


def test_scale_is_latched_per_grip():
    gate = controller()
    gate.arm(feedback(1.0), CAL, 1.0)
    gate.tick(sample(0, 1.0, grip=False), 1, feedback(1.0), 1.0)
    step = None
    for i in range(1, 12):
        at = 1.0 + i * TICK
        step = gate.tick(sample(i, at, position=(0.04, 0, 0), scale=0.25), 1, feedback(at), at)
    assert step.command.joints_rad[0] == pytest.approx(0.1)  # anchored where it engaged
    for i in range(12, 24):
        at = 1.0 + i * TICK
        step = gate.tick(sample(i, at, position=(0.08, 0, 0), scale=1.0), 1, feedback(at), at)
    assert step.command.joints_rad[0] == pytest.approx(0.1 + 0.25 * 0.04)


def test_joint_speed_uses_elapsed_time_and_a_late_tick_cannot_leap():
    gate = controller()
    engaged(gate)
    far = (0.9, 0, 0)
    step = gate.tick(sample(2, 1.02, position=far), 1, feedback(1.02), 1.02)
    assert step.command.joints_rad[0] == pytest.approx(0.1 + 1 * 0.01)
    step = gate.tick(sample(3, 1.04, position=far), 1, feedback(1.04), 1.04)
    assert step.command.joints_rad[0] == pytest.approx(0.11 + 1 * 0.02)
    # 90 ms late but still inside both watchdogs: capped at max_step_s of travel.
    step = gate.tick(sample(4, 1.13, position=far), 1, feedback(1.13), 1.13)
    assert step.command.joints_rad[0] == pytest.approx(0.13 + 1 * 0.05)


def test_joint_and_workspace_limits_bound_the_command():
    gate = controller(limits=limits(joint_max=(0.15, 2, 2, 2, 2), workspace_max=(1, 1, 0.32)))
    engaged(gate)
    step = None
    for i in range(2, 60):
        at = 1.0 + i * TICK
        step = gate.tick(sample(i, at, position=(0.5, 0, 0.5)), 1, feedback(at), at)
    assert step.command.joints_rad[0] == pytest.approx(0.15)
    assert step.command.joints_rad[2] == pytest.approx(0.32)


def test_starting_outside_the_limits_never_moves_the_arm_without_the_hand():
    outside = So101State((0.4, 0.2, 0.5, 0.0, 0.0), 0.05)
    tight = limits(joint_max=(0.15, 2, 2, 2, 2), workspace_max=(1, 1, 0.32), gripper_min=0.2)
    gate = controller(limits=tight)
    engaged(gate, state=outside)
    step = None
    for i in range(2, 40):
        at = 1.0 + i * TICK
        step = gate.tick(sample(i, at, gripper=0.05), 1, feedback(at, outside), at)
    assert step.command == outside  # a still hand asks for no motion at all

    # Further out is refused on every bounded axis; back inward is allowed.
    for i in range(40, 80):
        at = 1.0 + i * TICK
        further = sample(i, at, position=(0.1, 0, 0.1), gripper=0.0)
        step = gate.tick(further, 1, feedback(at, outside), at)
    assert step.command == outside
    for i in range(80, 140):
        at = 1.0 + i * TICK
        inward = sample(i, at, position=(-0.1, 0, -0.1), gripper=0.5)
        step = gate.tick(inward, 1, feedback(at, outside), at)
    assert step.command.joints_rad == pytest.approx((0.3, 0.2, 0.4, 0.0, 0.0))
    assert step.command.gripper == pytest.approx(0.5)
    for i in range(140, 260):
        at = 1.0 + i * TICK
        step = gate.tick(sample(i, at, position=(0.5, 0, 0.5)), 1, feedback(at, outside), at)
    # The joint limit now holds at where the command is; the workspace at where the grip began.
    assert step.command.joints_rad == pytest.approx((0.3, 0.2, 0.5, 0.0, 0.0))


def test_gripper_follows_only_while_engaged_within_its_limits_and_speed():
    gate = controller(limits=limits(gripper_min=0.2, gripper_speed=1))
    engaged(gate)
    step = gate.tick(sample(2, 1.02, gripper=0.0), 1, feedback(1.02), 1.02)
    assert step.command.gripper == pytest.approx(0.49)
    for i in range(3, 60):
        at = 1.0 + i * TICK
        step = gate.tick(sample(i, at, gripper=0.0), 1, feedback(at), at)
    assert step.command.gripper == pytest.approx(0.2)


@pytest.mark.parametrize(
    "reply",
    [(0.0, 0.0), (float("nan"), 0, 0, 0, 0), (float("inf"), 0, 0, 0, 0), "wrong", (1e400,) * 5],
)
def test_failed_or_malformed_inverse_kinematics_stops_progress(reply):
    toy = ToyKinematics(reach=0.5)
    gate = controller(toy)
    engaged(gate)
    before = gate.tick(sample(2, 1.02, position=(0.05, 0, 0)), 1, feedback(1.02), 1.02).command
    step = gate.tick(sample(3, 1.03, position=(0.9, 0, 0)), 1, feedback(1.03), 1.03)
    assert (step.state, step.command) == (ENGAGED, before)
    assert step.reason == "inverse kinematics rejected the target"
    toy.reply = reply
    step = gate.tick(sample(4, 1.04, position=(0.05, 0, 0)), 1, feedback(1.04), 1.04)
    assert (step.state, step.command) == (ENGAGED, before)
    toy.reply = None
    step = gate.tick(sample(5, 1.05, position=(0.05, 0, 0)), 1, feedback(1.05), 1.05)
    assert step.reason == "tracking"


def test_release_holds_at_the_measured_state_not_the_last_request():
    gate = controller()
    engaged(gate)
    step = None
    for i in range(2, 20):
        at = 1.0 + i * TICK
        step = gate.tick(sample(i, at, position=(0.5, 0, 0)), 1, feedback(at), at)
    assert step.command.joints_rad[0] > 0.25
    lagging = So101State((0.14, 0.2, 0.3, 0.0, 0.0), 0.5)
    step = gate.tick(sample(20, 1.20, grip=False), 1, feedback(1.20, lagging), 1.20)
    assert (step.state, step.command, step.reason) == (ARMED, lagging, "ready")
    # The hold is latched once; later sag in the measurement must not walk it downward.
    sagging = So101State((0.13, 0.2, 0.3, 0.0, 0.0), 0.5)
    step = gate.tick(sample(21, 1.21, grip=False), 1, feedback(1.21, sagging), 1.21)
    assert step.command == lagging


def test_stale_input_holds_and_needs_a_new_release_and_press():
    gate = controller()
    engaged(gate)
    gate.tick(sample(2, 1.02, position=(0.1, 0, 0)), 1, feedback(1.02), 1.02)
    # The loop keeps ticking with the last sample while no new input arrives.
    old = sample(3, 1.03, position=(0.1, 0, 0))
    assert gate.tick(old, 1, feedback(1.20), 1.20).state == ENGAGED
    step = gate.tick(old, 1, feedback(1.24), 1.24)
    assert (step.state, step.command, step.reason) == (ARMED, REST, "no fresh input")
    assert gate.tick(None, 1, feedback(1.25), 1.25).command == REST

    step = gate.tick(sample(4, 1.26, position=(0.1, 0, 0)), 1, feedback(1.26), 1.26)
    assert (step.state, step.reason) == (ARMED, "release the grip before pressing it")
    gate.tick(sample(5, 1.27, grip=False), 1, feedback(1.27), 1.27)
    assert gate.tick(sample(6, 1.28), 1, feedback(1.28), 1.28).state == ENGAGED


def test_a_release_seen_before_an_input_gap_does_not_survive_it():
    gate = controller()
    gate.arm(feedback(1.0), CAL, 1.0)
    assert gate.tick(sample(0, 1.0, grip=False), 1, feedback(1.0), 1.0).reason == "ready"
    assert gate.tick(None, 1, feedback(1.05), 1.05).reason == "no fresh input"
    step = gate.tick(sample(1, 1.06), 1, feedback(1.06), 1.06)
    assert (step.state, step.reason) == (ARMED, "release the grip before pressing it")


def test_input_stamped_in_the_future_is_not_fresh():
    gate = controller()
    gate.arm(feedback(1.0), CAL, 1.0)
    step = gate.tick(sample(0, 5.0, grip=False), 1, feedback(1.0), 1.0)
    assert step.reason == "no fresh input"


def test_out_of_order_and_older_session_input_cannot_drive():
    gate = controller()
    engaged(gate, session=2)
    gate.tick(sample(7, 1.02), 2, feedback(1.02), 1.02)
    step = gate.tick(sample(6, 1.03, position=(0.3, 0, 0)), 2, feedback(1.03), 1.03)
    assert (step.state, step.reason) == (ARMED, "no fresh input")
    step = gate.tick(sample(99, 1.04, grip=False), 1, feedback(1.04), 1.04)
    assert step.reason == "no fresh input"
    for bad in (0, -1, True, None, 2.0):
        assert gate.tick(sample(100, 1.05), bad, feedback(1.05), 1.05).reason == "no fresh input"
    # Repeating the newest sample is normal: the loop outpaces the controller.
    assert gate.tick(sample(7, 1.06, grip=False), 2, feedback(1.06), 1.06).reason == "ready"


def test_reconnection_cannot_inherit_an_engaged_grip():
    gate = controller()
    engaged(gate, session=1)
    step = gate.tick(sample(0, 1.02, position=(0.3, 0, 0)), 2, feedback(1.02), 1.02)
    assert (step.state, step.command) == (ARMED, REST)
    assert step.reason == "release the grip before pressing it"
    assert gate.tick(sample(1, 1.03), 2, feedback(1.03), 1.03).state == ARMED
    gate.tick(sample(2, 1.04, grip=False), 2, feedback(1.04), 1.04)
    assert gate.tick(sample(3, 1.05), 2, feedback(1.05), 1.05).state == ENGAGED


def test_a_release_seen_in_another_session_does_not_count():
    gate = controller()
    gate.arm(feedback(1.0), CAL, 1.0)
    gate.tick(sample(0, 1.0, grip=False), 1, feedback(1.0), 1.0)
    step = gate.tick(sample(0, 1.01), 2, feedback(1.01), 1.01)
    assert (step.state, step.reason) == (ARMED, "release the grip before pressing it")


@pytest.mark.parametrize("stale", [None, "old", "future", "wrong type"])
def test_stale_feedback_faults_and_never_seeds_a_command(stale):
    gate = controller()
    engaged(gate)
    gate.tick(sample(2, 1.02, position=(0.2, 0, 0)), 1, feedback(1.02), 1.02)
    lost = {
        None: None,
        "old": feedback(0.9),
        "future": feedback(9.0),
        "wrong type": REST,
    }[stale]
    step = gate.tick(sample(3, 1.03, position=(0.2, 0, 0)), 1, lost, 1.03)
    assert (step.state, step.command) == (FAULTED, None)
    assert step.reason == "measured feedback is stale"
    # Feedback returning does not resume, hold or re-arm.
    step = gate.tick(sample(4, 1.04, grip=False), 1, feedback(1.04), 1.04)
    assert (step.state, step.command) == (FAULTED, None)
    with pytest.raises(RuntimeError, match="cannot arm while faulted"):
        gate.arm(feedback(1.04), CAL, 1.04)
    gate.clear()
    assert gate.state == DISARMED
    gate.arm(feedback(1.05), CAL, 1.05)
    step = gate.tick(sample(5, 1.05), 1, feedback(1.05), 1.05)
    assert (step.state, step.reason) == (ARMED, "release the grip before pressing it")


def test_stale_feedback_faults_while_armed_and_idle():
    gate = controller()
    gate.arm(feedback(1.0), CAL, 1.0)
    assert gate.tick(None, 0, feedback(1.0), 1.2).state == FAULTED


def test_stop_latches_and_holds_from_fresh_measurement():
    gate = controller()
    engaged(gate)
    for i in range(2, 10):
        at = 1.0 + i * TICK
        gate.tick(sample(i, at, position=(0.5, 0, 0)), 1, feedback(at), at)
    gate.stop("operator pressed stop")
    assert gate.state == STOPPED
    lagging = So101State((0.13, 0.2, 0.3, 0.0, 0.0), 0.5)
    step = gate.tick(sample(10, 1.10, position=(0.5, 0, 0)), 1, feedback(1.10, lagging), 1.10)
    assert (step.state, step.command, step.reason) == (STOPPED, lagging, "operator pressed stop")
    step = gate.tick(sample(11, 1.11, grip=False), 1, feedback(1.11), 1.11)
    assert (step.state, step.command) == (STOPPED, lagging)

    with pytest.raises(RuntimeError, match="cannot arm while stopped"):
        gate.arm(feedback(1.11), CAL, 1.11)
    gate.stop("a second cause")
    assert gate.tick(None, 0, feedback(1.12), 1.12).reason == "operator pressed stop"
    gate.clear()
    assert gate.state == DISARMED
    assert gate.tick(sample(12, 1.13), 1, feedback(1.13), 1.13).command is None
    with pytest.raises(RuntimeError, match="no latched stop"):
        gate.clear()


def test_stop_without_fresh_feedback_sends_nothing():
    gate = controller()
    engaged(gate)
    gate.stop()
    step = gate.tick(None, 0, feedback(1.0), 1.5)
    assert (step.state, step.command) == (STOPPED, None)
    assert gate.tick(None, 0, feedback(1.6), 1.6).command is None


def test_stop_while_idle_keeps_the_hold_and_stop_while_disarmed_still_latches():
    gate = controller()
    engaged(gate)
    held = gate.tick(sample(2, 1.02, grip=False), 1, feedback(1.02), 1.02).command
    gate.stop()
    moved = So101State((0.0,) * 5, 0.5)
    assert gate.tick(None, 0, feedback(1.03, moved), 1.03).command == held

    idle = controller()
    idle.stop()
    assert idle.tick(None, 0, feedback(1.0), 1.0) == Step(STOPPED, None, "stop requested")
    with pytest.raises(RuntimeError, match="cannot arm while stopped"):
        idle.arm(feedback(1.0), CAL, 1.0)


def test_disarm_is_only_for_an_idle_armed_gate():
    gate = controller()
    with pytest.raises(RuntimeError, match="cannot disarm while disarmed"):
        gate.disarm()
    engaged(gate)
    with pytest.raises(RuntimeError, match="stop instead"):
        gate.disarm()
    gate.tick(sample(2, 1.02, grip=False), 1, feedback(1.02), 1.02)
    gate.disarm()
    assert gate.tick(sample(3, 1.03, grip=False), 1, feedback(1.03), 1.03).command is None
    gate.arm(feedback(1.04), CAL, 1.04)
    step = gate.tick(sample(4, 1.04), 1, feedback(1.04), 1.04)
    assert (step.command, step.reason) == (None, "release the grip before pressing it")


def test_a_backwards_clock_is_an_error_not_a_negative_step():
    gate = controller()
    gate.tick(None, 0, None, 2.0)
    with pytest.raises(ValueError, match="clock moved backwards"):
        gate.tick(None, 0, None, 1.0)


def test_forward_kinematics_that_returns_no_pose_faults_instead_of_engaging():
    toy = ToyKinematics()
    toy.fk = lambda joints: None
    gate = controller(toy)
    step = engaged(gate)
    assert (step.state, step.command) == (FAULTED, None)
    assert step.reason == "forward kinematics did not return a pose"


@pytest.mark.parametrize(
    "changes",
    [
        {"joint_min": (0,) * 5, "joint_max": (0,) * 5},
        {"joint_min": (-2,) * 4},
        {"joint_speed": (1, 1, 1, 1, 0)},
        {"joint_speed": (1, 1, 1, 1, float("nan"))},
        {"gripper_min": 0.6, "gripper_max": 0.4},
        {"gripper_max": 1.5},
        {"gripper_speed": -1},
        {"workspace_min": (1, 1, 1)},
        {"max_step_s": 0},
    ],
)
def test_limits_reject_malformed_bounds(changes):
    with pytest.raises(ValueError):
        limits(**changes)


@pytest.mark.parametrize(
    "changes",
    [
        {"mapping": None},
        {"limits": None},
        {"xr_to_base": (0, 0, 0, 0)},
        {"feedback_timeout": 0},
        {"input_timeout": float("inf")},
        {"rotation_scale": -1},
    ],
)
def test_controller_rejects_malformed_configuration(changes):
    with pytest.raises(ValueError):
        controller(**changes)


class Loop:
    """A fixed-rate host loop: real input buffer, the gate, and the fake follower."""

    def __init__(self, follower, **changes):
        self.now = 10.0
        self.buffer = InputBuffer(clock=lambda: self.now)
        self.follower = follower
        self.gate = controller(**changes)
        self.session = self.buffer.open_session()
        self.seq = 0
        self.steps = []

    def feedback(self):
        return Telemetry(self.follower.state(), None, None, self.now)

    def run(self, ticks, **pose):
        """Tick the loop. With no pose, the controller has simply gone quiet."""
        for _ in range(ticks):
            self.now += TICK
            if pose:
                self.buffer.submit(
                    self.session,
                    {
                        "type": "pose",
                        "version": 1,
                        "seq": self.seq,
                        "position": [0, 0, 0],
                        "orientation": [0, 0, 0, 1],
                        "gripper": 0.5,
                        "scale": 1.0,
                        **pose,
                    },
                )
                self.seq += 1
            step = self.gate.tick(*self.buffer.snapshot(), self.feedback(), self.now)
            if step.command is not None:
                self.follower.send_action(to_follower_action(step.command))
            self.follower.advance(TICK)
            self.steps.append(step)
        return self.steps[-1]


def test_end_to_end_input_going_quiet_stops_progress_while_the_loop_keeps_ticking():
    loop = Loop(FakeFollower(REST, speed=0.5))
    loop.gate.arm(loop.feedback(), CAL, loop.now)
    loop.run(2, grip=False)
    assert loop.follower.sent == []
    assert loop.run(1, grip=True).command.joints_rad == pytest.approx(REST.joints_rad)
    step = loop.run(30, grip=True, position=[0.3, 0, 0])
    assert step.state == ENGAGED
    assert step.command.joints_rad[0] > loop.follower.measured.joints_rad[0] > 0.1  # it lags

    loop.run(25)  # the headset stops sending; nothing blocks the watchdog
    states = [s.state for s in loop.steps[-25:]]
    assert states.index(ARMED) <= 21  # within the 200 ms input timeout, to one tick
    hold = loop.steps[-1].command
    assert hold.joints_rad[0] < step.command.joints_rad[0]  # measured, not the request
    loop.run(50)
    assert loop.follower.measured.joints_rad == pytest.approx(hold.joints_rad)

    # Input resumes with the grip still held: no motion until release and press.
    assert loop.run(10, grip=True, position=[0.6, 0, 0]).state == ARMED
    loop.run(1, grip=False)
    assert loop.run(1, grip=True).state == ENGAGED


def test_end_to_end_a_jammed_gripper_stays_measured_and_seeds_the_hold():
    loop = Loop(FakeFollower(REST, jam_below=0.4))
    loop.gate.arm(loop.feedback(), CAL, loop.now)
    loop.run(1, grip=False)
    step = loop.run(60, grip=True, gripper=0.0)
    assert step.command.gripper == pytest.approx(0.0)
    assert loop.follower.measured.gripper == pytest.approx(0.4)
    record = Telemetry(loop.follower.state(), step.command, None, loop.now)
    assert record.to_dict()["measured"]["gripper"] == pytest.approx(0.4)
    assert record.to_dict()["requested"]["gripper"] == pytest.approx(0.0)

    hold = loop.run(1, grip=False).command
    assert hold.gripper == pytest.approx(0.4)
    # The next grip starts from where the gripper really is.
    assert loop.run(1, grip=True, gripper=0.0).command.gripper == pytest.approx(0.4)


def test_end_to_end_reconnect_with_the_grip_held_does_not_move_the_arm():
    loop = Loop(FakeFollower(REST))
    loop.gate.arm(loop.feedback(), CAL, loop.now)
    loop.run(1, grip=False)
    loop.run(1, grip=True)
    loop.run(10, grip=True, position=[0.2, 0, 0])
    assert loop.follower.measured.joints_rad[0] > 0.1  # it was really moving
    loop.buffer.close_session(loop.session)
    loop.session = loop.buffer.open_session()
    loop.seq = 0
    step = loop.run(10, grip=True, position=[0.9, 0, 0])
    assert step.state == ARMED
    loop.run(60)
    assert loop.follower.measured.joints_rad == pytest.approx(step.command.joints_rad)
    assert loop.follower.measured.joints_rad[0] < 0.2


def test_mapping_zero_and_sign_are_applied_before_kinematics():
    seen = []
    toy = ToyKinematics()
    real_fk = toy.fk
    toy.fk = lambda joints: seen.append(joints) or real_fk(joints)
    engaged(controller(toy))
    assert seen == [pytest.approx((0.1, -(0.2 - radians(10)), 0.3, 0.0, 0.0))]
