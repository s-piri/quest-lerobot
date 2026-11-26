import pytest
from test_control import CAL, OTHER_CAL, REST, TICK, controller, feedback, sample

from lerobot_teleoperator_quest.contracts import So101State
from lerobot_teleoperator_quest.control import (
    ARMED,
    DISARMED,
    ENGAGED,
    FAULTED,
    STOPPED,
    Group,
)
from lerobot_teleoperator_quest.so101 import UrdfMapping

RIGHT = So101State((-0.1, 0.2, 0.3, 0.0, 0.0), 0.5)
RIGHT_MAPPING = UrdfMapping((1, 1, 1, 1, 1), (0, 0, 0, 0, 0), OTHER_CAL)
CALS = {"left": CAL, "right": OTHER_CAL}


def group():
    return Group({"left": controller(), "right": controller(mapping=RIGHT_MAPPING)})


def both(at, left=REST, right=RIGHT):
    return {"left": feedback(at, left), "right": feedback(at, right)}


def inputs(seq, at, left=None, right=None, session=1):
    """Each hand is None when untracked, else the keyword arguments of its sample."""
    hands = {"left": left, "right": right}
    return {
        name: (None if hand is None else sample(seq, at, **hand), session)
        for name, hand in hands.items()
    }


def ready(arms, at=1.0):
    arms.arm(both(at), CALS, at)
    released = {"grip": False}
    return arms.tick(inputs(0, at, released, released), both(at), at)


def test_arming_is_every_arm_or_none():
    arms = group()
    with pytest.raises(RuntimeError, match="fresh measured feedback"):
        arms.arm({"left": feedback(1.0), "right": feedback(0.5, RIGHT)}, CALS, 1.0)
    assert set(arms.states.values()) == {DISARMED}
    with pytest.raises(RuntimeError, match="fresh measured feedback"):
        arms.arm({"left": feedback(1.0)}, CALS, 1.0)
    assert set(arms.states.values()) == {DISARMED}
    with pytest.raises(ValueError, match="calibration does not match"):
        arms.arm(both(1.0), {"left": CAL, "right": CAL}, 1.0)  # the left file for both arms
    assert set(arms.states.values()) == {DISARMED}
    with pytest.raises(ValueError, match="calibration does not match"):
        arms.arm(both(1.0), {"left": CAL}, 1.0)
    assert set(arms.states.values()) == {DISARMED}
    arms.arm(both(1.0), CALS, 1.0)
    assert set(arms.states.values()) == {ARMED}


def test_each_hand_drives_only_its_own_arm():
    arms = group()
    ready(arms)
    steps = None
    for i in range(1, 12):
        at = 1.0 + i * TICK
        hands = inputs(i, at, {"position": (0.05, 0, 0) if i > 1 else (0, 0, 0)}, {"grip": False})
        steps = arms.tick(hands, both(at), at)
    assert steps["left"].state == ENGAGED
    assert steps["left"].command.joints_rad[0] == pytest.approx(0.15)
    assert (steps["right"].state, steps["right"].command) == (ARMED, None)

    # The right hand grips later: anchored at ITS arm's measured state, with ITS mapping.
    steps = arms.tick(inputs(12, 1.12, {"position": (0.05, 0, 0)}, {}), both(1.12), 1.12)
    assert steps["right"].command == RIGHT
    for i in range(13, 24):
        at = 1.0 + i * TICK
        moved = {"position": (0, 0.03, 0)}
        steps = arms.tick(inputs(i, at, {"position": (0.05, 0, 0)}, moved), both(at), at)
    assert steps["right"].command.joints_rad[1] == pytest.approx(0.23)  # sign +1 on the right
    assert steps["left"].command.joints_rad[1] == pytest.approx(0.2)  # the left did not follow

    # Releasing one hand holds that arm; the other keeps tracking.
    lagging = So101State((-0.1, 0.22, 0.3, 0.0, 0.0), 0.5)
    steps = arms.tick(
        inputs(24, 1.24, {"position": (0.05, 0, 0)}, {"grip": False}),
        both(1.24, right=lagging),
        1.24,
    )
    assert (steps["right"].state, steps["right"].command) == (ARMED, lagging)
    assert steps["left"].state == ENGAGED


def test_an_untracked_hand_holds_its_arm_and_must_release_again():
    arms = group()
    ready(arms)
    arms.tick(inputs(1, 1.01, {}, {}), both(1.01), 1.01)
    steps = arms.tick(inputs(2, 1.02, {}, None), both(1.02), 1.02)
    assert steps["left"].state == ENGAGED
    assert (steps["right"].state, steps["right"].reason) == (ARMED, "no fresh input")
    steps = arms.tick(inputs(3, 1.03, {}, {}), both(1.03), 1.03)
    assert steps["right"].reason == "release the grip before pressing it"


def test_a_fault_in_one_arm_stops_the_other_at_its_measured_state():
    arms = group()
    ready(arms)
    for i in range(1, 12):
        at = 1.0 + i * TICK
        moving = {"position": (0.2, 0, 0) if i > 1 else (0, 0, 0)}
        arms.tick(inputs(i, at, moving, moving), both(at), at)
    assert set(arms.states.values()) == {ENGAGED}

    lagging = So101State((0.13, 0.2, 0.3, 0.0, 0.0), 0.5)
    lost = {"left": feedback(1.12, lagging), "right": None}
    moving = {"position": (0.2, 0, 0)}
    steps = arms.tick(inputs(12, 1.12, moving, moving), lost, 1.12)
    assert (steps["right"].state, steps["right"].command) == (FAULTED, None)
    assert (steps["left"].state, steps["left"].command) == (STOPPED, lagging)
    assert steps["left"].reason == "right faulted: measured feedback is stale"

    # Feedback returning changes nothing until an explicit clear and re-arm.
    steps = arms.tick(inputs(13, 1.13, {"grip": False}, {"grip": False}), both(1.13), 1.13)
    assert (steps["left"].state, steps["right"].state) == (STOPPED, FAULTED)
    assert steps["left"].command == lagging
    with pytest.raises(RuntimeError, match="cannot arm"):
        arms.arm(both(1.13), CALS, 1.13)
    arms.clear()
    assert set(arms.states.values()) == {DISARMED}
    steps = ready(arms, at=1.14)
    assert all(step.command is None for step in steps.values())


def test_stop_latches_every_arm():
    arms = group()
    ready(arms)
    for i in range(1, 6):
        at = 1.0 + i * TICK
        arms.tick(
            inputs(i, at, {"position": (0.2, 0, 0) if i > 1 else (0, 0, 0)}, None), both(at), at
        )
    assert arms.states == {"left": ENGAGED, "right": ARMED}
    arms.stop("operator pressed stop")
    lagging = So101State((0.12, 0.2, 0.3, 0.0, 0.0), 0.5)
    steps = arms.tick(inputs(6, 1.06, {}, {}), both(1.06, left=lagging), 1.06)
    assert (steps["left"].state, steps["left"].command) == (STOPPED, lagging)
    assert (steps["right"].state, steps["right"].command) == (STOPPED, None)  # it never moved
    assert steps["right"].reason == "operator pressed stop"
    arms.clear()
    with pytest.raises(RuntimeError, match="no latched stop"):
        arms.clear()


def test_disarm_needs_every_arm_idle():
    arms = group()
    with pytest.raises(RuntimeError, match="cannot disarm"):
        arms.disarm()
    ready(arms)
    arms.tick(inputs(1, 1.01, {}, {"grip": False}), both(1.01), 1.01)
    with pytest.raises(RuntimeError, match="stop instead"):
        arms.disarm()
    assert arms.states == {"left": ENGAGED, "right": ARMED}  # nothing was half-disarmed
    arms.tick(inputs(2, 1.02, {"grip": False}, {}), both(1.02), 1.02)
    with pytest.raises(RuntimeError, match="stop instead"):
        arms.disarm()
    assert arms.states == {"left": ARMED, "right": ENGAGED}  # whichever arm is the busy one
    arms.tick(inputs(3, 1.03, {"grip": False}, {"grip": False}), both(1.03), 1.03)
    arms.disarm()
    assert set(arms.states.values()) == {DISARMED}


def test_a_group_of_one_behaves_like_its_controller():
    arms = Group({"": controller()})
    arms.arm({"": feedback(1.0)}, {"": CAL}, 1.0)
    arms.tick({"": (sample(0, 1.0, grip=False), 1)}, {"": feedback(1.0)}, 1.0)
    steps = arms.tick({"": (sample(1, 1.01), 1)}, {"": feedback(1.01)}, 1.01)
    assert (steps[""].state, steps[""].command) == (ENGAGED, REST)
    assert arms.tick({}, {}, 1.02)[""].state == FAULTED  # no feedback at all


@pytest.mark.parametrize("build", [dict, lambda: {"left": None}, lambda: None])
def test_group_rejects_malformed_membership(build):
    with pytest.raises(ValueError):
        Group(build())


def test_two_arms_cannot_share_one_controller():
    shared = controller()
    with pytest.raises(ValueError, match="its own controller"):
        Group({"left": shared, "right": shared})
