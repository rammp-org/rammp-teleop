"""Pure-Python tests for the teleop mapping and integrators. No ROS needed."""

import pytest

from rammp_teleop.logic import (
    GripperIntegrator,
    JointSelector,
    XboxMap,
    apply_deadzone,
    setpoint_topic,
    trigger_amount,
)


def approx(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


# ---------------------------------------------------------------- deadzone


def test_deadzone_zeroes_small_inputs():
    assert apply_deadzone(0.05, 0.1) == 0.0
    assert apply_deadzone(-0.09, 0.1) == 0.0


def test_deadzone_rescales_to_full_range():
    assert apply_deadzone(1.0, 0.1) == pytest.approx(1.0)
    assert apply_deadzone(-1.0, 0.1) == pytest.approx(-1.0)
    assert apply_deadzone(0.55, 0.1) == pytest.approx(0.5)


def test_trigger_amount_maps_rest_to_zero_and_pressed_to_one():
    assert trigger_amount(1.0) == pytest.approx(0.0)
    assert trigger_amount(-1.0) == pytest.approx(1.0)
    assert trigger_amount(0.0) == pytest.approx(0.5)


# ---------------------------------------------------------------- Xbox mapping

MAP = XboxMap()  # defaults are the Xbox Series X layout under ros-humble-joy


def joy(axes=None, buttons=None):
    a = [0.0] * 8
    a[MAP.axis_lt] = 1.0
    a[MAP.axis_rt] = 1.0
    b = [0] * 12
    for i, v in (axes or {}).items():
        a[i] = v
    for i, v in (buttons or {}).items():
        b[i] = v
    return a, b


def test_cartesian_command_full_forward_stick():
    axes, buttons = joy({MAP.axis_left_y: 1.0})
    v, w = MAP.cartesian_command(axes, deadzone=0.1)
    assert approx(v, (1.0, 0.0, 0.0))
    assert approx(w, (0.0, 0.0, 0.0))


def test_cartesian_command_all_six_dof():
    axes, _ = joy(
        {
            MAP.axis_left_y: 0.5,  # +x
            MAP.axis_left_x: -1.0,  # -y
            MAP.axis_right_y: 1.0,  # +z
            MAP.axis_right_x: 1.0,  # +yaw
            MAP.axis_dpad_y: 1.0,  # nose up = -pitch
            MAP.axis_dpad_x: -1.0,  # D-pad right = +roll
        }
    )
    v, w = MAP.cartesian_command(axes, deadzone=0.0)
    assert approx(v, (0.5, -1.0, 1.0))
    assert approx(w, (1.0, -1.0, 1.0))


def test_cartesian_command_rotate_mode_sticks_turn_and_never_translate():
    # left stick up -> nose up (-pitch), left stick left -> -roll, right stick left -> +yaw
    axes, _ = joy({MAP.axis_left_y: 1.0, MAP.axis_left_x: 1.0, MAP.axis_right_x: 1.0})
    v, w = MAP.cartesian_command(axes, 0.0, rotate=True)
    assert v == (0.0, 0.0, 0.0)
    assert approx(w, (-1.0, -1.0, 1.0))
    # right stick Y and the D-pad do nothing in rotate mode
    axes, _ = joy({MAP.axis_right_y: 1.0, MAP.axis_dpad_x: 1.0, MAP.axis_dpad_y: 1.0})
    v, w = MAP.cartesian_command(axes, 0.0, rotate=True)
    assert v == (0.0, 0.0, 0.0) and w == (0.0, 0.0, 0.0)


def test_is_active_follows_the_mode():
    right_y, _ = joy({MAP.axis_right_y: 1.0})
    assert MAP.is_active(right_y, 0.15)
    assert not MAP.is_active(right_y, 0.15, rotate=True)
    assert MAP.is_active(joy({MAP.axis_left_x: 1.0})[0], 0.15, rotate=True)


def test_edge_buttons():
    _, buttons = joy(buttons={MAP.button_estop: 1})
    assert MAP.pressed(buttons, MAP.button_estop)
    assert not MAP.pressed(buttons, MAP.button_estop_clear)


def test_is_active_false_at_rest_including_released_triggers():
    axes, _ = joy()
    assert not MAP.is_active(axes, deadzone=0.15)


def test_is_active_ignores_deflection_inside_deadzone():
    axes, _ = joy({MAP.axis_left_x: 0.1, MAP.axis_right_y: -0.1})
    assert not MAP.is_active(axes, deadzone=0.15)


def test_is_active_on_stick_dpad_or_trigger():
    axes, _ = joy({MAP.axis_left_y: 0.5})
    assert MAP.is_active(axes, deadzone=0.15)
    axes, _ = joy({MAP.axis_dpad_x: -1.0})
    assert MAP.is_active(axes, deadzone=0.15)
    axes, _ = joy({MAP.axis_rt: 0.0})  # trigger half pressed (rest is +1)
    assert MAP.is_active(axes, deadzone=0.15)


def test_is_active_false_when_no_joy_at_all():
    assert not MAP.is_active([], deadzone=0.15)


# ---------------------------------------------------------------- Joint selector


def test_joint_selector_velocities_selected_joint_only():
    s = JointSelector()
    s.select(2)
    v = s.velocities(0.5, max_speed=0.2)
    assert len(v) == 7
    assert v[2] == pytest.approx(0.1)
    assert all(x == 0.0 for i, x in enumerate(v) if i != 2)


def test_joint_selector_zero_rate_is_all_zeros():
    assert JointSelector().velocities(0.0, max_speed=0.2) == [0.0] * 7


def test_joint_selector_wraps():
    s = JointSelector()
    s.select(6)
    s.select_next(+1)
    assert s.selected == 0
    s.select_next(-1)
    assert s.selected == 6


# ---------------------------------------------------------------- Gripper integrator


def test_gripper_integrator_closes_opens_and_clamps():
    g = GripperIntegrator(speed=1.0)
    g.reset(0.5)
    assert g.step(close=1.0, open_=0.0, dt=0.2) is True
    assert g.position == pytest.approx(0.7)
    g.step(close=0.0, open_=1.0, dt=5.0)
    assert g.position == pytest.approx(0.0)
    assert g.step(close=0.0, open_=0.0, dt=0.2) is False  # no change, nothing to send
    g.step(close=1.0, open_=0.0, dt=5.0)
    assert g.position == pytest.approx(1.0)


# ---------------------------------------------------------------- setpoint topic


def test_setpoint_topic_prefixes_driver_channel_names():
    # The driver names channels relative to /setpoint/ ("pose"), not as topics.
    assert setpoint_topic("pose") == "/setpoint/pose"
    assert setpoint_topic("joint_position") == "/setpoint/joint_position"
    assert setpoint_topic("/setpoint/pose") == "/setpoint/pose"
