"""Pure-Python tests for the teleop mapping and integrators. No ROS needed."""
import math

import pytest

from rammp_teleop.logic import (
    CartesianIntegrator,
    GripperIntegrator,
    JointIntegrator,
    XboxMap,
    apply_deadzone,
    quat_angle,
    quat_from_rotvec,
    quat_mul,
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


# ---------------------------------------------------------------- quaternions


def test_quat_from_rotvec_identity():
    assert approx(quat_from_rotvec((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0, 1.0))


def test_quat_from_rotvec_90deg_about_z():
    q = quat_from_rotvec((0.0, 0.0, math.pi / 2))
    s = math.sqrt(0.5)
    assert approx(q, (0.0, 0.0, s, s))


def test_quat_angle_between():
    a = quat_from_rotvec((0.0, 0.0, 0.3))
    b = quat_from_rotvec((0.0, 0.0, -0.2))
    assert quat_angle(a, b) == pytest.approx(0.5)
    assert quat_angle(a, a) == pytest.approx(0.0)


def test_quat_mul_composes_rotations():
    a = quat_from_rotvec((0.0, 0.0, 0.3))
    b = quat_from_rotvec((0.0, 0.0, 0.4))
    assert approx(quat_mul(a, b), quat_from_rotvec((0.0, 0.0, 0.7)))


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
            MAP.axis_left_y: 0.5,   # +x
            MAP.axis_left_x: -1.0,  # -y
            MAP.axis_right_y: 1.0,  # +z
            MAP.axis_right_x: 1.0,  # +yaw
            MAP.axis_dpad_y: 1.0,   # nose up = -pitch
            MAP.axis_dpad_x: -1.0,  # D-pad right = +roll
        }
    )
    v, w = MAP.cartesian_command(axes, deadzone=0.0)
    assert approx(v, (0.5, -1.0, 1.0))
    assert approx(w, (1.0, -1.0, 1.0))


def test_deadman_and_edge_buttons():
    _, buttons = joy(buttons={MAP.button_deadman: 1, MAP.button_estop: 1})
    assert MAP.deadman(buttons)
    assert MAP.pressed(buttons, MAP.button_estop)
    assert not MAP.pressed(buttons, MAP.button_estop_clear)


# ---------------------------------------------------------------- Cartesian integrator


def test_cartesian_integrator_translates_at_max_speed():
    integ = CartesianIntegrator(max_linear=0.1, max_angular=1.0, lead_m=10.0, lead_rad=10.0)
    integ.reset((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    integ.step((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), dt=0.5, actual_p=(0.0, 0.0, 0.0), actual_q=(0.0, 0.0, 0.0, 1.0))
    assert approx(integ.position, (0.05, 0.0, 0.0))


def test_cartesian_integrator_rotates_about_world_z():
    integ = CartesianIntegrator(max_linear=0.1, max_angular=1.0, lead_m=10.0, lead_rad=10.0)
    integ.reset((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    integ.step((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), dt=0.2, actual_p=(0.0, 0.0, 0.0), actual_q=(0.0, 0.0, 0.0, 1.0))
    assert approx(integ.orientation, quat_from_rotvec((0.0, 0.0, 0.2)), tol=1e-9)


def test_cartesian_leash_caps_position_lead():
    integ = CartesianIntegrator(max_linear=1.0, max_angular=1.0, lead_m=0.05, lead_rad=10.0)
    integ.reset((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for _ in range(10):
        integ.step((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), dt=0.1, actual_p=(0.0, 0.0, 0.0), actual_q=(0.0, 0.0, 0.0, 1.0))
    assert approx(integ.position, (0.05, 0.0, 0.0))


def test_cartesian_leash_caps_orientation_lead():
    integ = CartesianIntegrator(max_linear=1.0, max_angular=1.0, lead_m=10.0, lead_rad=0.1)
    integ.reset((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for _ in range(10):
        integ.step((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), dt=0.1, actual_p=(0.0, 0.0, 0.0), actual_q=(0.0, 0.0, 0.0, 1.0))
    assert quat_angle(integ.orientation, (0.0, 0.0, 0.0, 1.0)) == pytest.approx(0.1)


def test_cartesian_orientation_stays_normalized():
    integ = CartesianIntegrator(max_linear=1.0, max_angular=2.0, lead_m=10.0, lead_rad=10.0)
    integ.reset((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for _ in range(500):
        integ.step((0.0, 0.0, 0.0), (0.3, -0.7, 0.2), dt=0.02, actual_p=(0.0, 0.0, 0.0), actual_q=integ.orientation)
    assert math.sqrt(sum(c * c for c in integ.orientation)) == pytest.approx(1.0, abs=1e-9)


# ---------------------------------------------------------------- Joint integrator


def test_joint_integrator_jogs_selected_joint_only():
    integ = JointIntegrator(max_speed=0.5, lead_rad=10.0)
    integ.reset([0.0] * 7)
    integ.select(3)
    integ.step(-1.0, dt=0.1, actual=[0.0] * 7)
    assert integ.positions[3] == pytest.approx(-0.05)
    assert all(p == 0.0 for i, p in enumerate(integ.positions) if i != 3)


def test_joint_integrator_selection_wraps():
    integ = JointIntegrator(max_speed=0.5, lead_rad=10.0)
    integ.select(6)
    integ.select_next(+1)
    assert integ.selected == 0
    integ.select_next(-1)
    assert integ.selected == 6


def test_joint_integrator_leash():
    integ = JointIntegrator(max_speed=1.0, lead_rad=0.1)
    integ.reset([0.0] * 7)
    integ.select(0)
    for _ in range(10):
        integ.step(1.0, dt=0.1, actual=[0.0] * 7)
    assert integ.positions[0] == pytest.approx(0.1)


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
