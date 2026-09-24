"""Pure-Python tests for the feeding-test helpers: taught waypoints, hold-to-go
buttons, the fork tilt axis and the stab-then-scoop macro. No ROS needed."""

import math

import pytest

from rammp_teleop.feeding import (
    HoldButton,
    ScoopMacro,
    ScoopParams,
    WaypointStore,
    quat_rotate,
    tilt_axis,
)

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def approx(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def quat_about_z(deg):
    h = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


# ---------------------------------------------------------------- waypoints


def test_waypoint_store_round_trips_through_the_file(tmp_path):
    path = tmp_path / "wp.json"
    store = WaypointStore(str(path))
    assert store.get("food") is None
    store.save("food", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    again = WaypointStore(str(path))
    assert again.get("food") == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]


def test_waypoint_store_creates_parent_dirs(tmp_path):
    path = tmp_path / "deeper" / "wp.json"
    WaypointStore(str(path)).save("mouth", [0.0] * 7)
    assert path.exists()


# ---------------------------------------------------------------- hold button


def test_hold_button_fires_once_after_hold_and_rearms_on_release():
    hb = HoldButton(hold_s=1.0)
    assert hb.update(True, now=10.0) is False
    assert hb.update(True, now=10.5) is False
    assert hb.update(True, now=11.0) is True
    assert hb.update(True, now=12.0) is False  # one firing per press
    assert hb.update(False, now=12.1) is False
    assert hb.update(True, now=13.0) is False  # a new press restarts the timer
    assert hb.update(True, now=14.0) is True


def test_hold_button_reset_clears_the_press():
    hb = HoldButton(hold_s=1.0)
    hb.update(True, now=0.0)
    hb.reset()
    assert hb.update(True, now=5.0) is False  # timer restarted, not fired


# ---------------------------------------------------------------- tilt axis


def test_quat_rotate_identity_and_z_rotation():
    assert approx(quat_rotate(IDENTITY, (1.0, 2.0, 3.0)), (1.0, 2.0, 3.0))
    assert approx(quat_rotate(quat_about_z(90), (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))


def test_tilt_axis_positive_rotation_raises_the_tip():
    # Fork along base +x: the axis is -y, and rotating +x about -y lifts it.
    axis = tilt_axis((1.0, 0.0, 0.0), IDENTITY)
    assert approx(axis, (0.0, -1.0, 0.0))
    # Fork along base +y (tool yawed 90 deg): the axis is +x.
    axis = tilt_axis((1.0, 0.0, 0.0), quat_about_z(90))
    assert approx(axis, (1.0, 0.0, 0.0))


def test_tilt_axis_ignores_the_forks_vertical_component():
    # A fork drooping 30 deg still tilts about the same horizontal axis.
    f = (math.cos(math.radians(30)), 0.0, -math.sin(math.radians(30)))
    assert approx(tilt_axis(f, IDENTITY), (0.0, -1.0, 0.0))


def test_tilt_axis_undefined_for_a_vertical_fork():
    assert tilt_axis((0.0, 0.0, 1.0), IDENTITY) is None


# ---------------------------------------------------------------- scoop macro

PARAMS = ScoopParams(
    depth_m=0.03, tilt_deg=20.0, lift_m=0.05, linear_speed=0.03, angular_speed=0.2
)
AXIS = (0.0, -1.0, 0.0)


def run_macro(macro, dt=0.02):
    steps = []
    while not macro.done:
        steps.append(macro.step(dt))
    return steps


def test_scoop_descends_then_lifts_while_tilting():
    steps = run_macro(ScoopMacro(PARAMS, AXIS))
    dz = sum(lin[2] * 0.02 for lin, _ in steps)
    down = sum(lin[2] * 0.02 for lin, _ in steps if lin[2] < 0)
    up = sum(lin[2] * 0.02 for lin, _ in steps if lin[2] > 0)
    tilt = sum(-ang[1] * 0.02 for _, ang in steps)  # about -y => -ang.y
    assert down == pytest.approx(-0.03, abs=1e-9)
    assert up == pytest.approx(0.05, abs=1e-9)
    assert dz == pytest.approx(0.02, abs=1e-9)
    assert tilt == pytest.approx(math.radians(20.0), abs=1e-9)
    # Nothing tilts during the stab, and the tilt overlaps the lift.
    first_tilt = next(i for i, (_, ang) in enumerate(steps) if ang != (0.0, 0.0, 0.0))
    last_descend = max(i for i, (lin, _) in enumerate(steps) if lin[2] < 0)
    assert first_tilt > last_descend
    assert any(lin[2] > 0 and ang != (0.0, 0.0, 0.0) for lin, ang in steps)
    # Never faster than the configured speeds, and never sideways.
    assert all(
        abs(lin[2]) <= 0.03 + 1e-12 and lin[0] == lin[1] == 0.0 for lin, _ in steps
    )
    assert all(abs(ang[1]) <= 0.2 + 1e-12 for _, ang in steps)


def test_scoop_negative_tilt_tilts_the_other_way():
    p = ScoopParams(
        depth_m=0.0, tilt_deg=-10.0, lift_m=0.0, linear_speed=0.03, angular_speed=0.2
    )
    steps = run_macro(ScoopMacro(p, AXIS))
    tilt = sum(-ang[1] * 0.02 for _, ang in steps)
    assert tilt == pytest.approx(math.radians(-10.0), abs=1e-9)
    assert all(lin == (0.0, 0.0, 0.0) for lin, _ in steps)


def test_scoop_phase_names():
    m = ScoopMacro(PARAMS, AXIS)
    assert m.phase == "stab"
    m.step(1.0)  # past the 1 s stab
    assert m.phase == "scoop"
    m.step(5.0)
    assert m.done and m.phase == "done"
