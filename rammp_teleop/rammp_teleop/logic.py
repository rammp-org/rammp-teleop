"""Pure-Python teleop logic: Xbox mapping, deadzone, quaternions, integrators.

Nothing in this module imports ROS, so it is unit-testable anywhere. The node
(teleop_node.py) owns the ROS plumbing and calls into these classes once per tick.

Convention for `sensor_msgs/Joy` under ros-humble-joy (SDL backend) with an Xbox
Series X controller over USB, as sampled on abra 2026-09-16:

    axes[0] left stick X   (+1 = left)      buttons[0] A     buttons[6]  Back
    axes[1] left stick Y   (+1 = up)        buttons[1] B     buttons[7]  Start
    axes[2] LT             (+1 = released)  buttons[2] X     buttons[8]  Guide
    axes[3] right stick X  (+1 = left)      buttons[3] Y     buttons[9]  L-stick
    axes[4] right stick Y  (+1 = up)        buttons[4] LB    buttons[10] R-stick
    axes[5] RT             (+1 = released)  buttons[5] RB    buttons[11] Share
    axes[6] D-pad X        (+1 = left)
    axes[7] D-pad Y        (+1 = up)

Quaternions are (x, y, z, w) tuples, matching geometry_msgs/Quaternion.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


# ---------------------------------------------------------------------------
# Scalar helpers
# ---------------------------------------------------------------------------


def apply_deadzone(value: float, deadzone: float) -> float:
    """Zero inside the deadzone, then rescale so full deflection is still +/-1."""
    if abs(value) <= deadzone:
        return 0.0
    if deadzone >= 1.0:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def trigger_amount(axis_value: float) -> float:
    """Trigger axes rest at +1 and go to -1 when fully pressed. Return 0..1 pressed."""
    return min(1.0, max(0.0, (1.0 - axis_value) / 2.0))


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


# ---------------------------------------------------------------------------
# Quaternion helpers (x, y, z, w)
# ---------------------------------------------------------------------------


def quat_normalize(q: Quat) -> Quat:
    n = math.sqrt(sum(c * c for c in q))
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_mul(a: Quat, b: Quat) -> Quat:
    """Hamilton product a*b: apply b first, then a (for world-frame increments)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def quat_from_rotvec(rv: Vec3) -> Quat:
    angle = math.sqrt(rv[0] ** 2 + rv[1] ** 2 + rv[2] ** 2)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2.0) / angle
    return (rv[0] * s, rv[1] * s, rv[2] * s, math.cos(angle / 2.0))


def quat_to_rotvec(q: Quat) -> Vec3:
    q = quat_normalize(q)
    if q[3] < 0:  # shortest path
        q = (-q[0], -q[1], -q[2], -q[3])
    vn = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2)
    if vn < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vn, q[3])
    return (q[0] / vn * angle, q[1] / vn * angle, q[2] / vn * angle)


def quat_angle(a: Quat, b: Quat) -> float:
    """Rotation angle (rad) taking a to b, shortest path."""
    rel = quat_mul(b, quat_conj(a))
    rv = quat_to_rotvec(rel)
    return math.sqrt(rv[0] ** 2 + rv[1] ** 2 + rv[2] ** 2)


# ---------------------------------------------------------------------------
# Xbox mapping
# ---------------------------------------------------------------------------


@dataclass
class XboxMap:
    """Axis/button indices and the stick -> command mapping."""

    axis_left_x: int = 0
    axis_left_y: int = 1
    axis_lt: int = 2
    axis_right_x: int = 3
    axis_right_y: int = 4
    axis_rt: int = 5
    axis_dpad_x: int = 6
    axis_dpad_y: int = 7

    button_estop: int = 1         # B: engage /estop
    button_estop_clear: int = 7   # Start: clear /estop
    button_resync: int = 3        # Y: snap target back to measured state

    @staticmethod
    def _axis(axes: Sequence[float], idx: int) -> float:
        return float(axes[idx]) if 0 <= idx < len(axes) else 0.0

    def is_active(self, axes: Sequence[float], deadzone: float) -> bool:
        """True while any stick, D-pad direction or trigger is deflected past the deadzone.

        The self-centering sticks are the deadman: nothing is commanded at rest.
        """
        if not axes:
            return False
        linear, angular = self.cartesian_command(axes, deadzone)
        close, open_ = self.gripper_command(axes)
        return any(v != 0.0 for v in linear + angular) or close > 0.0 or open_ > 0.0

    @staticmethod
    def pressed(buttons: Sequence[int], idx: int) -> bool:
        return 0 <= idx < len(buttons) and buttons[idx] != 0

    def cartesian_command(self, axes: Sequence[float], deadzone: float) -> Tuple[Vec3, Vec3]:
        """Normalized (linear, angular) command in the base frame, each in [-1, 1].

        left stick Y  -> +x (forward)     right stick X -> +yaw (about +z, left = CCW)
        left stick X  -> +y (left)        D-pad up      -> nose up  (-pitch about +y)
        right stick Y -> +z (up)          D-pad right   -> +roll about +x
        """
        a = lambda i: apply_deadzone(self._axis(axes, i), deadzone)  # noqa: E731
        linear = (a(self.axis_left_y), a(self.axis_left_x), a(self.axis_right_y))
        angular = (-a(self.axis_dpad_x), -a(self.axis_dpad_y), a(self.axis_right_x))
        return linear, angular

    def joint_jog_command(self, axes: Sequence[float], deadzone: float) -> float:
        """Normalized jog rate for the selected joint: left stick Y."""
        return apply_deadzone(self._axis(axes, self.axis_left_y), deadzone)

    def joint_select_step(self, axes: Sequence[float]) -> int:
        """+1 / -1 / 0 from the D-pad X axis (right = next joint)."""
        v = self._axis(axes, self.axis_dpad_x)
        if v < -0.5:
            return +1
        if v > 0.5:
            return -1
        return 0

    def gripper_command(self, axes: Sequence[float]) -> Tuple[float, float]:
        """(close, open) amounts in 0..1 from RT and LT."""
        return trigger_amount(self._axis(axes, self.axis_rt)), trigger_amount(self._axis(axes, self.axis_lt))


# ---------------------------------------------------------------------------
# Integrators. Each holds an absolute target that the node streams every tick.
# `lead_*` is a leash: the target may never run further ahead of the measured
# state than this, so a lagging arm does not keep chasing a runaway target
# after the stick is released.
# ---------------------------------------------------------------------------


class CartesianIntegrator:
    def __init__(self, max_linear: float, max_angular: float, lead_m: float, lead_rad: float):
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.lead_m = lead_m
        self.lead_rad = lead_rad
        self.position: Vec3 = (0.0, 0.0, 0.0)
        self.orientation: Quat = (0.0, 0.0, 0.0, 1.0)

    def reset(self, position: Vec3, orientation: Quat) -> None:
        self.position = tuple(position)  # type: ignore[assignment]
        self.orientation = quat_normalize(tuple(orientation))  # type: ignore[arg-type]

    def step(self, linear: Vec3, angular: Vec3, dt: float, actual_p: Vec3, actual_q: Quat) -> None:
        p = [self.position[i] + linear[i] * self.max_linear * dt for i in range(3)]
        dq = quat_from_rotvec(tuple(angular[i] * self.max_angular * dt for i in range(3)))  # type: ignore[arg-type]
        q = quat_normalize(quat_mul(dq, self.orientation))  # world-frame increment

        # Leash the position.
        d = [p[i] - actual_p[i] for i in range(3)]
        dist = math.sqrt(sum(c * c for c in d))
        if dist > self.lead_m > 0.0:
            s = self.lead_m / dist
            p = [actual_p[i] + d[i] * s for i in range(3)]

        # Leash the orientation: scale the relative rotation actual -> target.
        rel = quat_mul(q, quat_conj(actual_q))
        rv = quat_to_rotvec(rel)
        ang = math.sqrt(sum(c * c for c in rv))
        if ang > self.lead_rad > 0.0:
            s = self.lead_rad / ang
            rel = quat_from_rotvec((rv[0] * s, rv[1] * s, rv[2] * s))
            q = quat_normalize(quat_mul(rel, actual_q))

        self.position = (p[0], p[1], p[2])
        self.orientation = q


class JointIntegrator:
    NUM_JOINTS = 7

    def __init__(self, max_speed: float, lead_rad: float):
        self.max_speed = max_speed
        self.lead_rad = lead_rad
        self.positions: List[float] = [0.0] * self.NUM_JOINTS
        self.selected = 0

    def reset(self, positions: Sequence[float]) -> None:
        self.positions = [float(v) for v in positions[: self.NUM_JOINTS]]

    def select(self, idx: int) -> None:
        self.selected = idx % self.NUM_JOINTS

    def select_next(self, step: int) -> None:
        self.select(self.selected + step)

    def step(self, rate: float, dt: float, actual: Sequence[float]) -> None:
        j = self.selected
        target = self.positions[j] + rate * self.max_speed * dt
        if self.lead_rad > 0.0:
            target = clamp(target, actual[j] - self.lead_rad, actual[j] + self.lead_rad)
        self.positions[j] = target


class GripperIntegrator:
    def __init__(self, speed: float):
        self.speed = speed  # fraction of full travel per second
        self.position = 0.0  # 0 open .. 1 closed

    def reset(self, position: float) -> None:
        self.position = clamp(float(position), 0.0, 1.0)

    def step(self, close: float, open_: float, dt: float) -> bool:
        """Integrate; return True if the target changed and should be sent."""
        new = clamp(self.position + (close - open_) * self.speed * dt, 0.0, 1.0)
        changed = abs(new - self.position) > 1e-9
        self.position = new
        return changed
