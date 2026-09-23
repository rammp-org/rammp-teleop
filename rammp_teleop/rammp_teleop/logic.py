"""Pure-Python teleop logic: Xbox mapping, deadzone, joint selection, gripper.

Nothing in this module imports ROS, so it is unit-testable anywhere. The node
(xbox_node.py) owns the ROS plumbing and calls into these classes once per tick.

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

The sticks command VELOCITIES (a base-frame tool twist, or one joint's rate);
the driver's velocity mode integrates them, so nothing here holds a target.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

Vec3 = Tuple[float, float, float]


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


def setpoint_topic(channel: str) -> str:
    """Topic for a driver channel. The driver names channels relative to /setpoint/
    ("pose" -> "/setpoint/pose"); a name that is already a topic is kept."""
    return channel if channel.startswith("/") else f"/setpoint/{channel}"


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

    button_estop: int = 1  # B: engage /estop
    button_estop_clear: int = 7  # Start: clear /estop
    button_resync: int = 3  # Y: re-seed the gripper from measured state
    button_mode: int = 2  # X: toggle translate / rotate

    @staticmethod
    def _axis(axes: Sequence[float], idx: int) -> float:
        return float(axes[idx]) if 0 <= idx < len(axes) else 0.0

    def is_active(
        self, axes: Sequence[float], deadzone: float, rotate: bool = False
    ) -> bool:
        """True while an axis the current mode reads, or a trigger, is deflected past
        the deadzone. The self-centering sticks are the deadman: nothing is commanded
        at rest.
        """
        if not axes:
            return False
        linear, angular = self.cartesian_command(axes, deadzone, rotate)
        close, open_ = self.gripper_command(axes)
        return any(v != 0.0 for v in linear + angular) or close > 0.0 or open_ > 0.0

    @staticmethod
    def pressed(buttons: Sequence[int], idx: int) -> bool:
        return 0 <= idx < len(buttons) and buttons[idx] != 0

    def cartesian_command(
        self, axes: Sequence[float], deadzone: float, rotate: bool = False
    ) -> Tuple[Vec3, Vec3]:
        """Normalized (linear, angular) command in the base frame, each in [-1, 1].

        translate (default):
        left stick Y  -> +x (forward)     right stick X -> +yaw (about +z, left = CCW)
        left stick X  -> +y (left)        D-pad up      -> nose up  (-pitch about +y)
        right stick Y -> +z (up)          D-pad right   -> +roll about +x

        rotate: no translation; the left stick takes over the D-pad's job.
        left stick Y  -> nose up (-pitch)  right stick X -> +yaw
        left stick X  -> roll (left = -roll, like D-pad left)
        """
        a = lambda i: apply_deadzone(self._axis(axes, i), deadzone)  # noqa: E731
        if rotate:
            linear = (0.0, 0.0, 0.0)
            angular = (-a(self.axis_left_x), -a(self.axis_left_y), a(self.axis_right_x))
        else:
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
        return trigger_amount(self._axis(axes, self.axis_rt)), trigger_amount(
            self._axis(axes, self.axis_lt)
        )


# ---------------------------------------------------------------------------
# Joint jog: which joint the left stick drives in joint_velocity mode.
# ---------------------------------------------------------------------------


class JointSelector:
    NUM_JOINTS = 7

    def __init__(self) -> None:
        self.selected = 0

    def select(self, idx: int) -> None:
        self.selected = idx % self.NUM_JOINTS

    def select_next(self, step: int) -> None:
        self.select(self.selected + step)

    def velocities(self, rate: float, max_speed: float) -> List[float]:
        """Seven joint velocities: `rate` (-1..1) times `max_speed` on the selected joint."""
        v = [0.0] * self.NUM_JOINTS
        v[self.selected] = rate * max_speed
        return v


# ---------------------------------------------------------------------------
# Gripper: the one thing still commanded as a position, integrated from the triggers.
# ---------------------------------------------------------------------------


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
