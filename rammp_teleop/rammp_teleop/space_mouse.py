#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import time
from typing import Optional, Sequence, Tuple

from sensor_msgs.msg import Joy

from rammp_teleop.logic import apply_deadzone
from rammp_teleop.session import Target, TeleopNodeBase, run, twist_msg

Vec3 = Tuple[float, float, float]
ZERO3: Vec3 = (0.0, 0.0, 0.0)


def space_twist(
    axes: Sequence[float],
    deadzone: float,
    signs: Sequence[float] = (1.0,) * 6,
    expo: float = 1.0,
) -> Tuple[Vec3, Vec3]:
    twist = []
    for i in range(6):
        v = apply_deadzone(float(axes[i]) if i < len(axes) else 0.0, deadzone)
        v *= float(signs[i]) if i < len(signs) else 1.0
        twist.append(math.copysign(abs(v) ** expo, v) if v else 0.0)
    return (twist[0], twist[1], twist[2]), (twist[3], twist[4], twist[5])


def button_held(buttons: Sequence[int], idx: int) -> bool:
    return 0 <= idx < len(buttons) and buttons[idx] != 0


class SpaceTeleopNode(TeleopNodeBase):
    def __init__(self) -> None:
        super().__init__(
            "space_teleop", default_controller="ee_twist", default_rate_hz=50.0
        )
        dp = self.declare_parameter
        self.joy_topic: str = dp("joy_topic", "/spacenav/joy").value
        self.deadzone: float = dp("deadzone", 0.08).value
        self.joy_timeout_s: float = dp("joy_timeout_s", 0.3).value

        self.max_linear: float = dp("max_linear_speed", 0.2).value  # m/s
        self.max_angular: float = dp("max_angular_speed", 0.3).value  # rad/s
        self.signs: list = [float(v) for v in dp("axis_signs", [1.0] * 6).value]
        self.expo: float = dp("expo", 2.0).value  # 1.0 = linear
        self.button_close: int = dp("button_close", 0).value
        self.button_open: int = dp("button_open", 1).value

        self.grip_hold: Optional[float] = None
        self._last_gripper_sent: Optional[float] = None

        # to allow for changing orientation of how space mouse is used
        if len(self.signs) != 6:
            raise ValueError("axis_signs need six values")

        self.joy: Optional[Joy] = None
        self.joy_rx_time = 0.0
        self._lin: Vec3 = ZERO3
        self._ang: Vec3 = ZERO3
        self._grip_close = 0.0
        self._grip_open = 0.0
        self._active_prev = False

        self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)
        self.get_logger().info(
            f"Puck controls - max linear vel: {self.max_linear:.3f} m/s, max angular: {self.max_angular:.2f} rad/s; "
            "Left closes gripper, right opens"
        )

    def on_joy(self, msg: Joy) -> None:
        self.joy = msg
        self.joy_rx_time = time.monotonic()

    # HOOKS
    def engaged_hint(self) -> str:
        return "move puck"

    def tick_input(self, dt: float) -> bool:
        fresh = (
            self.joy is not None
            and (time.monotonic() - self.joy_rx_time) <= self.joy_timeout_s
        )
        if not fresh and self._active_prev:
            self.get_logger().warn(
                f"no {self.joy_topic} for {self.joy_timeout_s:.1f}s -> streaming zero twist",
                throttle_duration_sec=2.0,
            )

        if fresh:
            self._lin, self._ang = space_twist(
                self.joy.axes, self.deadzone, self.signs, self.expo
            )
            buttons = self.joy.buttons
            self._grip_close = 1.0 if button_held(buttons, self.button_close) else 0.0
            self._grip_open = 1.0 if button_held(buttons, self.button_open) else 0.0

        else:
            self._lin, self._ang = ZERO3, ZERO3
            self._grip_close, self._grip_open = 0.0, 0.0

        active = (
            any(v != 0.0 for v in self._lin + self._ang)
            or self._grip_close > 0.0
            or self._grip_open > 0.0
        )

        if active != self._active_prev:
            self.resync("puck deflected" if active else "puck released")
            self._active_prev = active
        return active

    def compute_target(self, dt: float, engaged: bool) -> Optional[Target]:
        lin, ang = (self._lin, self._ang) if engaged else (ZERO3, ZERO3)
        return twist_msg(
            [v * self.max_linear for v in lin], [v * self.max_angular for v in ang]
        )

    def gripper_target(self, dt: float, engaged: bool) -> Optional[float]:
        if self._grip_close > 0.0:
            self.grip_hold = None
            g = 1.0
        elif self._grip_open > 0.0:
            self.grip_hold = None
            g = 0.0
        else:
            if self.grip_hold is None:
                self.grip_hold = self.gripper_position()
            g = self.grip_hold
        # send on change only, not every tick
        if g is None or g == self._last_gripper_sent:
            return None
        self._last_gripper_sent = g
        return g

    def seed_from_state(self) -> None:
        self.grip_hold = self.gripper_position()


def main(argv=None) -> int:
    return run(SpaceTeleopNode, argv)


if __name__ == "__main__":
    sys.exit(main())
