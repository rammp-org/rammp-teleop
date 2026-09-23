"""Xbox-controller teleop through the streaming tier, as velocities.

The sticks command a base-frame tool twist (`ee_twist`) or one joint's rate
(`joint_velocity`). On `ee_twist` the X button toggles between translate mode
(sticks move, D-pad tilts) and rotate mode (sticks turn the tool about the base
axes, no translation); the switch only applies while the sticks are at rest.
The driver's velocity mode integrates the velocities, so there is no
target to run ahead of the arm and nothing to leash. The self-centering sticks
are the deadman: zero velocity is streamed whenever they are at rest. The
gripper is the one position target, integrated from the triggers and re-seeded
from the measured gripper state on every deflected/released edge. Holding Back
for `home_hold_s` sends the arm to `home_joints` through the driver's planner;
the move runs only while Back stays held (release, or any stick input, cancels).
"""

from __future__ import annotations

import sys
import time
from typing import Optional

from sensor_msgs.msg import Joy

from rammp_teleop.logic import GripperIntegrator, JointSelector, XboxMap
from rammp_teleop.session import TeleopNodeBase, run, twist_msg

ZERO3 = (0.0, 0.0, 0.0)


class XboxTeleopNode(TeleopNodeBase):
    def __init__(self) -> None:
        super().__init__(
            "xbox_teleop", default_controller="ee_twist", default_rate_hz=50.0
        )
        if self.controller not in ("ee_twist", "joint_velocity"):
            raise ValueError(
                "xbox_teleop streams velocities; controller must be ee_twist or joint_velocity"
            )
        dp = self.declare_parameter
        self.joy_topic: str = dp("joy_topic", "/joy").value
        self.deadzone: float = dp("deadzone", 0.15).value
        self.joy_timeout_s: float = dp("joy_timeout_s", 0.5).value

        self.max_linear: float = dp("max_linear_speed", 0.05).value  # m/s at full stick
        self.max_angular: float = dp(
            "max_angular_speed", 0.3
        ).value  # rad/s at full stick
        self.max_joint: float = dp("max_joint_speed", 0.2).value  # rad/s at full stick
        gripper_speed = dp("gripper_speed", 1.0).value  # travel fraction per second
        self.home_joints: list = list(dp("home_joints", [0.0] * 7).value)
        self.button_home: int = dp("button_home", 6).value  # Back
        self.home_hold_s: float = dp("home_hold_s", 2.0).value
        self._home_pressed_at = 0.0  # monotonic time Back went down; 0 = up
        self._home_fired = False  # one homing per press

        self.map = XboxMap(
            axis_left_x=dp("axis_left_x", 0).value,
            axis_left_y=dp("axis_left_y", 1).value,
            axis_lt=dp("axis_lt", 2).value,
            axis_right_x=dp("axis_right_x", 3).value,
            axis_right_y=dp("axis_right_y", 4).value,
            axis_rt=dp("axis_rt", 5).value,
            axis_dpad_x=dp("axis_dpad_x", 6).value,
            axis_dpad_y=dp("axis_dpad_y", 7).value,
            button_estop=dp("button_estop", 1).value,
            button_estop_clear=dp("button_estop_clear", 7).value,
            button_resync=dp("button_resync", 3).value,
            button_mode=dp("button_mode", 2).value,
        )
        self.rotate = False  # ee_twist: False = translate mode, True = rotate mode

        self.joints = JointSelector()
        self.gripper = GripperIntegrator(gripper_speed)

        self._joy: Optional[Joy] = None
        self._joy_rx_time = 0.0
        self._axes: list = []
        self._prev_buttons: list = []
        self._prev_select_step = 0
        self._active_prev = False

        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.get_logger().info(
            f"Sticks move the arm. X = translate/rotate, hold Back {self.home_hold_s:.0f}s "
            "= home, B = e-stop, Start = clear, Y = re-seed the gripper."
        )

    def engaged_hint(self) -> str:
        return "move a stick"

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg
        self._joy_rx_time = time.monotonic()

    # ------------------------------------------------------------------ hooks

    def tick_input(self, dt: float) -> bool:
        now = time.monotonic()
        joy_fresh = (
            self._joy is not None and (now - self._joy_rx_time) <= self.joy_timeout_s
        )
        axes = list(self._joy.axes) if joy_fresh else []
        buttons = list(self._joy.buttons) if joy_fresh else []
        prev = self._prev_buttons
        self._prev_buttons = buttons
        self._axes = axes

        def rising(idx: int) -> bool:
            return self.map.pressed(buttons, idx) and not self.map.pressed(prev, idx)

        if rising(self.map.button_estop):
            self.publish_estop(True, "xbox B button")
        if rising(self.map.button_estop_clear):
            self.publish_estop(False, "xbox Start button")
        if rising(self.map.button_resync):
            self.resync("Y button")
        if self.map.pressed(buttons, self.button_home):
            if self._home_pressed_at == 0.0:
                self._home_pressed_at = now
                self._home_fired = False
                self.get_logger().info(f"hold Back {self.home_hold_s:.1f}s to go home")
            elif (
                not self._home_fired and now - self._home_pressed_at >= self.home_hold_s
            ):
                self._home_fired = True
                if self._active_prev:
                    self.get_logger().warn("release the sticks to go home")
                elif len(self.home_joints) != 7:
                    self.get_logger().warn("home_joints must hold 7 values; not homing")
                else:
                    self.go_home(self.home_joints)
        else:
            self._home_pressed_at = 0.0
            self.cancel_home("Back released")
        if rising(self.map.button_mode) and self.uses_twist:
            if self._active_prev:
                self.get_logger().warn("release the sticks to change mode")
            else:
                self.rotate = not self.rotate
                self.get_logger().info(
                    f"mode: {'rotate' if self.rotate else 'translate'}"
                )

        active = joy_fresh and self.map.is_active(axes, self.deadzone, self.rotate)
        if active != self._active_prev:
            self.resync("sticks deflected" if active else "sticks released")
            self._active_prev = active
        return active

    def compute_target(self, dt: float, engaged: bool):
        if self.uses_twist:
            lin, ang = (
                self.map.cartesian_command(self._axes, self.deadzone, self.rotate)
                if engaged
                else (ZERO3, ZERO3)
            )
            return twist_msg(
                [v * self.max_linear for v in lin], [v * self.max_angular for v in ang]
            )

        rate = 0.0
        if engaged:
            step = self.map.joint_select_step(self._axes)
            if step != 0 and self._prev_select_step == 0:
                self.joints.select_next(step)
                self.get_logger().info(f"jogging joint_{self.joints.selected + 1}")
            self._prev_select_step = step
            rate = self.map.joint_jog_command(self._axes, self.deadzone)
        return self.joints.velocities(rate, self.max_joint)

    def gripper_target(self, dt: float, engaged: bool):
        if not engaged:
            return None
        close, open_ = self.map.gripper_command(self._axes)
        return self.gripper.position if self.gripper.step(close, open_, dt) else None

    def seed_from_state(self) -> None:
        g = self.gripper_position()
        if g is not None:
            self.gripper.reset(g)


def main(argv=None) -> int:
    return run(XboxTeleopNode, argv)


if __name__ == "__main__":
    sys.exit(main())
