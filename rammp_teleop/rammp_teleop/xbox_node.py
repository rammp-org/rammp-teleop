"""Xbox-controller teleop through the streaming tier.

The self-centering sticks are the deadman: the arm moves while a stick, the
D-pad or a trigger is deflected and holds otherwise. On every deflected -> released
edge (and back) the target is re-seeded from the measured state, so letting go
stops the arm where it is rather than at a leashed target ahead of it.
"""

from __future__ import annotations

import sys
import time
from typing import Optional

from sensor_msgs.msg import Joy

from rammp_teleop.logic import (
    CartesianIntegrator,
    GripperIntegrator,
    JointIntegrator,
    XboxMap,
)
from rammp_teleop.session import TeleopNodeBase, pose_msg, run


class XboxTeleopNode(TeleopNodeBase):
    def __init__(self) -> None:
        super().__init__(
            "xbox_teleop", default_controller="ee_pose_position", default_rate_hz=50.0
        )
        dp = self.declare_parameter
        self.joy_topic: str = dp("joy_topic", "/joy").value
        self.deadzone: float = dp("deadzone", 0.15).value
        self.joy_timeout_s: float = dp("joy_timeout_s", 0.5).value

        max_linear = dp("max_linear_speed", 0.05).value  # m/s at full stick
        max_angular = dp("max_angular_speed", 0.3).value  # rad/s at full stick
        max_joint = dp("max_joint_speed", 0.2).value  # rad/s at full stick
        lead_m = dp("target_lead_m", 0.05).value  # leash; 0 disables
        lead_rad = dp("target_lead_rad", 0.2).value
        joint_lead = dp("joint_target_lead_rad", 0.1).value
        gripper_speed = dp("gripper_speed", 1.0).value  # travel fraction per second

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
        )

        self.cart = CartesianIntegrator(max_linear, max_angular, lead_m, lead_rad)
        self.joints = JointIntegrator(max_joint, joint_lead)
        self.gripper = GripperIntegrator(gripper_speed)

        self._joy: Optional[Joy] = None
        self._joy_rx_time = 0.0
        self._axes: list = []
        self._prev_buttons: list = []
        self._prev_select_step = 0
        self._active_prev = False

        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.get_logger().info(
            "Sticks move the arm. B = e-stop, Start = clear, Y = resync."
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

        active = joy_fresh and self.map.is_active(axes, self.deadzone)
        if active != self._active_prev:
            self.resync("sticks deflected" if active else "sticks released")
            self._active_prev = active
        return active

    def compute_target(self, dt: float, engaged: bool):
        if self.uses_pose:
            ee = self.ee_pose()
            if ee is None:
                return None
            ap, aq = ee
            if engaged:
                lin, ang = self.map.cartesian_command(self._axes, self.deadzone)
                self.cart.step(lin, ang, dt, ap, aq)
            return pose_msg(self.cart.position, self.cart.orientation)

        q = self.joint_positions()
        if q is None:
            return None
        if engaged:
            step = self.map.joint_select_step(self._axes)
            if step != 0 and self._prev_select_step == 0:
                self.joints.select_next(step)
                self.get_logger().info(f"jogging joint_{self.joints.selected + 1}")
            self._prev_select_step = step
            self.joints.step(
                self.map.joint_jog_command(self._axes, self.deadzone), dt, q
            )
        return list(self.joints.positions)

    def gripper_target(self, dt: float, engaged: bool):
        if not engaged:
            return None
        close, open_ = self.map.gripper_command(self._axes)
        return self.gripper.position if self.gripper.step(close, open_, dt) else None

    def seed_from_state(self) -> None:
        ee = self.ee_pose()
        q = self.joint_positions()
        if self.uses_pose and ee is not None:
            self.cart.reset(*ee)
        elif q is not None:
            self.joints.reset(q)
        g = self.gripper_position()
        if g is not None:
            self.gripper.reset(g)


def main(argv=None) -> int:
    return run(XboxTeleopNode, argv)


if __name__ == "__main__":
    sys.exit(main())
