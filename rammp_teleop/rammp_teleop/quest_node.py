"""Quest 3 controller teleop through the streaming tier.

Squeeze the grip to clutch in: the controller's motion from that moment is
applied to the EE pose from that moment (no absolute calibration; R_align is the
one rotation knob). Release to freeze. Index trigger drives the gripper. B
engages /estop; A re-captures the references.

Default controller is ee_pose_impedance, the same joint-impedance-with-IK law the
original UDP setup ran with --joint-impedance.
"""

from __future__ import annotations

import sys
import time
from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy

from rammp_teleop.logic import XboxMap
from rammp_teleop.quest.command import QuestCommand
from rammp_teleop.quest.mapping import MappingConfig
from rammp_teleop.quest.safety import SafetyConfig
from rammp_teleop.quest.transforms import make_pose
from rammp_teleop.session import TeleopNodeBase, pose_msg, run


def _euler_zyx(deg) -> Rotation:
    return Rotation.from_euler("ZYX", [float(v) for v in deg], degrees=True)


class QuestTeleopNode(TeleopNodeBase):
    def __init__(self) -> None:
        super().__init__(
            "quest_teleop", default_controller="ee_pose_impedance", default_rate_hz=60.0
        )
        if not self.uses_pose:
            raise ValueError(
                "quest_teleop streams EE poses; controller must be ee_pose_position or ee_pose_impedance"
            )
        dp = self.declare_parameter
        self.hand: str = dp("hand", "right").value
        self.input_timeout_s: float = dp("input_timeout_s", 0.5).value
        self.button_grip: int = dp("button_grip", 0).value
        self.button_resync: int = dp("button_resync", 1).value  # A
        self.button_estop: int = dp("button_estop", 2).value  # B
        self.axis_trigger: int = dp("axis_trigger", 0).value

        mapping = MappingConfig(
            R_align=_euler_zyx(dp("r_align_euler_zyx_deg", [0.0, 0.0, 0.0]).value),
            trans_smooth=dp("trans_smooth", 0.8).value,
            rot_smooth=dp("rot_smooth", 0.8).value,
            jump_pos_tol=dp("jump_pos_tol", 0.05).value,
            jump_rot_tol=dp("jump_rot_tol", 0.5).value,
            pos_scale=dp("pos_scale", 1.0).value,
            rot_frame=dp("rot_frame", "base").value,
            R_tool=_euler_zyx(dp("r_tool_euler_zyx_deg", [0.0, 0.0, 0.0]).value),
        )
        safety = SafetyConfig(
            ws_min=np.array(dp("ws_min", [0.2, -0.4, 0.05]).value, float),
            ws_max=np.array(dp("ws_max", [0.75, 0.4, 0.7]).value, float),
            max_lin_step=dp("max_lin_step", 0.01).value,
            max_ang_step=dp("max_ang_step", 0.05).value,
        )
        self.cmd = QuestCommand(
            mapping,
            safety,
            gripper_binary=dp("gripper_binary", False).value,
            gripper_threshold=dp("gripper_threshold", 0.5).value,
        )

        self._ctrl_pose: Optional[np.ndarray] = None
        self._pose_rx_time = 0.0
        self._joy: Optional[Joy] = None
        self._joy_rx_time = 0.0
        self._prev_buttons: list = []
        self._trigger = 0.0
        self._last_gripper_sent: Optional[float] = None

        self.create_subscription(
            PoseStamped, f"/quest/{self.hand}/pose", self._on_pose, 10
        )
        self.create_subscription(Joy, "/quest/joy", self._on_joy, 10)
        self.get_logger().info(
            "Squeeze GRIP to move. Trigger = gripper, B = e-stop, A = resync."
        )

    def engaged_hint(self) -> str:
        return "squeeze the grip"

    def _on_pose(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        self._ctrl_pose = make_pose(
            [p.x, p.y, p.z], Rotation.from_quat([o.x, o.y, o.z, o.w])
        )
        self._pose_rx_time = time.monotonic()

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg
        self._joy_rx_time = time.monotonic()

    # ------------------------------------------------------------------ hooks

    def tick_input(self, dt: float) -> bool:
        now = time.monotonic()
        fresh = (
            self._ctrl_pose is not None
            and self._joy is not None
            and now - self._pose_rx_time <= self.input_timeout_s
            and now - self._joy_rx_time <= self.input_timeout_s
        )
        buttons = list(self._joy.buttons) if fresh else []
        axes = list(self._joy.axes) if fresh else []
        prev = self._prev_buttons
        self._prev_buttons = buttons

        def rising(idx: int) -> bool:
            return XboxMap.pressed(buttons, idx) and not XboxMap.pressed(prev, idx)

        if rising(self.button_estop):
            self.publish_estop(True, "quest B button")
        if rising(self.button_resync):
            self.resync("quest A button")

        self._trigger = (
            float(axes[self.axis_trigger])
            if 0 <= self.axis_trigger < len(axes)
            else 0.0
        )
        return fresh and XboxMap.pressed(buttons, self.button_grip)

    def compute_target(self, dt: float, engaged: bool):
        ee = self.ee_pose()
        if ee is None or self._ctrl_pose is None:
            return None
        ee_pos = np.asarray(ee[0], float)
        ee_rot = Rotation.from_quat(ee[1])
        pos, rot, _ = self.cmd.update(self._ctrl_pose, engaged, ee_pos, ee_rot)
        return pose_msg(pos, rot.as_quat())

    def gripper_target(self, dt: float, engaged: bool):
        if not engaged:
            return None
        g = self.cmd.gripper(self._trigger)
        if (
            self._last_gripper_sent is not None
            and abs(g - self._last_gripper_sent) < 1e-3
        ):
            return None
        self._last_gripper_sent = g
        return g

    def seed_from_state(self) -> None:
        # The mapper captures the EE reference itself on the next grip rising
        # edge; all we do is forget the freeze target and the old references.
        self.cmd.resync()
        self._last_gripper_sent = None


def main(argv=None) -> int:
    return run(QuestTeleopNode, argv)


if __name__ == "__main__":
    sys.exit(main())
