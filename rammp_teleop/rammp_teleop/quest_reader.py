"""Republish a Meta Quest controller as ROS topics.

  /quest/<hand>/pose   geometry_msgs/PoseStamped   raw controller pose, headset tracking frame
  /quest/joy           sensor_msgs/Joy             axes=[index trigger 0..1], buttons=[grip, A, B]

Nothing is remapped here: the teleop node's R_align is the one calibration knob.
With ``mock:=true`` a scripted controller (engage, move, grip, release) loops
forever, so the whole pipeline runs with no headset and no adb.

The ROS imports live inside ``main`` so ``encode_sample`` stays testable on a
host without rclpy.
"""
from __future__ import annotations

import sys

import numpy as np
from scipy.spatial.transform import Rotation

from rammp_teleop.quest.pose_source import Buttons, MockPoseSource, default_script


def encode_sample(pose: np.ndarray, btn: Buttons, hand: str):
    """(position, quat_xyzw), Joy axes, Joy buttons for one controller sample."""
    pos = tuple(float(v) for v in pose[:3, 3])
    quat = tuple(float(v) for v in Rotation.from_matrix(pose[:3, :3]).as_quat())
    a_key, b_key = ("A", "B") if hand == "right" else ("X", "Y")
    buttons = [
        int(bool(btn.grip)),
        int(bool(btn.extra.get(a_key, False))),
        int(bool(btn.extra.get(b_key, False))),
    ]
    return (pos, quat), [float(btn.trigger)], buttons


def _build_source(hand: str, quest_ip: str, mock: bool):
    if mock:
        return MockPoseSource(default_script())
    from rammp_teleop.quest.pose_source import OculusPoseSource  # imports oculus_reader (needs adb)

    return OculusPoseSource(hand="r" if hand == "right" else "l", ip_address=quest_ip or None)


def main(argv=None) -> int:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Joy

    class QuestReaderNode(Node):
        def __init__(self) -> None:
            super().__init__("quest_reader")
            dp = self.declare_parameter
            self.hand: str = dp("hand", "right").value
            self.quest_ip: str = dp("quest_ip", "").value
            self.rate_hz: float = dp("rate_hz", 60.0).value
            self.mock: bool = dp("mock", False).value
            self.frame_id: str = dp("frame_id", "quest").value
            if self.hand not in ("right", "left"):
                raise ValueError(f"hand must be 'right' or 'left', got {self.hand!r}")
            self.source = _build_source(self.hand, self.quest_ip, self.mock)
            self._pose_pub = self.create_publisher(PoseStamped, f"/quest/{self.hand}/pose", 10)
            self._joy_pub = self.create_publisher(Joy, "/quest/joy", 10)
            self.create_timer(1.0 / self.rate_hz, self._tick)
            self.get_logger().info(
                f"quest_reader: {'MOCK scripted controller' if self.mock else 'Quest over adb'}, "
                f"hand={self.hand}, {self.rate_hz:.0f} Hz"
            )

        def _tick(self) -> None:
            if self.mock and self.source.done:
                self.source.reset()
            pose, btn = self.source.read()
            (pos, quat), axes, buttons = encode_sample(pose, btn, self.hand)
            stamp = self.get_clock().now().to_msg()

            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.frame_id
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = quat
            self._pose_pub.publish(ps)

            joy = Joy()
            joy.header.stamp = stamp
            joy.header.frame_id = self.frame_id
            joy.axes = axes
            joy.buttons = buttons
            self._joy_pub.publish(joy)

        def close(self) -> None:
            self.source.close()

    rclpy.init(args=argv)
    node = QuestReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
