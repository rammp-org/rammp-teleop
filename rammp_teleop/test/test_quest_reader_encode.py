"""encode_sample is the only logic in quest_reader; it must not need ROS to test."""

import numpy as np
from scipy.spatial.transform import Rotation

from rammp_teleop.quest.pose_source import Buttons
from rammp_teleop.quest.transforms import make_pose
from rammp_teleop.quest_reader import encode_sample


def test_encode_pose_is_xyzw_and_position():
    rot = Rotation.from_euler("z", 90, degrees=True)
    pose = make_pose([0.1, 0.2, 0.3], rot)
    (pos, quat), axes, buttons = encode_sample(
        pose, Buttons(grip=True, trigger=0.4), "right"
    )
    assert np.allclose(pos, [0.1, 0.2, 0.3])
    assert np.allclose(quat, rot.as_quat())  # scipy order is x, y, z, w
    assert axes == [0.4]
    assert buttons == [1, 0, 0]


def test_encode_right_hand_face_buttons():
    pose = np.eye(4)
    _, _, buttons = encode_sample(
        pose, Buttons(grip=False, trigger=0.0, extra={"A": True, "B": False}), "right"
    )
    assert buttons == [0, 1, 0]


def test_encode_left_hand_uses_x_and_y():
    pose = np.eye(4)
    _, _, buttons = encode_sample(
        pose, Buttons(grip=False, trigger=0.0, extra={"X": True, "Y": True}), "left"
    )
    assert buttons == [0, 1, 1]
