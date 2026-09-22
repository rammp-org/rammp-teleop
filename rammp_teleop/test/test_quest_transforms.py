import numpy as np
from scipy.spatial.transform import Rotation

from rammp_teleop.quest import transforms as tf


def test_quat_wxyz_roundtrip():
    r = Rotation.from_euler("xyz", [10, 20, 30], degrees=True)
    q = tf.rot_to_quat_wxyz(r)
    r2 = tf.quat_wxyz_to_rot(q)
    assert tf.angle_between(r, r2) < 1e-9


def test_identity_quat_is_wxyz_1000():
    q = tf.rot_to_quat_wxyz(Rotation.identity())
    # w should be ~1, vector part ~0 (sign of scipy may flip, so check |w|).
    assert abs(abs(q[0]) - 1.0) < 1e-9
    assert np.allclose(q[1:], 0, atol=1e-9)


def test_pose_pos_quat_roundtrip():
    pos = np.array([0.3, -0.1, 0.5])
    rot = Rotation.from_euler("z", 45, degrees=True)
    T = tf.make_pose(pos, rot)
    p2, q2 = tf.pos_quat_from_pose(T)
    assert np.allclose(p2, pos)
    assert tf.angle_between(rot, tf.quat_wxyz_to_rot(q2)) < 1e-9


def test_invert_pose():
    T = tf.make_pose([1, 2, 3], Rotation.from_euler("y", 60, degrees=True))
    Ti = tf.invert_pose(T)
    assert np.allclose(T @ Ti, np.eye(4), atol=1e-12)


def test_slerp_endpoints_and_midpoint():
    a = Rotation.identity()
    b = Rotation.from_euler("z", 90, degrees=True)
    assert tf.angle_between(tf.slerp_rotation(a, b, 0.0), a) < 1e-9
    assert tf.angle_between(tf.slerp_rotation(a, b, 1.0), b) < 1e-9
    mid = tf.slerp_rotation(a, b, 0.5)
    assert abs(np.degrees(tf.angle_between(a, mid)) - 45.0) < 1e-6


def test_slerp_clamps_alpha():
    a = Rotation.identity()
    b = Rotation.from_euler("z", 90, degrees=True)
    assert tf.angle_between(tf.slerp_rotation(a, b, -5.0), a) < 1e-9
    assert tf.angle_between(tf.slerp_rotation(a, b, 5.0), b) < 1e-9
