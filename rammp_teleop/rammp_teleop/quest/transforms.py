"""SE(3) helpers shared across the supervisor.

Poses are 4x4 homogeneous matrices (numpy). Quaternions on the wire are
**w, x, y, z** (Eigen order); scipy's ``Rotation`` uses x, y, z, w, so all
conversions to/from scipy go through the explicit helpers here.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def quat_wxyz_to_rot(quat_wxyz) -> Rotation:
    w, x, y, z = quat_wxyz
    return Rotation.from_quat([x, y, z, w])


def rot_to_quat_wxyz(rot: Rotation) -> np.ndarray:
    x, y, z, w = rot.as_quat()
    return np.array([w, x, y, z])


def make_pose(pos, rot: Rotation) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = np.asarray(pos, dtype=float)
    return T


def pose_from_pos_quat(pos, quat_wxyz) -> np.ndarray:
    return make_pose(pos, quat_wxyz_to_rot(quat_wxyz))


def pos_quat_from_pose(T) -> tuple[np.ndarray, np.ndarray]:
    """Return (pos[3], quat_wxyz[4]) from a 4x4 pose."""
    pos = np.asarray(T[:3, 3], dtype=float)
    quat = rot_to_quat_wxyz(Rotation.from_matrix(T[:3, :3]))
    return pos, quat


def invert_pose(T) -> np.ndarray:
    R = T[:3, :3]
    p = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


def slerp_rotation(r_from: Rotation, r_to: Rotation, alpha: float) -> Rotation:
    """Interpolate a fraction ``alpha`` in [0, 1] from ``r_from`` toward ``r_to``."""
    alpha = float(np.clip(alpha, 0.0, 1.0))
    slerp = Slerp([0.0, 1.0], Rotation.concatenate([r_from, r_to]))
    return slerp([alpha])[0]


def angle_between(r_a: Rotation, r_b: Rotation) -> float:
    """Geodesic angle (radians) between two rotations."""
    return (r_a.inv() * r_b).magnitude()
