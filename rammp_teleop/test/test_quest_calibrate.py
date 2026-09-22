import numpy as np
from scipy.spatial.transform import Rotation

from rammp_teleop.quest.calibrate import r_align_from_gestures, run_calibration
from rammp_teleop.quest.pose_source import Buttons


class ScriptedSource:
    """Replays (translation, grip) frames as (pose_4x4, Buttons) for capture."""

    def __init__(self, frames):
        self.frames = frames
        self._i = 0

    def read(self):
        pos, grip = self.frames[min(self._i, len(self.frames) - 1)]
        self._i += 1
        T = np.eye(4)
        T[:3, 3] = pos
        return T, Buttons(grip=grip)


def _apply(R, v):
    return R.apply(v)


def test_orthonormal_gestures_recover_identity():
    # Hand moved exactly along base +X, +Y, +Z in the controller frame already.
    R = r_align_from_gestures(fwd=[1, 0, 0], left=[0, 1, 0], up=[0, 0, 1])
    assert np.allclose(R.as_matrix(), np.eye(3), atol=1e-9)


def test_recovers_a_known_rotation():
    # Build gestures from a known controller->base rotation R_true:
    # moving along base axis e_i is observed in the controller frame as
    # R_true^{-1} @ e_i, so r_align_from_gestures must recover R_true.
    R_true = Rotation.from_euler("ZYX", [40.0, -25.0, 15.0], degrees=True)
    inv = R_true.inv()
    fwd = inv.apply([1, 0, 0])
    left = inv.apply([0, 1, 0])
    up = inv.apply([0, 0, 1])
    R = r_align_from_gestures(fwd=fwd, left=left, up=up)
    assert np.allclose(R.as_matrix(), R_true.as_matrix(), atol=1e-9)


def test_magnitude_independent():
    # Only direction matters; gesture length must not change the result.
    R_true = Rotation.from_euler("z", 90.0, degrees=True)
    inv = R_true.inv()
    R = r_align_from_gestures(fwd=inv.apply([0.23, 0, 0]),
                              left=inv.apply([0, 0.07, 0]),
                              up=inv.apply([0, 0, 0.5]))
    assert np.allclose(R.as_matrix(), R_true.as_matrix(), atol=1e-9)


def test_run_calibration_captures_grip_windows_and_recovers_rotation():
    R_true = Rotation.from_euler("ZYX", [30.0, 10.0, -20.0], degrees=True)
    inv = R_true.inv()
    p0 = np.array([0.5, -0.1, 0.3])
    frames = []
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):  # fwd, left, up
        d = inv.apply(np.asarray(axis) * 0.2)
        frames.append((p0, True))        # grip rising -> start
        frames.append((p0 + d, False))   # grip falling -> end, delta = d
    R = run_calibration(ScriptedSource(frames))
    assert np.allclose(R.as_matrix(), R_true.as_matrix(), atol=1e-9)


def test_noisy_non_orthogonal_gestures_give_a_proper_rotation():
    # Real hand motions aren't perfectly orthogonal; result must still be a
    # valid rotation (orthonormal, det +1), not a skewed matrix.
    R = r_align_from_gestures(fwd=[1.0, 0.1, -0.05],
                              left=[-0.08, 1.0, 0.06],
                              up=[0.04, -0.09, 1.0])
    M = R.as_matrix()
    assert np.allclose(M @ M.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(M), 1.0, atol=1e-9)
