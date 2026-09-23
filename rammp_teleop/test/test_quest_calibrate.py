import pytest
import numpy as np
from scipy.spatial.transform import Rotation

from rammp_teleop.quest.calibrate import (
    LiveCalibrator,
    load_r_align,
    r_align_from_forward,
    r_align_from_gestures,
    run_calibration,
    save_r_align,
)
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
    R = r_align_from_gestures(
        fwd=inv.apply([0.23, 0, 0]),
        left=inv.apply([0, 0.07, 0]),
        up=inv.apply([0, 0, 0.5]),
    )
    assert np.allclose(R.as_matrix(), R_true.as_matrix(), atol=1e-9)


def test_run_calibration_captures_grip_windows_and_recovers_rotation():
    R_true = Rotation.from_euler("ZYX", [30.0, 10.0, -20.0], degrees=True)
    inv = R_true.inv()
    p0 = np.array([0.5, -0.1, 0.3])
    frames = []
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):  # fwd, left, up
        d = inv.apply(np.asarray(axis) * 0.2)
        frames.append((p0, True))  # grip rising -> start
        frames.append((p0 + d, False))  # grip falling -> end, delta = d
    R = run_calibration(ScriptedSource(frames))
    assert np.allclose(R.as_matrix(), R_true.as_matrix(), atol=1e-9)


def test_noisy_non_orthogonal_gestures_give_a_proper_rotation():
    # Real hand motions aren't perfectly orthogonal; result must still be a
    # valid rotation (orthonormal, det +1), not a skewed matrix.
    R = r_align_from_gestures(
        fwd=[1.0, 0.1, -0.05], left=[-0.08, 1.0, 0.06], up=[0.04, -0.09, 1.0]
    )
    M = R.as_matrix()
    assert np.allclose(M @ M.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(M), 1.0, atol=1e-9)


# --- one-gesture (yaw-only) calibration -------------------------------------


def _ctrl_frame(yaw_deg: float) -> Rotation:
    """A controller->base rotation that is the y-up/z-up swap plus a yaw."""
    swap = Rotation.from_matrix(
        np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    )  # ctrl -z -> base +x, ctrl -x -> base +y, ctrl +y -> base +z
    return Rotation.from_euler("z", yaw_deg, degrees=True) * swap


def test_one_gesture_recovers_yaw_with_gravity_up():
    R_true = _ctrl_frame(37.0)
    fwd_ctrl = R_true.inv().apply([1.0, 0.0, 0.0])  # what the controller saw
    R = r_align_from_forward(fwd_ctrl)
    assert np.allclose(R.apply(fwd_ctrl), [1.0, 0.0, 0.0], atol=1e-6)
    assert np.allclose(R.apply([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0], atol=1e-6)
    assert np.allclose(
        R.apply(R_true.inv().apply([0.0, 1.0, 0.0])), [0.0, 1.0, 0.0], atol=1e-6
    )


def test_one_gesture_ignores_vertical_component_of_the_move():
    R_true = _ctrl_frame(-100.0)
    fwd_ctrl = R_true.inv().apply(
        [1.0, 0.0, 0.4]
    )  # hand drifted up while moving forward
    R = r_align_from_forward(fwd_ctrl)
    assert np.allclose(R.as_matrix(), R_true.as_matrix(), atol=1e-6)


def test_one_gesture_rejects_a_purely_vertical_move():
    with pytest.raises(ValueError):
        r_align_from_forward([0.0, 0.3, 0.0])


# --- live calibrator state machine ------------------------------------------


def test_live_calibrator_arms_after_holding_a():
    c = LiveCalibrator(hold_s=2.0)
    assert c.step(0.0, a_held=True, grip=False, ctrl_pos=None) is None
    assert c.step(1.9, a_held=True, grip=False, ctrl_pos=None) is None
    assert not c.active
    ev = c.step(2.1, a_held=True, grip=False, ctrl_pos=None)
    assert ev == ("armed", None)
    assert c.active


def test_live_calibrator_releasing_a_early_does_not_arm():
    c = LiveCalibrator(hold_s=2.0)
    c.step(0.0, a_held=True, grip=False, ctrl_pos=None)
    c.step(1.0, a_held=False, grip=False, ctrl_pos=None)
    assert c.step(2.5, a_held=True, grip=False, ctrl_pos=None) is None
    assert not c.active


def test_live_calibrator_captures_grip_move_and_returns_rotation():
    R_true = _ctrl_frame(20.0)
    start = np.array([0.1, 0.9, -0.3])
    end = start + R_true.inv().apply([0.25, 0.0, 0.0])
    c = LiveCalibrator(hold_s=2.0)
    c.step(0.0, a_held=True, grip=False, ctrl_pos=start)
    assert c.step(2.0, a_held=True, grip=False, ctrl_pos=start) == ("armed", None)
    assert c.step(3.0, a_held=False, grip=True, ctrl_pos=start) is None  # rising edge
    kind, R = c.step(4.0, a_held=False, grip=False, ctrl_pos=end)
    assert kind == "done"
    assert np.allclose(R.as_matrix(), R_true.as_matrix(), atol=1e-6)
    assert not c.active


def test_live_calibrator_rejects_short_move_and_stays_armed():
    c = LiveCalibrator(hold_s=2.0, min_move=0.10)
    p = np.zeros(3)
    c.step(0.0, a_held=True, grip=False, ctrl_pos=p)
    c.step(2.0, a_held=True, grip=False, ctrl_pos=p)
    c.step(3.0, a_held=False, grip=True, ctrl_pos=p)
    assert c.step(4.0, a_held=False, grip=False, ctrl_pos=p + [0.0, 0.0, -0.03]) == (
        "rejected",
        None,
    )
    assert c.active


def test_live_calibrator_times_out():
    c = LiveCalibrator(hold_s=2.0, timeout_s=30.0)
    c.step(0.0, a_held=True, grip=False, ctrl_pos=None)
    c.step(2.0, a_held=True, grip=False, ctrl_pos=None)
    assert c.step(33.0, a_held=False, grip=False, ctrl_pos=None) == ("timeout", None)
    assert not c.active


# --- persistence --------------------------------------------------------------


def test_r_align_round_trips_through_file(tmp_path):
    path = tmp_path / "quest_r_align.json"
    assert load_r_align(path) is None
    R = _ctrl_frame(131.0)
    save_r_align(path, R)
    R2 = load_r_align(path)
    assert np.allclose(R2.as_matrix(), R.as_matrix(), atol=1e-6)
