"""QuestCommand: mapper -> safety -> hold-on-release, driven by scripted controllers.

The 'arm' here is a loopback: the measured EE pose is whatever was commanded last
tick, so smoothing and clamps are the only lag.
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rammp_teleop.quest import scenarios
from rammp_teleop.quest.command import QuestCommand
from rammp_teleop.quest.mapping import MappingConfig
from rammp_teleop.quest.pose_source import MockPoseSource, default_script
from rammp_teleop.quest.safety import SafetyConfig

EE0 = np.array([0.45, 0.0, 0.35])
ROT0 = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])  # tool pointing down


def run(cmd, source, ee_pos=EE0, ee_rot=ROT0, sag=None):
    """Replay the whole script; return list of (pos, rot, engaged)."""
    out = []
    while not source.done:
        pose, btn = source.read()
        pos, rot, engaged = cmd.update(pose, btn.grip, ee_pos, ee_rot)
        out.append((pos.copy(), rot, engaged))
        ee_pos, ee_rot = pos.copy(), rot
        if sag is not None and not engaged:
            ee_pos = ee_pos + sag  # a compliant arm droops when not driven
    return out


def make(**kw):
    return QuestCommand(MappingConfig(trans_smooth=0.8, rot_smooth=0.8), SafetyConfig(), **kw)


def test_idle_before_engage_holds_first_ee_pose():
    out = run(make(), MockPoseSource(default_script()))
    idle = out[:20]
    assert all(not e for _, _, e in idle)
    for p, _, _ in idle:
        assert np.allclose(p, EE0)


def test_hold_target_does_not_chase_sag():
    out = run(make(), MockPoseSource(default_script()), sag=np.array([0.0, 0.0, -0.001]))
    for p, _, _ in out[:20]:
        assert np.allclose(p, EE0), "freeze target must be latched, not re-read from a sagging EE"


def test_engaged_motion_follows_script_in_base_frame():
    out = run(make(), MockPoseSource(default_script()))
    # default_script moves +0.15 x, +0.05 z, yaws 30 deg while engaged (0.5 s .. 1.5 s at 60 Hz)
    p_end, r_end, engaged = out[89]
    assert engaged
    assert p_end[0] - EE0[0] > 0.10
    assert p_end[2] - EE0[2] > 0.03
    assert abs(p_end[1] - EE0[1]) < 0.01
    assert (ROT0.inv() * r_end).magnitude() > np.deg2rad(20)


def test_release_freezes_last_commanded_target():
    out = run(make(), MockPoseSource(default_script()))
    after = out[95:]
    assert all(not e for _, _, e in after)
    p_ref = after[0][0]
    for p, _, _ in after:
        assert np.allclose(p, p_ref)


def test_jump_glitch_is_rejected():
    cmd = make()
    run(cmd, MockPoseSource(scenarios.get("jump_glitch")))
    assert cmd.stats["rejected"] > 0


def test_workspace_escape_is_clamped_inside_box():
    cmd = make()
    out = run(cmd, MockPoseSource(scenarios.get("workspace_escape")))
    assert cmd.stats["ws_clamped"] > 0
    lo, hi = SafetyConfig().ws_min, SafetyConfig().ws_max
    for p, _, _ in out:
        assert np.all(p >= lo - 1e-9) and np.all(p <= hi + 1e-9)


def test_velocity_spike_is_rate_limited():
    cmd = make()
    out = run(cmd, MockPoseSource(scenarios.get("velocity_spike")))
    assert cmd.stats["vel_clamped"] > 0
    for (p0, _, _), (p1, _, _) in zip(out, out[1:]):
        assert np.linalg.norm(p1 - p0) <= SafetyConfig().max_lin_step + 1e-9


def test_gripper_passthrough_and_binary():
    assert make().gripper(0.3) == pytest.approx(0.3)
    assert make().gripper(1.7) == pytest.approx(1.0)
    b = make(gripper_binary=True, gripper_threshold=0.5)
    assert b.gripper(0.49) == 0.0
    assert b.gripper(0.5) == 1.0


def test_resync_recaptures_reference_while_engaged():
    cmd = make()
    run(cmd, MockPoseSource(default_script()))
    # The arm was moved elsewhere by someone else and the operator hits A.
    cmd.resync()
    elsewhere = np.array([0.5, 0.2, 0.4])
    pose, _ = MockPoseSource(default_script()).read()
    p, r, engaged = cmd.update(pose, True, elsewhere, ROT0)
    assert engaged
    assert np.allclose(p, elsewhere), "first tick after resync must not jump away from the measured EE"
