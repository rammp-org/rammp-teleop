import numpy as np
from scipy.spatial.transform import Rotation

from rammp_teleop.quest.safety import SafetyFilter, SafetyConfig
from rammp_teleop.quest.transforms import angle_between

ID = Rotation.identity()


def test_workspace_box_clamps_position():
    sf = SafetyFilter(
        SafetyConfig(
            ws_min=np.array([0.2, -0.4, 0.05]), ws_max=np.array([0.75, 0.4, 0.7])
        )
    )
    r = sf.filter([1.5, -2.0, 0.4], ID)
    assert r.clamped_workspace
    assert r.pos[0] == 0.75
    assert r.pos[1] == -0.4
    assert r.pos[2] == 0.4


def test_inside_box_not_clamped():
    sf = SafetyFilter()
    r = sf.filter([0.45, 0.0, 0.35], ID)
    assert not r.clamped_workspace
    assert not r.clamped_velocity


def test_velocity_clamp_limits_translation_step():
    sf = SafetyFilter(SafetyConfig(max_lin_step=0.01))
    sf.reset([0.45, 0.0, 0.35], ID)
    r = sf.filter([0.55, 0.0, 0.35], ID)  # wants +0.10 m in one tick
    assert r.clamped_velocity
    step = np.linalg.norm(r.pos - np.array([0.45, 0.0, 0.35]))
    assert abs(step - 0.01) < 1e-9


def test_velocity_clamp_limits_rotation_step():
    sf = SafetyFilter(SafetyConfig(max_ang_step=0.05))
    sf.reset([0.45, 0.0, 0.35], ID)
    big = Rotation.from_euler("z", 60, degrees=True)
    r = sf.filter([0.45, 0.0, 0.35], big)
    assert r.clamped_velocity
    assert (angle_between(ID, r.rot) - 0.05) < 1e-9


def test_small_step_passes_through():
    sf = SafetyFilter(SafetyConfig(max_lin_step=0.01))
    sf.reset([0.45, 0.0, 0.35], ID)
    r = sf.filter([0.455, 0.0, 0.35], ID)  # 0.005 m < limit
    assert not r.clamped_velocity
    assert np.allclose(r.pos, [0.455, 0.0, 0.35])
