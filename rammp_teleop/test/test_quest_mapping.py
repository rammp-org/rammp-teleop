import numpy as np
from scipy.spatial.transform import Rotation

from rammp_teleop.quest.mapping import ClutchedDeltaMapper, MappingConfig
from rammp_teleop.quest.transforms import make_pose, angle_between

EE_REF_POS = np.array([0.45, 0.0, 0.35])
EE_REF_ROT = Rotation.from_quat([0, 1, 0, 0])  # tool down


def ctrl(pos, rot=None):
    return make_pose(pos, rot if rot is not None else Rotation.identity())


def engage(mapper, ctrl_pos, ctrl_rot=None):
    """Rising-edge engage at a controller pose; returns the first result."""
    return mapper.update(ctrl(ctrl_pos, ctrl_rot), True, EE_REF_POS, EE_REF_ROT)


def test_holds_ee_before_engage():
    m = ClutchedDeltaMapper()
    r = m.update(ctrl([0, 0, 0]), False, EE_REF_POS, EE_REF_ROT)
    assert not r.engaged
    assert np.allclose(r.pos, EE_REF_POS)


def test_first_engage_output_is_ee_reference():
    m = ClutchedDeltaMapper()
    r = engage(m, [0, 0, 0])
    assert r.engaged
    assert np.allclose(r.pos, EE_REF_POS)
    assert angle_between(r.rot, EE_REF_ROT) < 1e-9


def test_translation_maps_to_ee_translation():
    m = ClutchedDeltaMapper(MappingConfig(trans_smooth=1.0, rot_smooth=1.0))
    engage(m, [0.0, 0.0, 0.0])
    # Move controller +0.10 m in x over small (<jump_tol) steps.
    r = None
    for x in np.linspace(0.02, 0.10, 5):
        r = m.update(ctrl([x, 0, 0]), True, EE_REF_POS, EE_REF_ROT)
    assert np.allclose(r.pos, EE_REF_POS + [0.10, 0, 0], atol=1e-6)


def test_rotation_does_not_move_position_decoupled():
    m = ClutchedDeltaMapper(MappingConfig(trans_smooth=1.0, rot_smooth=1.0))
    engage(m, [0, 0, 0])
    # Rotate controller in place; position must stay at the EE reference.
    r = None
    for deg in np.linspace(5, 25, 5):
        rot = Rotation.from_euler("z", deg, degrees=True)
        r = m.update(ctrl([0, 0, 0], rot), True, EE_REF_POS, EE_REF_ROT)
    assert np.allclose(r.pos, EE_REF_POS, atol=1e-9)


def test_R_align_rotates_translation_axis():
    # R_align maps controller +x onto base +y.
    cfg = MappingConfig(
        trans_smooth=1.0, R_align=Rotation.from_euler("z", 90, degrees=True)
    )
    m = ClutchedDeltaMapper(cfg)
    engage(m, [0, 0, 0])
    r = None
    for x in np.linspace(0.02, 0.10, 5):
        r = m.update(ctrl([x, 0, 0]), True, EE_REF_POS, EE_REF_ROT)
    assert np.allclose(r.pos, EE_REF_POS + [0, 0.10, 0], atol=1e-6)


def test_jump_is_rejected():
    m = ClutchedDeltaMapper(MappingConfig(trans_smooth=1.0))
    engage(m, [0, 0, 0])
    m.update(ctrl([0.02, 0, 0]), True, EE_REF_POS, EE_REF_ROT)
    before = m._cmd_pos.copy()
    # A 0.5 m teleport in one tick exceeds jump_pos_tol (0.05).
    r = m.update(ctrl([0.52, 0, 0]), True, EE_REF_POS, EE_REF_ROT)
    assert r.rejected
    assert np.allclose(r.pos, before)


def test_disengage_freezes_pose():
    m = ClutchedDeltaMapper(MappingConfig(trans_smooth=1.0))
    engage(m, [0, 0, 0])
    for x in np.linspace(0.02, 0.06, 3):
        m.update(ctrl([x, 0, 0]), True, EE_REF_POS, EE_REF_ROT)
    frozen = m._cmd_pos.copy()
    # Release grip and keep moving the controller: output must not change.
    r = m.update(ctrl([0.5, 0, 0]), False, EE_REF_POS, EE_REF_ROT)
    assert not r.engaged
    assert np.allclose(r.pos, frozen)
    r2 = m.update(ctrl([0.9, 0.3, 0.2]), False, EE_REF_POS, EE_REF_ROT)
    assert np.allclose(r2.pos, frozen)


def test_reengage_recaptures_reference():
    m = ClutchedDeltaMapper(MappingConfig(trans_smooth=1.0))
    engage(m, [0, 0, 0])
    for x in np.linspace(0.02, 0.06, 3):
        m.update(ctrl([x, 0, 0]), True, EE_REF_POS, EE_REF_ROT)
    moved = m._cmd_pos.copy()
    m.update(ctrl([0.06, 0, 0]), False, EE_REF_POS, EE_REF_ROT)  # disengage
    # Re-engage at a new controller location with the EE now at `moved`.
    r = m.update(ctrl([1.0, 1.0, 1.0]), True, moved, EE_REF_ROT)
    assert np.allclose(r.pos, moved)  # no jump on re-engage


# --- rotation frame ---------------------------------------------------------


def _tool_cfg(**kw):
    return MappingConfig(trans_smooth=1.0, rot_smooth=1.0, rot_frame="tool", **kw)


def test_rot_frame_is_validated():
    import pytest

    with pytest.raises(ValueError):
        MappingConfig(rot_frame="wrist")


def test_tool_frame_with_engage_offset_matches_base_frame():
    """The identity that makes the whole choice meaningful.

    Setting R_tool to the controller->EE offset implied at engage time makes tool
    frame algebraically identical to base frame:

        R_tool = ee_ref^-1 * (R_align * ctrl_ref)
        =>  ee_ref * (R_tool dB R_tool^-1) == (R dR_ctrl R^-1) * ee_ref

    So simply "switching to EE frame" is a no-op unless you also pick a DIFFERENT
    axis correspondence. Identity (the default) is that different choice.
    """
    R_align = Rotation.from_euler("z", 35, degrees=True)
    ctrl_ref = Rotation.from_euler("xyz", [10, -20, 30], degrees=True)
    R_tool = EE_REF_ROT.inv() * (R_align * ctrl_ref)

    base = ClutchedDeltaMapper(
        MappingConfig(trans_smooth=1.0, rot_smooth=1.0, R_align=R_align)
    )
    tool = ClutchedDeltaMapper(_tool_cfg(R_align=R_align, R_tool=R_tool))
    engage(base, [0, 0, 0], ctrl_ref)
    engage(tool, [0, 0, 0], ctrl_ref)

    for deg in np.linspace(4, 20, 5):
        rot = ctrl_ref * Rotation.from_euler(
            "xyz", [deg, deg / 2, -deg / 3], degrees=True
        )
        rb = base.update(ctrl([0, 0, 0], rot), True, EE_REF_POS, EE_REF_ROT)
        rt = tool.update(ctrl([0, 0, 0], rot), True, EE_REF_POS, EE_REF_ROT)
        assert angle_between(rb.rot, rt.rot) < 1e-9


def test_tool_frame_rotates_about_the_tools_own_axis():
    # EE reference is "tool down": its own +z points along base -z.
    # A controller roll about its own +z must rotate the EE about the EE's +z,
    # i.e. about base -z -- the OPPOSITE base-frame sense from the default mode.
    tool = ClutchedDeltaMapper(_tool_cfg())
    base = ClutchedDeltaMapper(MappingConfig(trans_smooth=1.0, rot_smooth=1.0))
    engage(tool, [0, 0, 0])
    engage(base, [0, 0, 0])

    rot = Rotation.from_euler("z", 20, degrees=True)
    rt = tool.update(ctrl([0, 0, 0], rot), True, EE_REF_POS, EE_REF_ROT)
    rb = base.update(ctrl([0, 0, 0], rot), True, EE_REF_POS, EE_REF_ROT)

    # Both rotate by the same amount from the reference...
    assert abs(angle_between(rt.rot, EE_REF_ROT) - np.deg2rad(20)) < 1e-9
    assert abs(angle_between(rb.rot, EE_REF_ROT) - np.deg2rad(20)) < 1e-9
    # ...but in opposite senses, so they are 40 deg apart from each other.
    assert abs(angle_between(rt.rot, rb.rot) - np.deg2rad(40)) < 1e-9

    # Concretely: the tool-frame result is a rotation about base -z.
    delta = (EE_REF_ROT.inv() * rt.rot).as_rotvec()
    assert np.allclose(delta / np.linalg.norm(delta), [0, 0, 1], atol=1e-9)


def test_tool_frame_rotation_ignores_R_align():
    # R_align cancels out of the body delta, so the translation calibration can no
    # longer corrupt rotation -- one fewer coupled knob to get wrong.
    a = ClutchedDeltaMapper(_tool_cfg(R_align=Rotation.identity()))
    b = ClutchedDeltaMapper(
        _tool_cfg(R_align=Rotation.from_euler("xyz", [15, 40, -25], degrees=True))
    )
    engage(a, [0, 0, 0])
    engage(b, [0, 0, 0])
    rot = Rotation.from_euler("xyz", [12, -8, 20], degrees=True)
    ra = a.update(ctrl([0, 0, 0], rot), True, EE_REF_POS, EE_REF_ROT)
    rb = b.update(ctrl([0, 0, 0], rot), True, EE_REF_POS, EE_REF_ROT)
    assert angle_between(ra.rot, rb.rot) < 1e-9


def test_tool_frame_rotation_does_not_move_position():
    m = ClutchedDeltaMapper(_tool_cfg())
    engage(m, [0, 0, 0])
    r = None
    for deg in np.linspace(5, 25, 5):
        rot = Rotation.from_euler("y", deg, degrees=True)
        r = m.update(ctrl([0, 0, 0], rot), True, EE_REF_POS, EE_REF_ROT)
    assert np.allclose(r.pos, EE_REF_POS, atol=1e-9)
