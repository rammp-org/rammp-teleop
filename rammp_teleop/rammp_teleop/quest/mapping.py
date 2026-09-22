"""Clutched delta-pose mapping: controller motion -> base-frame EE target.

When the clutch (grip) engages we capture a reference controller pose and the
current EE pose. While held, the controller's *delta* from its reference is
applied to the EE reference, so no absolute headset<->robot calibration is
needed. Only the controller->base **rotation** (``R_align``) must be right for
"controller forward = EE forward"; tune it against the mock first.

Convention is decoupled (DROID / oculus_reader style): controller translation
maps to EE translation and controller rotation maps to EE rotation,
independently (no lever-arm coupling). Translation is always base frame;
``rot_frame`` selects whether rotation acts about base axes (default) or the
tool's own axes. Output is smoothed (lerp translation,
SLERP rotation) and pose jumps above tolerance are rejected to kill
teleport-on-glitch.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .transforms import slerp_rotation


@dataclass
class MappingConfig:
    # Controller-frame -> base-frame rotation (the one calibration knob).
    R_align: Rotation = None  # type: ignore[assignment]
    # Output smoothing per tick, in (0, 1]. 1.0 = follow raw target exactly.
    trans_smooth: float = 0.5
    rot_smooth: float = 0.5
    # Per-tick controller jump rejection (a glitch/teleport guard).
    jump_pos_tol: float = 0.05  # meters of controller motion per tick
    jump_rot_tol: float = 0.5  # radians of controller rotation per tick
    # Optional scaling of controller motion -> EE motion.
    pos_scale: float = 1.0
    # Frame the ROTATION delta acts in. Position is always base frame.
    #   "base" (default) - rotate the EE about BASE axes. The delta is spatial
    #       (left-multiplied). Feels right while your hand and the tool are
    #       roughly aligned.
    #   "tool"           - rotate the EE about its OWN axes. The delta is a body
    #       rotation in the controller's own frame, applied on the right. Feels
    #       right when the tool points somewhere your hand does not, which is
    #       most of the time once the arm is working away from you.
    rot_frame: str = "base"
    # Correspondence from controller BODY axes to EE BODY axes, used only when
    # rot_frame == "tool". Identity means "twist the controller about its own
    # forward axis -> the tool twists about its own forward axis".
    #
    # NOTE this knob is the whole content of the choice. Setting it to
    # R_tool = ee_ref^-1 * (R_align * ctrl_ref) -- the offset implied at engage
    # time -- makes tool frame algebraically IDENTICAL to base frame (see
    # test_tool_frame_with_engage_offset_matches_base_frame). Switching frames
    # without deciding the axis correspondence changes nothing.
    R_tool: Rotation = None  # type: ignore[assignment]

    def __post_init__(self):
        if self.R_align is None:
            self.R_align = Rotation.identity()
        if self.R_tool is None:
            self.R_tool = Rotation.identity()
        if self.rot_frame not in ("base", "tool"):
            raise ValueError(
                f"rot_frame must be 'base' or 'tool', got {self.rot_frame!r}"
            )


@dataclass
class MapResult:
    pos: np.ndarray  # target EE position, base frame
    rot: Rotation  # target EE rotation, base frame
    engaged: bool  # clutch currently held
    rejected: bool = False  # this tick's controller sample was a jump


class ClutchedDeltaMapper:
    def __init__(self, config: MappingConfig | None = None):
        self.cfg = config or MappingConfig()
        self._engaged = False
        self._ctrl_ref_pos: np.ndarray | None = None
        self._ctrl_ref_rot: Rotation | None = None
        self._ee_ref_pos: np.ndarray | None = None
        self._ee_ref_rot: Rotation | None = None
        self._prev_ctrl_pos: np.ndarray | None = None
        self._prev_ctrl_rot: Rotation | None = None
        self._cmd_pos: np.ndarray | None = None
        self._cmd_rot: Rotation | None = None

    @property
    def engaged(self) -> bool:
        return self._engaged

    def _capture_reference(self, ctrl_pos, ctrl_rot, ee_pos, ee_rot) -> None:
        self._ctrl_ref_pos = np.asarray(ctrl_pos, float).copy()
        self._ctrl_ref_rot = ctrl_rot
        self._ee_ref_pos = np.asarray(ee_pos, float).copy()
        self._ee_ref_rot = ee_rot
        self._prev_ctrl_pos = self._ctrl_ref_pos.copy()
        self._prev_ctrl_rot = ctrl_rot
        self._cmd_pos = self._ee_ref_pos.copy()
        self._cmd_rot = ee_rot

    def update(
        self, ctrl_pose: np.ndarray, grip: bool, ee_pos, ee_rot: Rotation
    ) -> MapResult:
        """Advance the mapping one tick.

        ``ctrl_pose`` is the controller's 4x4 pose (its own frame). ``ee_pos`` /
        ``ee_rot`` are the current EE pose from feedback (base frame), used to
        capture the reference at engage time.
        """
        ctrl_pos = np.asarray(ctrl_pose[:3, 3], float)
        ctrl_rot = Rotation.from_matrix(ctrl_pose[:3, :3])

        rising = grip and not self._engaged
        falling = not grip and self._engaged

        if rising:
            self._engaged = True
            self._capture_reference(ctrl_pos, ctrl_rot, ee_pos, ee_rot)
            return MapResult(self._cmd_pos.copy(), self._cmd_rot, engaged=True)

        if falling:
            self._engaged = False

        if not self._engaged:
            # Disengaged: hold the last commanded pose (deadman freeze).
            if self._cmd_pos is None:
                return MapResult(np.asarray(ee_pos, float), ee_rot, engaged=False)
            return MapResult(self._cmd_pos.copy(), self._cmd_rot, engaged=False)

        # --- engaged: jump rejection on the controller sample ---------------
        d_pos = np.linalg.norm(ctrl_pos - self._prev_ctrl_pos)
        d_ang = (self._prev_ctrl_rot.inv() * ctrl_rot).magnitude()
        if d_pos > self.cfg.jump_pos_tol or d_ang > self.cfg.jump_rot_tol:
            # Reject this sample; hold command, keep the last accepted baseline
            # so we resume only when the controller returns near it.
            return MapResult(
                self._cmd_pos.copy(), self._cmd_rot, engaged=True, rejected=True
            )

        # --- decoupled delta in base frame ----------------------------------
        R = self.cfg.R_align
        dp_ctrl = (ctrl_pos - self._ctrl_ref_pos) * self.cfg.pos_scale
        dp_base = R.apply(dp_ctrl)
        raw_pos = self._ee_ref_pos + dp_base

        if self.cfg.rot_frame == "tool":
            # Body delta in the controller's OWN frame, re-expressed on the EE's
            # own axes and applied on the RIGHT. R_align cancels out entirely here
            # (it drops out of C_ref^-1 C), so rotation no longer depends on the
            # translation calibration -- only R_tool sets the axis correspondence.
            dR_body = self._ctrl_ref_rot.inv() * ctrl_rot
            Rt = self.cfg.R_tool
            raw_rot = self._ee_ref_rot * (Rt * dR_body * Rt.inv())
        else:
            # Spatial delta conjugated into base and applied on the LEFT, i.e. the
            # EE rotates about BASE axes.
            dR_ctrl = ctrl_rot * self._ctrl_ref_rot.inv()
            raw_rot = (R * dR_ctrl * R.inv()) * self._ee_ref_rot

        # --- smooth the commanded output ------------------------------------
        a = self.cfg.trans_smooth
        self._cmd_pos = self._cmd_pos + a * (raw_pos - self._cmd_pos)
        self._cmd_rot = slerp_rotation(self._cmd_rot, raw_rot, self.cfg.rot_smooth)

        self._prev_ctrl_pos = ctrl_pos.copy()
        self._prev_ctrl_rot = ctrl_rot
        return MapResult(self._cmd_pos.copy(), self._cmd_rot, engaged=True)
