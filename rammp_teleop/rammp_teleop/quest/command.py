"""One teleop tick, minus any driver: controller pose -> safe EE target.

This is ``TeleopLoop.tick`` from kinova-quest-teleop with the UDP driver, fault
latching and gain packets removed. The ROS node calls :meth:`update` once per
timer tick and streams whatever comes back.

Disengaged (grip released) we keep commanding a *latched* target: the last safe
target while engaged, or the EE pose seen at the first freeze. Re-sending the
measured EE pose every tick instead would make a compliant arm chase its own sag.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .mapping import ClutchedDeltaMapper, MappingConfig
from .safety import SafetyConfig, SafetyFilter


class QuestCommand:
    def __init__(
        self,
        mapping: MappingConfig,
        safety: SafetyConfig,
        gripper_binary: bool = False,
        gripper_threshold: float = 0.5,
    ):
        self._mapping_cfg = mapping
        self.mapper = ClutchedDeltaMapper(mapping)
        self.safety = SafetyFilter(safety)
        self.gripper_binary = gripper_binary
        self.gripper_threshold = gripper_threshold
        self._hold_pos: np.ndarray | None = None
        self._hold_rot: Rotation | None = None
        self._reset_safety = True  # clamp baseline must start from the measured EE
        self.stats = {"rejected": 0, "vel_clamped": 0, "ws_clamped": 0}

    def resync(self) -> None:
        """Forget the hold target and the clutch references.

        If the grip is held on the next tick the mapper sees a rising edge and
        re-captures both references, so the first output equals the measured EE.
        """
        self._hold_pos = None
        self._hold_rot = None
        self._reset_safety = True
        self.mapper = ClutchedDeltaMapper(self._mapping_cfg)

    def gripper(self, trigger: float) -> float:
        if self.gripper_binary:
            return 1.0 if trigger >= self.gripper_threshold else 0.0
        return float(np.clip(trigger, 0.0, 1.0))

    def update(
        self, ctrl_pose: np.ndarray, grip: bool, ee_pos, ee_rot: Rotation
    ) -> tuple[np.ndarray, Rotation, bool]:
        if self._reset_safety:
            self.safety.reset(ee_pos, ee_rot)
            self._reset_safety = False
        result = self.mapper.update(ctrl_pose, grip, ee_pos, ee_rot)
        if result.rejected:
            self.stats["rejected"] += 1

        if result.engaged:
            safe = self.safety.filter(result.pos, result.rot)
            if safe.clamped_velocity:
                self.stats["vel_clamped"] += 1
            if safe.clamped_workspace:
                self.stats["ws_clamped"] += 1
            self._hold_pos, self._hold_rot = safe.pos.copy(), safe.rot
            return safe.pos, safe.rot, True

        if self._hold_pos is None:
            self._hold_pos = np.asarray(ee_pos, float).copy()
            self._hold_rot = ee_rot
        self.safety.reset(ee_pos, ee_rot)  # clamp baseline tracks reality for re-engage
        return self._hold_pos.copy(), self._hold_rot, False
