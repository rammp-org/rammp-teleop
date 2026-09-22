"""Safety filtering applied to every commanded target before it leaves the loop.

Independent of the mapper's smoothing, this is a hard cap: it bounds EE speed
(per-tick translation/rotation step) and confines the target to a workspace box.
The deadman behavior (disengage -> freeze in place) is expressed by the loop via
the FREEZE flag; fault reaction is handled in the loop. Keeping these limits here
means they apply regardless of which PoseSource or mapping produced the target.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from .transforms import slerp_rotation


@dataclass
class SafetyConfig:
    # Workspace box in base frame, meters: [xmin, ymin, zmin], [xmax, ymax, zmax].
    ws_min: np.ndarray = field(default_factory=lambda: np.array([0.2, -0.4, 0.05]))
    ws_max: np.ndarray = field(default_factory=lambda: np.array([0.75, 0.4, 0.7]))
    # Max per-tick motion (velocity clamp). At 60 Hz, 0.01 m/tick = 0.6 m/s.
    max_lin_step: float = 0.01    # meters per tick
    max_ang_step: float = 0.05    # radians per tick

    def __post_init__(self):
        self.ws_min = np.asarray(self.ws_min, float)
        self.ws_max = np.asarray(self.ws_max, float)


@dataclass
class SafetyResult:
    pos: np.ndarray
    rot: Rotation
    clamped_velocity: bool = False
    clamped_workspace: bool = False


class SafetyFilter:
    """Bounds the commanded target's speed and confines it to the workspace box.

    Stateful: tracks the last issued command to enforce the per-tick velocity
    clamp. Call :meth:`reset` with the current EE pose whenever control resumes
    (e.g. on engage or after a fault) so the clamp baseline is correct.
    """

    def __init__(self, config: SafetyConfig | None = None):
        self.cfg = config or SafetyConfig()
        self._last_pos: np.ndarray | None = None
        self._last_rot: Rotation | None = None

    def reset(self, pos, rot: Rotation) -> None:
        self._last_pos = np.asarray(pos, float).copy()
        self._last_rot = rot

    def filter(self, pos, rot: Rotation) -> SafetyResult:
        pos = np.asarray(pos, float).copy()

        # Workspace box clamp first (so the velocity clamp can't be defeated by
        # a target that sits far outside the box).
        clipped = np.clip(pos, self.cfg.ws_min, self.cfg.ws_max)
        clamped_ws = not np.array_equal(clipped, pos)
        pos = clipped

        if self._last_pos is None:
            self._last_pos, self._last_rot = pos.copy(), rot
            return SafetyResult(pos, rot, clamped_workspace=clamped_ws)

        clamped_vel = False

        # Translation velocity clamp.
        delta = pos - self._last_pos
        dist = np.linalg.norm(delta)
        if dist > self.cfg.max_lin_step:
            pos = self._last_pos + delta * (self.cfg.max_lin_step / dist)
            clamped_vel = True

        # Rotation velocity clamp (limit geodesic step).
        ang = (self._last_rot.inv() * rot).magnitude()
        if ang > self.cfg.max_ang_step:
            rot = slerp_rotation(self._last_rot, rot, self.cfg.max_ang_step / ang)
            clamped_vel = True

        self._last_pos, self._last_rot = pos.copy(), rot
        return SafetyResult(pos, rot, clamped_velocity=clamped_vel,
                            clamped_workspace=clamped_ws)
