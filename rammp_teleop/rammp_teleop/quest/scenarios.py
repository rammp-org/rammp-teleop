"""Scripted mock-Quest inputs for whole-system testing without a headset.

Each scenario returns a list of ``ScriptFrame`` (controller pose + buttons) that a
``MockPoseSource`` replays one frame per tick. Scenarios are deterministic and
reusable across the ``loopback`` driver (offline, self-checking) and the ``udp``
driver (real C++ socket server, on the Jetson), so the same mock motion exercises
the whole pipeline end to end.

Each scenario targets a specific behavior; see ``EXPECTATIONS`` and
``tests/test_scenarios.py`` for the pass criteria.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .pose_source import ScriptFrame


def _ease(s: float) -> float:
    """Cosine ease-in/out on a 0..1 parameter."""
    return 0.5 - 0.5 * np.cos(np.pi * float(np.clip(s, 0.0, 1.0)))


class _Builder:
    """Builds a controller trajectory frame-by-frame at a fixed dt.

    Holds a moving "cursor" controller pose; methods append frames that hold or
    smoothly move it. ``grip``/``trigger`` are sticky until changed.
    """

    def __init__(self, dt: float = 1.0 / 60.0):
        self.dt = dt
        self.frames: list[ScriptFrame] = []
        self.pos = np.zeros(3)
        self.rot = Rotation.identity()
        self.grip = False
        self.trigger = 0.0

    def _emit(self) -> None:
        self.frames.append(ScriptFrame(pos=self.pos.copy(), rot=self.rot,
                                       grip=self.grip, trigger=self.trigger))

    def set_grip(self, g: bool) -> "_Builder":
        self.grip = g
        return self

    def set_trigger(self, t: float) -> "_Builder":
        self.trigger = float(t)
        return self

    def hold(self, seconds: float) -> "_Builder":
        for _ in range(max(1, int(round(seconds / self.dt)))):
            self._emit()
        return self

    def move(self, d_pos=(0, 0, 0), d_euler_deg=(0, 0, 0), seconds: float = 1.0,
             ease: bool = True) -> "_Builder":
        """Smoothly translate/rotate the cursor by a delta over ``seconds``."""
        n = max(1, int(round(seconds / self.dt)))
        p0 = self.pos.copy()
        r0 = self.rot
        d_pos = np.asarray(d_pos, float)
        d_rot = Rotation.from_euler("xyz", d_euler_deg, degrees=True)
        r1 = d_rot * r0
        for i in range(1, n + 1):
            s = _ease(i / n) if ease else i / n
            self.pos = p0 + s * d_pos
            # SLERP from r0 to r1.
            self.rot = Rotation.from_rotvec((r1 * r0.inv()).as_rotvec() * s) * r0
            self._emit()
        self.rot = r1
        return self

    def glitch(self, d_pos=(0, 0, 0), d_euler_deg=(0, 0, 0)) -> "_Builder":
        """Emit ONE teleported frame without moving the cursor (a tracking
        glitch the jump-reject must drop)."""
        saved_p, saved_r = self.pos.copy(), self.rot
        self.pos = saved_p + np.asarray(d_pos, float)
        self.rot = Rotation.from_euler("xyz", d_euler_deg, degrees=True) * saved_r
        self._emit()
        self.pos, self.rot = saved_p, saved_r  # cursor unchanged
        return self

    def build(self) -> list[ScriptFrame]:
        return self.frames


# --- scenarios --------------------------------------------------------------

def demo(dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    """Representative on-camera motion: engage, move through space, use the
    gripper, return, disengage."""
    b = _Builder(dt)
    b.hold(0.4)
    b.set_grip(True)
    b.move(d_pos=(0.12, 0, 0), seconds=0.8)        # forward
    b.move(d_pos=(0, 0.10, 0), seconds=0.7)        # right
    b.move(d_pos=(0, 0, 0.08), seconds=0.6)        # up
    b.move(d_euler_deg=(0, 0, 25), seconds=0.6)    # wrist yaw
    b.set_trigger(1.0).hold(0.4)                   # close gripper
    b.move(d_euler_deg=(0, 20, 0), seconds=0.6)    # tilt
    b.set_trigger(0.0).hold(0.3)                   # open gripper
    b.move(d_pos=(-0.12, -0.10, -0.08), d_euler_deg=(0, -20, -25), seconds=1.0)
    b.set_grip(False).hold(0.4)
    return b.build()


def axis_translations(dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    """Engage, translate +x/+y/+z each and return — validates the control-basis
    rotation (R_align) axis mapping."""
    b = _Builder(dt)
    b.hold(0.3).set_grip(True)
    for axis in ([0.10, 0, 0], [0, 0.10, 0], [0, 0, 0.10]):
        b.move(d_pos=axis, seconds=0.6)
        b.move(d_pos=[-x for x in axis], seconds=0.6)
    b.set_grip(False).hold(0.2)
    return b.build()


def axis_rotations(dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    """Engage, rotate about roll/pitch/yaw each and return — validates rotation
    mapping and translation/rotation decoupling (position must not drift)."""
    b = _Builder(dt)
    b.hold(0.3).set_grip(True)
    for rpy in ([20, 0, 0], [0, 20, 0], [0, 0, 20]):
        b.move(d_euler_deg=rpy, seconds=0.6)
        b.move(d_euler_deg=[-a for a in rpy], seconds=0.6)
    b.set_grip(False).hold(0.2)
    return b.build()


def gripper_cycle(dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    """Engage, then close/open/close the gripper while holding pose."""
    b = _Builder(dt)
    b.hold(0.3).set_grip(True).hold(0.3)
    b.set_trigger(1.0).hold(0.4)
    b.set_trigger(0.0).hold(0.4)
    b.set_trigger(1.0).hold(0.4)
    b.set_grip(False).hold(0.2)
    return b.build()


def jump_glitch(dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    """Engage, move, then inject a single-frame teleport (tracking dropout) that
    the jump-reject must reject — the EE must not lurch."""
    b = _Builder(dt)
    b.hold(0.3).set_grip(True)
    b.move(d_pos=(0.05, 0, 0), seconds=0.5)
    b.glitch(d_pos=(0.6, -0.4, 0.3))      # teleport, one frame
    b.glitch(d_euler_deg=(90, 0, 0))      # orientation teleport, one frame
    b.move(d_pos=(0.05, 0, 0), seconds=0.5)  # resume normal motion
    b.set_grip(False).hold(0.2)
    return b.build()


def workspace_escape(dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    """Engage and drive the target far past the workspace box — the box clamp
    must bound the commanded target."""
    b = _Builder(dt)
    b.hold(0.3).set_grip(True)
    # Many small (< jump tol) steps that accumulate well past the +x/+z box edge.
    for _ in range(40):
        b.move(d_pos=(0.03, 0, 0.02), seconds=dt, ease=False)
    b.set_grip(False).hold(0.2)
    return b.build()


def velocity_spike(dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    """Engage and move fast — under the per-tick jump tolerance but above the
    velocity clamp — so the velocity clamp (not jump-reject) must engage."""
    b = _Builder(dt)
    b.hold(0.3).set_grip(True)
    for _ in range(20):
        b.move(d_pos=(0.03, 0, 0), seconds=dt, ease=False)  # 0.03/tick > 0.01 clamp
    b.set_grip(False).hold(0.2)
    return b.build()


def clutch_freeze(dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    """Engage and move, disengage and fly the controller away (deadman must
    freeze the EE), then re-engage cleanly (no lurch)."""
    b = _Builder(dt)
    b.hold(0.3).set_grip(True)
    b.move(d_pos=(0.08, 0.04, 0), seconds=0.6)
    b.set_grip(False)
    b.move(d_pos=(0.5, -0.5, 0.4), seconds=0.6)   # controller flies while released
    b.set_grip(True).hold(0.2)                    # re-engage at new location
    b.move(d_pos=(0, 0, 0.05), seconds=0.4)
    b.set_grip(False).hold(0.2)
    return b.build()


SCENARIOS = {
    "demo": demo,
    "axis_translations": axis_translations,
    "axis_rotations": axis_rotations,
    "gripper_cycle": gripper_cycle,
    "jump_glitch": jump_glitch,
    "workspace_escape": workspace_escape,
    "velocity_spike": velocity_spike,
    "clutch_freeze": clutch_freeze,
}

# Human-readable pass criteria, surfaced by the runbook and the CLI.
EXPECTATIONS = {
    "demo": "EE follows a smooth path; gripper closes then opens; returns near home; no faults.",
    "axis_translations": "EE translates along each base axis in turn (check R_align).",
    "axis_rotations": "EE rotates about each axis; EE position stays put (decoupled).",
    "gripper_cycle": "gripper_state cycles closed/open/closed; EE holds pose.",
    "jump_glitch": "stats.rejected > 0; EE does not lurch on the teleport frames.",
    "workspace_escape": "stats.ws_clamped > 0; commanded target stays inside the box.",
    "velocity_spike": "stats.vel_clamped > 0; EE speed bounded by max_lin_step.",
    "clutch_freeze": "EE freezes while released; re-engages with no jump.",
}


def get(name: str, dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    if name not in SCENARIOS:
        raise KeyError(f"unknown scenario '{name}'; choose from {list(SCENARIOS)}")
    return SCENARIOS[name](dt)
