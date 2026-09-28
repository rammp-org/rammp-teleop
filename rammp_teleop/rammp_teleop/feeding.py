"""Pure-Python helpers for the fixed-fork feeding tests: taught joint waypoints,
hold-to-go buttons, and the stab-then-scoop macro.

Nothing here imports ROS. The Xbox node owns the plumbing and calls in once per
tick, the same split as logic.py.

The scoop macro is open-loop on purpose: it streams a constant twist for the time
each distance needs and the driver's velocity mode integrates it. The last tick of
each leg is scaled so the integral lands on the requested distance exactly.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]
ZERO3: Vec3 = (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Taught waypoints: named 7-joint configurations in a JSON file.
# ---------------------------------------------------------------------------


class WaypointStore:
    def __init__(self, path: str):
        self.path = Path(path).expanduser()
        self.slots: Dict[str, List[float]] = {}
        if self.path.exists():
            raw = json.loads(self.path.read_text())
            self.slots = {k: [float(v) for v in vals] for k, vals in raw.items()}

    def get(self, name: str) -> Optional[List[float]]:
        return self.slots.get(name)

    def save(self, name: str, joints: Sequence[float]) -> None:
        self.slots[name] = [float(v) for v in joints]
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.slots, indent=2) + "\n")


# ---------------------------------------------------------------------------
# Hold-to-go: a button that fires once after being held for `hold_s`.
# ---------------------------------------------------------------------------


class HoldButton:
    def __init__(self, hold_s: float):
        self.hold_s = hold_s
        self._pressed_at = 0.0  # 0 = up
        self._fired = False

    def reset(self) -> None:
        self._pressed_at = 0.0
        self._fired = False

    def update(self, pressed: bool, now: float) -> bool:
        """Feed the button state each tick; True on the tick the hold completes."""
        if not pressed:
            self.reset()
            return False
        if self._pressed_at == 0.0:
            self._pressed_at = now
            return False
        if not self._fired and now - self._pressed_at >= self.hold_s:
            self._fired = True
            return True
        return False


# ---------------------------------------------------------------------------
# Fork geometry
# ---------------------------------------------------------------------------


def quat_rotate(q_xyzw: Sequence[float], v: Sequence[float]) -> Vec3:
    """Rotate vector `v` by unit quaternion `q` (x, y, z, w)."""
    x, y, z, w = q_xyzw
    cx, cy, cz = y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (
        v[0] + 2.0 * (w * cx + ccx),
        v[1] + 2.0 * (w * cy + ccy),
        v[2] + 2.0 * (w * cz + ccz),
    )


def tilt_axis(
    fork_axis_tool: Sequence[float], quat_xyzw: Sequence[float]
) -> Optional[Vec3]:
    """Base-frame unit axis about which a POSITIVE rotation raises the fork tip.

    That is the horizontal axis perpendicular to the fork's horizontal direction,
    `fork_h x z`. None when the fork points straight up or down.
    """
    f = quat_rotate(quat_xyzw, fork_axis_tool)
    n = math.hypot(f[0], f[1])
    if n < 1e-6:
        return None
    return (f[1] / n, -f[0] / n, 0.0)


# ---------------------------------------------------------------------------
# Stab, then lift while tilting the tines up.
# ---------------------------------------------------------------------------


@dataclass
class ScoopParams:
    depth_m: float  # stab: move straight down (base -z) this far
    tilt_deg: float  # then raise the tip by this angle (negative = lower it)
    lift_m: float  # while lifting straight up (base +z) this far
    linear_speed: float  # m/s for the stab and the lift
    angular_speed: float  # rad/s for the tilt


def _leg(rate: float, remaining: float, dt: float) -> float:
    """Signed rate for this tick, scaled down on the last tick so the leg ends exactly."""
    if remaining <= 0.0:
        return 0.0
    return rate * min(1.0, remaining / dt) if dt > 0 else 0.0


class ScoopMacro:
    def __init__(self, params: ScoopParams, axis: Vec3):
        self.p = params
        self.axis = axis
        self._t = 0.0
        v, w = params.linear_speed, params.angular_speed
        self.t_stab = abs(params.depth_m) / v if v > 0 else 0.0
        self.t_lift = abs(params.lift_m) / v if v > 0 else 0.0
        self.t_tilt = math.radians(abs(params.tilt_deg)) / w if w > 0 else 0.0
        self.t_end = self.t_stab + max(self.t_lift, self.t_tilt)

    @property
    def done(self) -> bool:
        return self._t >= self.t_end

    @property
    def phase(self) -> str:
        if self.done:
            return "done"
        return "stab" if self._t < self.t_stab else "scoop"

    def step(self, dt: float) -> Tuple[Vec3, Vec3]:
        """Advance by `dt` and return the (linear, angular) base-frame twist to stream."""
        t = self._t
        self._t = t + dt
        p = self.p
        if t < self.t_stab:
            vz = -math.copysign(p.linear_speed, p.depth_m)
            return (0.0, 0.0, _leg(vz, self.t_stab - t, dt)), ZERO3
        u = t - self.t_stab
        vz = _leg(math.copysign(p.linear_speed, p.lift_m), self.t_lift - u, dt)
        wt = _leg(math.copysign(p.angular_speed, p.tilt_deg), self.t_tilt - u, dt)
        ang = tuple(a * wt for a in self.axis) if wt != 0.0 else ZERO3
        return (0.0, 0.0, vz), ang
