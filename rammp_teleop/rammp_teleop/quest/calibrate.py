"""Calibrate ``R_align`` — the one controller->base rotation knob.

The Quest reports controller poses in its own (headset boot-time) tracking
frame, which has no fixed relationship to the robot base frame. ``R_align``
rotates controller-frame motion into base-frame motion so that "hand forward =
EE forward". Because the Quest frame's yaw depends on how the headset was
oriented at startup, this is a per-session calibration.

Procedure: the operator moves the controller along each robot base axis in turn
(+X forward, +Y left, +Z up) while holding the grip clutch. We capture the
controller-frame displacement for each and solve for the rotation that best maps
those three observed directions onto the base axes (orthogonal Procrustes, which
also cleans up the fact that hand gestures are never perfectly orthogonal).
"""

from __future__ import annotations

import time

import numpy as np
from scipy.spatial.transform import Rotation


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("gesture displacement is ~zero; move the hand farther")
    return v / n


def r_align_from_gestures(fwd, left, up) -> Rotation:
    """Controller->base rotation from three calibration gestures.

    ``fwd``/``left``/``up`` are the controller-frame displacement vectors
    observed while moving the hand along base +X / +Y / +Z respectively (only
    their directions matter). Returns ``R`` such that ``R.apply(delta_ctrl)`` is
    the corresponding base-frame motion.
    """
    # Columns = observed controller-frame directions (A); targets B = base axes.
    A = np.column_stack([_unit(fwd), _unit(left), _unit(up)])
    B = np.eye(3)
    # Orthogonal Procrustes: minimize ||R A - B||. H = A B^T = A (since B = I).
    U, _, Vt = np.linalg.svd(A @ B.T)
    V = Vt.T
    d = np.sign(np.linalg.det(V @ U.T))
    R = V @ np.diag([1.0, 1.0, d]) @ U.T
    return Rotation.from_matrix(R)


def euler_zyx_deg(rot: Rotation) -> list[float]:
    """Config-ready ZYX euler degrees (matches ``r_align_euler_zyx_deg``)."""
    return [round(float(a), 4) for a in rot.as_euler("ZYX", degrees=True)]


# --- interactive capture ----------------------------------------------------

def _capture_gesture(source, label: str) -> np.ndarray:
    """Wait for a grip-held move and return the controller-frame displacement.

    Squeeze the grip, move along ``label``, release. Start pos is sampled at the
    grip rising edge, end pos at the falling edge.
    """
    print(f"\n  >>> {label}")
    print("      squeeze GRIP, move the hand, then RELEASE grip.")
    # Wait for rising edge.
    prev = False
    start = None
    while True:
        pose, btn = source.read()
        if btn.grip and not prev:
            start = np.asarray(pose[:3, 3], float).copy()
            print("      [capturing... release grip when done]")
        if not btn.grip and prev and start is not None:
            end = np.asarray(pose[:3, 3], float)
            delta = end - start
            print(f"      captured |delta|={np.linalg.norm(delta):.3f} m")
            return delta
        prev = btn.grip
        time.sleep(1.0 / 60.0)


def run_calibration(source) -> Rotation:
    """Walk the operator through the three gestures and print the result."""
    print("=" * 64)
    print("R_align calibration — base frame is +X forward, +Y left, +Z up.")
    print("Make each move ~20 cm so the direction is clear.")
    print("=" * 64)
    fwd = _capture_gesture(source, "Move hand FORWARD (robot +X, away from base)")
    left = _capture_gesture(source, "Move hand LEFT (robot +Y)")
    up = _capture_gesture(source, "Move hand UP (robot +Z)")
    R = r_align_from_gestures(fwd, left, up)
    ez = euler_zyx_deg(R)
    print("\n" + "=" * 64)
    print("Calibration complete. Put this in your YAML config:")
    print(f"\n  r_align_euler_zyx_deg: [{ez[0]}, {ez[1]}, {ez[2]}]\n")
    print("=" * 64)
    return R


def main(argv=None) -> int:
    import argparse

    from .pose_source import OculusPoseSource

    ap = argparse.ArgumentParser(description="Calibrate R_align from the Quest")
    ap.add_argument("--hand", default="r", choices=["r", "l"])
    ap.add_argument("--quest-ip", default=None,
                    help="Quest IP for ADB-over-network (default: USB)")
    args = ap.parse_args(argv)

    source = OculusPoseSource(hand=args.hand, ip_address=args.quest_ip)
    try:
        run_calibration(source)
    except KeyboardInterrupt:
        print("\n[calibrate] aborted")
        return 1
    finally:
        source.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
