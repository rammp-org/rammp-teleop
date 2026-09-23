"""Pose sources: the one abstraction the rest of the supervisor consumes.

A ``PoseSource`` yields a controller pose (4x4, in the source's own frame) plus
button states. The mock and the real Quest are drop-in swappable behind this
interface; downstream code never knows which is behind it.
"""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from .transforms import make_pose


@dataclass
class Buttons:
    """Controller button/axis state.

    ``grip`` is the clutch deadman (engage when held). ``trigger`` drives the
    gripper (0=open .. 1=closed). Extra buttons can be added without changing
    the interface.
    """

    grip: bool = False
    trigger: float = 0.0
    extra: dict = field(default_factory=dict)


class PoseSource(ABC):
    """Source of controller poses + buttons."""

    @abstractmethod
    def read(self) -> tuple[np.ndarray, Buttons]:
        """Return (controller_pose_4x4, buttons). Non-blocking; latest sample."""

    def close(self) -> None:  # optional cleanup hook
        pass


@dataclass
class ScriptFrame:
    pos: np.ndarray
    rot: Rotation
    grip: bool
    trigger: float


def default_script(dt: float = 1.0 / 60.0) -> list[ScriptFrame]:
    """A deterministic engage -> move -> grip -> disengage trajectory.

    Timeline (60 Hz):
      - 0.00-0.50s: idle, grip released, controller at origin pose.
      - 0.50s:      grip engaged (clutch reference captured downstream).
      - 0.50-1.50s: controller translates +0.15 m in x, +0.05 m in z and yaws
                    +30 deg, smoothly (this is the motion the EE should follow).
      - 1.00s:      trigger closes (gripper).
      - 1.50s:      grip released (disengage; EE should freeze in place).
      - 1.50-2.00s: idle again.
    """
    frames: list[ScriptFrame] = []
    n = int(round(2.0 / dt))
    t_engage, t_release = 0.5, 1.5
    t_grip = 1.0
    for i in range(n):
        t = i * dt
        engaged = t_engage <= t < t_release
        # Smooth 0..1 ramp across the engaged window for the controller motion.
        if t < t_engage:
            s = 0.0
        elif t < t_release:
            s = (t - t_engage) / (t_release - t_engage)
            s = 0.5 - 0.5 * np.cos(np.pi * s)  # cosine ease-in/out
        else:
            s = 1.0
        pos = np.array([0.15 * s, 0.0, 0.05 * s])
        rot = Rotation.from_euler("z", 30.0 * s, degrees=True)
        trigger = 1.0 if t >= t_grip else 0.0
        frames.append(ScriptFrame(pos=pos, rot=rot, grip=engaged, trigger=trigger))
    return frames


class MockPoseSource(PoseSource):
    """Replays a deterministic scripted trajectory, one frame per ``read()``.

    ``read()`` past the end of the script repeats the final frame (held), so a
    consumer loop never crashes; ``done`` reports whether the script is
    exhausted.
    """

    def __init__(self, script: list[ScriptFrame] | None = None, dt: float = 1.0 / 60.0):
        self.script = script if script is not None else default_script(dt)
        self.dt = dt
        self._i = 0

    @property
    def done(self) -> bool:
        return self._i >= len(self.script)

    def read(self) -> tuple[np.ndarray, Buttons]:
        idx = min(self._i, len(self.script) - 1)
        f = self.script[idx]
        self._i += 1
        pose = make_pose(f.pos, f.rot)
        return pose, Buttons(grip=f.grip, trigger=f.trigger)

    def reset(self) -> None:
        self._i = 0


def parse_oculus_sample(
    transforms: dict,
    buttons: dict,
    hand: str = "r",
) -> tuple[np.ndarray | None, Buttons]:
    """Decode one ``oculus_reader`` snapshot into ``(pose_4x4, Buttons)``.

    The controller pose is returned **raw**, in the headset's own right-handed
    frame (verified det=+1, orthonormal). No axis remap happens here: the
    clutched delta-pose mapper applies ``R_align`` (controller->base) as the one
    calibration knob, so a fixed conversion would just be a redundant second
    knob fighting it.

    ``hand`` is ``"r"`` or ``"l"``. ``grip`` (the squeeze trigger, ``RG``/``LG``)
    is the clutch deadman; the index ``trigger`` (analog ``rightTrig``/
    ``leftTrig`` in 0..1) drives the gripper.

    Deadman safety: if the hand's transform is absent (startup before the first
    sample, or tracking dropped), this returns ``(None, Buttons(grip=False))`` —
    the clutch is forced released regardless of any stale button state, so the
    robot freezes rather than chasing a frozen pose.
    """
    grip_key = "RG" if hand == "r" else "LG"
    trig_key = "rightTrig" if hand == "r" else "leftTrig"

    pose = transforms.get(hand)
    if pose is None:
        return None, Buttons(grip=False, trigger=0.0)

    trig = buttons.get(trig_key) or (0.0,)
    return np.asarray(pose, dtype=float), Buttons(
        grip=bool(buttons.get(grip_key, False)),
        trigger=float(trig[0]),
        extra=dict(buttons),
    )


KEEP_AWAKE_COMMANDS = (
    "input keyevent KEYCODE_WAKEUP",  # wake now
    "am broadcast -a com.oculus.vrpowermanager.prox_close",  # pretend it is worn
    "svc power stayon usb",  # no screen-off timeout while on USB
)


def keep_headset_awake(shell) -> bool:
    """Defeat the Quest's proximity sensor so it keeps tracking when not worn.

    ``shell`` is an adb shell callable (``ppadb`` ``Device.shell``). The
    ``prox_close`` broadcast does not survive a headset reboot, so this runs on
    every connect. Returns whether the headset reports itself awake afterwards.
    Undo by hand with ``am broadcast -a com.oculus.vrpowermanager.automation_disable``.
    """
    for cmd in KEEP_AWAKE_COMMANDS:
        shell(cmd)
    return "mWakefulness=Awake" in (shell("dumpsys power | grep mWakefulness=") or "")


class OculusPoseSource(PoseSource):
    """Real Meta Quest source: wraps ``oculus_reader`` over ADB (~70 Hz).

    The reader runs its own background thread polling the headset; ``read()`` is
    non-blocking and returns the latest snapshot. The raw controller pose is
    passed straight through (see :func:`parse_oculus_sample`); base alignment is
    the mapper's ``R_align``. When no pose is available the last good pose is
    held but the clutch is forced released (deadman freeze).

    ``reader`` is injectable for testing; left ``None`` it constructs a real
    ``OculusReader`` (imported lazily so this module imports on hosts without
    ADB / the headset package installed). With ``keep_awake`` the headset's
    proximity sensor is defeated on connect (see :func:`keep_headset_awake`); ``awake``
    records the result, ``None`` when nothing was sent.

    The headset app can die silently (oculus_reader launches it once and then
    only tails logcat, so the last pose is repeated forever). With
    ``app_watchdog_s`` > 0 a daemon thread checks every that-many seconds that
    the app process exists and relaunches it if not; ``app_relaunches`` counts.
    """

    APP_START = (
        'am start -n "{pkg}/{pkg}.MainActivity" -a android.intent.action.MAIN '
        "-c android.intent.category.LAUNCHER"
    )

    def __init__(
        self,
        hand: str = "r",
        reader=None,
        ip_address: str | None = None,
        keep_awake: bool = True,
        app_watchdog_s: float = 2.0,
    ):
        if hand not in ("r", "l"):
            raise ValueError(f"hand must be 'r' or 'l', got {hand!r}")
        self.hand = hand
        if reader is None:
            from oculus_reader.reader import OculusReader  # lazy: needs ADB

            reader = OculusReader(ip_address=ip_address)
        self.reader = reader
        self.awake: bool | None = None
        device = getattr(reader, "device", None)
        if keep_awake and device is not None:
            self.awake = keep_headset_awake(device.shell)
        self._last_pose: np.ndarray | None = None
        self.app_relaunches = 0
        self._stop = threading.Event()
        if app_watchdog_s > 0 and device is not None:
            threading.Thread(
                target=self._watchdog, args=(app_watchdog_s,), daemon=True
            ).start()

    def check_app(self) -> bool:
        """Relaunch the headset app if its process is gone. True if relaunched."""
        pkg = getattr(self.reader, "APK_name", "com.rail.oculus.teleop")
        if (self.reader.device.shell(f"pidof {pkg}") or "").strip():
            return False
        self.reader.device.shell(self.APP_START.format(pkg=pkg))
        self.app_relaunches += 1
        return True

    def _watchdog(self, period_s: float) -> None:
        while not self._stop.wait(period_s):
            try:
                self.check_app()
            except Exception:  # adb hiccup; try again next period
                pass

    def read(self) -> tuple[np.ndarray, Buttons]:
        transforms, buttons = self.reader.get_transformations_and_buttons()
        pose, btn = parse_oculus_sample(transforms or {}, buttons or {}, self.hand)
        if pose is not None:
            self._last_pose = pose
        elif self._last_pose is None:
            self._last_pose = np.eye(4)  # before first sample: valid identity
        return self._last_pose.copy(), btn

    def close(self) -> None:
        self._stop.set()
        stop = getattr(self.reader, "stop", None)
        if callable(stop):
            stop()
