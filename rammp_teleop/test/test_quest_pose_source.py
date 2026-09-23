import numpy as np

from rammp_teleop.quest.pose_source import (
    MockPoseSource,
    OculusPoseSource,
    keep_headset_awake,
    parse_oculus_sample,
)


# Real capture from a Quest controller (see chat log). Right controller held at
# rest: thumb up, nothing pressed. Both transforms are proper SO(3) rotations.
SAMPLE_TRANSFORMS = {
    "l": np.array(
        [
            [-0.90386, -0.368162, -0.217932, 0.0979487],
            [0.0223743, 0.468015, -0.883437, 0.155136],
            [0.427244, -0.803379, -0.414783, 0.0535833],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
    "r": np.array(
        [
            [0.556341, -0.577223, 0.597744, 0.552038],
            [0.830682, 0.367931, -0.417844, -0.0277351],
            [0.02126, 0.728999, 0.684184, 0.0204228],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
}
SAMPLE_BUTTONS = {
    "A": False,
    "B": False,
    "RThU": True,
    "RJ": False,
    "RG": False,
    "RTr": False,
    "X": False,
    "Y": False,
    "LThU": True,
    "LJ": False,
    "LG": False,
    "LTr": False,
    "leftJS": (0.0, 0.0),
    "leftTrig": (0.0,),
    "leftGrip": (0.0,),
    "rightJS": (0.0, 0.0),
    "rightTrig": (0.0,),
    "rightGrip": (0.0,),
}


class FakeReader:
    """Stand-in for oculus_reader.OculusReader: replays canned snapshots."""

    def __init__(self, samples):
        self.samples = samples
        self._i = 0

    def get_transformations_and_buttons(self):
        s = self.samples[min(self._i, len(self.samples) - 1)]
        self._i += 1
        return s


def test_mock_is_deterministic():
    b = MockPoseSource()
    c = MockPoseSource()
    for _ in range(50):
        pb, bb = b.read()
        pc, bc = c.read()
        assert np.allclose(pb, pc)
        assert bb == bc


def test_mock_engage_window():
    dt = 1.0 / 60.0
    src = MockPoseSource(dt=dt)
    grips = []
    for _ in range(len(src.script)):
        _, btn = src.read()
        grips.append(btn.grip)
    # Grip should be released, then engaged in the middle, then released.
    assert grips[0] is False
    assert grips[-1] is False
    assert any(grips), "grip never engaged"
    # First engaged frame is around t=0.5s -> index 30.
    first = grips.index(True)
    assert 28 <= first <= 32


def test_mock_motion_only_during_engage():
    src = MockPoseSource()
    frames = src.script
    # Position is zero before engage, nonzero by the end of the engaged window.
    assert np.allclose(frames[0].pos, 0)
    engaged = [f for f in frames if f.grip]
    assert np.linalg.norm(engaged[-1].pos) > 0.1


def test_mock_trigger_closes_gripper():
    src = MockPoseSource()
    triggers = [f.trigger for f in src.script]
    assert triggers[0] == 0.0
    assert triggers[-1] == 1.0


def test_mock_repeats_last_frame_when_done():
    src = MockPoseSource()
    for _ in range(len(src.script)):
        src.read()
    assert src.done
    p1, _ = src.read()
    p2, _ = src.read()
    assert np.allclose(p1, p2)  # holds final frame, no crash


# --- Quest sample parsing --------------------------------------------------


def test_parse_returns_raw_right_controller_pose():
    pose, btn = parse_oculus_sample(SAMPLE_TRANSFORMS, SAMPLE_BUTTONS, hand="r")
    assert np.allclose(pose, SAMPLE_TRANSFORMS["r"])  # raw pose, no axis remap
    assert btn.grip is False  # RG not pressed -> deadman released
    assert btn.trigger == 0.0


def test_parse_left_controller_uses_left_keys():
    pose, btn = parse_oculus_sample(SAMPLE_TRANSFORMS, SAMPLE_BUTTONS, hand="l")
    assert np.allclose(pose, SAMPLE_TRANSFORMS["l"])
    assert btn.grip is False


def test_parse_grip_is_the_deadman():
    buttons = dict(SAMPLE_BUTTONS, RG=True)
    _, btn = parse_oculus_sample(SAMPLE_TRANSFORMS, buttons, hand="r")
    assert btn.grip is True


def test_parse_trigger_passes_through_analog_gripper_value():
    buttons = dict(SAMPLE_BUTTONS, rightTrig=(0.73,))
    _, btn = parse_oculus_sample(SAMPLE_TRANSFORMS, buttons, hand="r")
    assert btn.trigger == 0.73


def test_parse_missing_hand_is_a_safe_deadman_default():
    # Hand transform absent (startup, or tracking dropped) -> no pose, released.
    pose, btn = parse_oculus_sample({}, {}, hand="r")
    assert pose is None
    assert btn.grip is False
    assert btn.trigger == 0.0


def test_parse_missing_hand_forces_grip_released_even_if_button_says_held():
    # Stale button with no pose must NOT engage the clutch (deadman safety).
    pose, btn = parse_oculus_sample({}, dict(SAMPLE_BUTTONS, RG=True), hand="r")
    assert pose is None
    assert btn.grip is False


# --- OculusPoseSource ------------------------------------------------------


def test_oculus_source_reads_pose_and_buttons():
    src = OculusPoseSource(
        hand="r", reader=FakeReader([(SAMPLE_TRANSFORMS, SAMPLE_BUTTONS)])
    )
    pose, btn = src.read()
    assert pose.shape == (4, 4)
    assert np.allclose(pose, SAMPLE_TRANSFORMS["r"])
    assert btn.grip is False


def test_oculus_source_returns_valid_pose_before_any_data():
    # oculus_reader yields ({}, {}) until the first logcat line arrives.
    src = OculusPoseSource(hand="r", reader=FakeReader([({}, {})]))
    pose, btn = src.read()
    assert pose.shape == (4, 4)  # never None: loop indexes pose[:3, 3]
    assert btn.grip is False


def test_oculus_source_holds_last_pose_and_freezes_when_tracking_drops():
    held = dict(SAMPLE_BUTTONS, RG=True)  # grip held while tracked
    reader = FakeReader(
        [
            (SAMPLE_TRANSFORMS, held),  # tracked + engaged
            ({}, held),  # tracking dropped, button stale
        ]
    )
    src = OculusPoseSource(hand="r", reader=reader)
    pose1, btn1 = src.read()
    assert btn1.grip is True
    pose2, btn2 = src.read()
    assert np.allclose(pose2, pose1)  # holds last good pose
    assert btn2.grip is False  # but releases the clutch -> freeze


# --- keep-awake: defeat the proximity sensor so the headset keeps tracking ---


class FakeShell:
    """Records adb shell commands; answers ``dumpsys power`` with a canned state."""

    def __init__(self, wakefulness="Awake"):
        self.wakefulness = wakefulness
        self.commands = []

    def __call__(self, cmd):
        self.commands.append(cmd)
        if cmd.startswith("dumpsys power"):
            return f"  mWakefulness={self.wakefulness}\n"
        return ""


def test_keep_awake_wakes_disables_prox_and_stays_on():
    shell = FakeShell()
    assert keep_headset_awake(shell) is True
    assert shell.commands[:3] == [
        "input keyevent KEYCODE_WAKEUP",
        "am broadcast -a com.oculus.vrpowermanager.prox_close",
        "svc power stayon usb",
    ]


def test_keep_awake_reports_headset_still_asleep():
    assert keep_headset_awake(FakeShell(wakefulness="Asleep")) is False


def test_oculus_source_keeps_headset_awake_on_construction():
    class Device:
        def __init__(self):
            self.shell = FakeShell()

    reader = FakeReader([(SAMPLE_TRANSFORMS, SAMPLE_BUTTONS)])
    reader.device = Device()
    src = OculusPoseSource(hand="r", reader=reader)
    assert (
        "am broadcast -a com.oculus.vrpowermanager.prox_close"
        in reader.device.shell.commands
    )
    assert src.awake is True


def test_oculus_source_keep_awake_off_sends_nothing():
    class Device:
        def __init__(self):
            self.shell = FakeShell()

    reader = FakeReader([(SAMPLE_TRANSFORMS, SAMPLE_BUTTONS)])
    reader.device = Device()
    src = OculusPoseSource(hand="r", reader=reader, keep_awake=False)
    assert reader.device.shell.commands == []
    assert src.awake is None


# --- app watchdog: the headset app dies silently and oculus_reader never notices ---


class AppDevice:
    """adb device whose ``pidof`` answers ``running`` and which records am start."""

    def __init__(self, running: bool):
        self.running = running
        self.commands = []

    def shell(self, cmd):
        self.commands.append(cmd)
        if cmd.startswith("pidof "):
            return "12345\n" if self.running else ""
        return ""


def _source_with_device(running: bool) -> tuple[OculusPoseSource, AppDevice]:
    reader = FakeReader([(SAMPLE_TRANSFORMS, SAMPLE_BUTTONS)])
    reader.device = AppDevice(running)
    reader.APK_name = "com.rail.oculus.teleop"
    src = OculusPoseSource(hand="r", reader=reader, keep_awake=False, app_watchdog_s=0)
    return src, reader.device


def test_watchdog_relaunches_dead_app():
    src, dev = _source_with_device(running=False)
    assert src.check_app() is True
    assert any(
        c.startswith("am start -n") and "com.rail.oculus.teleop" in c
        for c in dev.commands
    )
    assert src.app_relaunches == 1


def test_watchdog_leaves_running_app_alone():
    src, dev = _source_with_device(running=True)
    assert src.check_app() is False
    assert not any(c.startswith("am start") for c in dev.commands)
    assert src.app_relaunches == 0
