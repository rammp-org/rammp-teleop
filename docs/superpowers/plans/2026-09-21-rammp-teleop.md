# rammp-teleop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A RAMMP module repo with one ament_python package, `rammp_teleop`, giving Xbox-pad and Quest 3 teleop of the Kinova Gen3 through `kinova-gen3-ros2`'s streaming tier, buildable on abra now and as a rammp-base image later, plus a local sheppy manifest edit that runs it from the abra checkout.

**Architecture:** A shared `TeleopNodeBase` owns the arm session (acquire, open stream, publish `PoseSetpoint`/`JointSetpoint` every tick, gripper, e-stop, recovery, teardown); each input is a thin subclass that turns its device stream into a target. Xbox logic is lifted from abra's `kinova_xbox_teleop`; Quest mapping/safety are lifted from `kinova-quest-teleop@main` behind a pure `QuestCommand` class, fed by a `quest_reader` node that republishes the headset as ROS topics.

**Tech Stack:** ROS 2 Humble, rclpy, ament_python, `rammp_arm_interfaces`/`rammp_common_interfaces` v1.0.0, numpy, scipy (Quest only), ros-humble-joy, rail-berkeley `oculus_reader` (Quest only), Docker on `ghcr.io/rammp-org/rammp-base:1.0.0-jp6`, sheppy manifests, hardware-loop (`hil`) to abra.

**Spec:** `docs/superpowers/specs/2026-09-21-rammp-teleop-design.md`

## Global Constraints

- Python 3.10 (abra); every module keeps `from __future__ import annotations`.
- Package name `rammp_teleop`; node names `xbox_teleop`, `quest_teleop`, `quest_reader`; console scripts `xbox_teleop`, `quest_teleop`, `quest_reader`, `quest_calibrate`.
- Device libraries are never imported at module import time: `oculus_reader` only inside `OculusPoseSource.__init__`; `joy` is an `exec_depend` only.
- Setpoint QoS is BEST_EFFORT / KEEP_LAST / depth 1. `stream_timeout_s` must exceed `1/rate_hz`.
- Quaternions inside `logic.py` and on every ROS message are (x, y, z, w). The ported Quest `transforms.py` keeps its wxyz helpers for its own tests; nothing new uses wxyz.
- Xbox default controller `ee_pose_position` at 50 Hz; Quest default `ee_pose_impedance` at 60 Hz.
- No `sheppy up`, `sheppy down` or `sheppy woof` on abra. Off-arm checks on abra use `ROS_DOMAIN_ID=42`.
- All ROS builds/tests run on abra through `uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec --target dojo -- bash -c '...'` (or without `--target` from a directory holding `.hil.yml`). Never hand-build ssh.
- Local pure-Python tests run with `cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test -q`.
- Commits end with the attribution trailer from the session reminder.
- Source paths used below:
  - XBOX = `/tmp/claude-1000/-home-swapnil-atdev-rammp-teleop/45d075a5-4ad6-4bb6-8f02-adefc30cd64b/scratchpad/abra/kinova-xbox-teleop` (pulled from abra; if missing, re-pull with `hil.py pull --target dojo --to <dir> ros2_ws/src/kinova-xbox-teleop`)
  - QUEST = `/home/swapnil/atdev/kinova-quest-teleop` (at `origin/main` 0bc99be)
  - TEMPLATE = `/home/swapnil/atdev/rammp-module-template`
  - DEPLOY = `/home/swapnil/atdev/rammp-deployments`

---

## File map

| Path | Responsibility |
|---|---|
| `rammp_teleop/package.xml`, `setup.py`, `setup.cfg`, `resource/rammp_teleop` | ament_python packaging, four console scripts |
| `rammp_teleop/rammp_teleop/logic.py` | deadzone, xyzw quaternion helpers, `XboxMap`, `CartesianIntegrator`, `JointIntegrator`, `GripperIntegrator` (verbatim from XBOX `teleop_logic.py`) |
| `rammp_teleop/rammp_teleop/session.py` | `TeleopNodeBase`, `pose_msg`, `run` |
| `rammp_teleop/rammp_teleop/xbox_node.py` | `XboxTeleopNode` |
| `rammp_teleop/rammp_teleop/quest/{transforms,mapping,safety,pose_source,scenarios,calibrate}.py` | verbatim ports from QUEST |
| `rammp_teleop/rammp_teleop/quest/command.py` | `QuestCommand`: mapper -> safety -> hold, no ROS |
| `rammp_teleop/rammp_teleop/quest_reader.py` | `QuestReaderNode`: headset -> `/quest/<hand>/pose` + `/quest/joy` |
| `rammp_teleop/rammp_teleop/quest_node.py` | `QuestTeleopNode` |
| `rammp_teleop/launch/xbox.launch.py`, `quest.launch.py` | input node + teleop node |
| `rammp_teleop/config/xbox.yaml`, `quest.yaml` | parameters |
| `rammp_teleop/test/*.py` | pure-logic tests, no ROS |
| `Dockerfile`, `.dockerignore`, `rammp-alternative.*.yaml`, `Makefile`, `scripts/`, `tests/`, `.github/`, `.pre-commit-config.yaml` | RAMMP module scaffolding |
| `docs/interface.md`, `docs/index.mdx`, `docs/_meta.js`, `README.md` | docs |
| `.hil.yml`, `.gitignore` | sync to abra, ignores |
| DEPLOY `december_2026/sheppy-manifest.yaml`, `december_2026/profiles/teleop-*.yaml` | local, uncommitted deployment edit |

---

### Task 1: Package skeleton and Xbox logic port

**Files:**
- Create: `rammp_teleop/package.xml`, `rammp_teleop/setup.py`, `rammp_teleop/setup.cfg`, `rammp_teleop/resource/rammp_teleop`, `rammp_teleop/rammp_teleop/__init__.py`, `rammp_teleop/rammp_teleop/logic.py`, `.gitignore`
- Test: `rammp_teleop/test/test_logic.py`

**Interfaces:**
- Produces: `rammp_teleop.logic` with `apply_deadzone`, `trigger_amount`, `clamp`, `quat_normalize`, `quat_mul`, `quat_conj`, `quat_from_rotvec`, `quat_to_rotvec`, `quat_angle`, `XboxMap` (fields `axis_left_x..axis_dpad_y`, `button_deadman`, `button_estop`, `button_estop_clear`, `button_resync`; methods `deadman(buttons)`, `pressed(buttons, idx)`, `cartesian_command(axes, deadzone) -> (Vec3, Vec3)`, `joint_jog_command(axes, deadzone) -> float`, `joint_select_step(axes) -> int`, `gripper_command(axes) -> (close, open)`), `CartesianIntegrator(max_linear, max_angular, lead_m, lead_rad)` with `.position`, `.orientation`, `reset(p, q)`, `step(lin, ang, dt, actual_p, actual_q)`, `JointIntegrator(max_speed, lead_rad)` with `.positions`, `.selected`, `reset`, `select_next(step)`, `step(rate, dt, actual)`, `GripperIntegrator(speed)` with `.position`, `reset`, `step(close, open, dt) -> bool`.

- [ ] **Step 1: Create the package files**

```bash
cd /home/swapnil/atdev/rammp-teleop
mkdir -p rammp_teleop/rammp_teleop rammp_teleop/resource rammp_teleop/test rammp_teleop/launch rammp_teleop/config
touch rammp_teleop/resource/rammp_teleop rammp_teleop/rammp_teleop/__init__.py
```

`rammp_teleop/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rammp_teleop</name>
  <version>0.1.0</version>
  <description>Operator-input teleop (Xbox pad, Meta Quest 3) for the Kinova Gen3 via kinova_gen3_ros2's streaming tier.</description>
  <maintainer email="swapnil.pande98@gmail.com">Swapnil Pande</maintainer>
  <license>Proprietary</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>rammp_arm_interfaces</depend>
  <depend>rammp_common_interfaces</depend>
  <depend>python3-numpy</depend>
  <exec_depend>python3-scipy</exec_depend>
  <exec_depend>joy</exec_depend>
  <exec_depend>ros2launch</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`rammp_teleop/setup.py`:

```python
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rammp_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Swapnil Pande",
    maintainer_email="swapnil.pande98@gmail.com",
    description="Xbox and Quest 3 teleop for the Kinova Gen3 via the streaming tier.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"xbox_teleop = {package_name}.xbox_node:main",
            f"quest_teleop = {package_name}.quest_node:main",
            f"quest_reader = {package_name}.quest_reader:main",
            f"quest_calibrate = {package_name}.quest.calibrate:main",
        ],
    },
)
```

`rammp_teleop/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/rammp_teleop
[install]
install_scripts=$base/lib/rammp_teleop
```

`.gitignore` (repo root):

```
__pycache__/
*.pyc
.pytest_cache/
.ruff_cache/
build/
install/
log/
*.egg-info/
.superpowers/
```

- [ ] **Step 2: Port the logic module and its tests**

```bash
XBOX=/tmp/claude-1000/-home-swapnil-atdev-rammp-teleop/45d075a5-4ad6-4bb6-8f02-adefc30cd64b/scratchpad/abra/kinova-xbox-teleop
cp "$XBOX/kinova_xbox_teleop/teleop_logic.py" rammp_teleop/rammp_teleop/logic.py
cp "$XBOX/test/test_teleop_logic.py" rammp_teleop/test/test_logic.py
sed -i 's/kinova_xbox_teleop\.teleop_logic/rammp_teleop.logic/' rammp_teleop/test/test_logic.py
sed -i '1s/.*/"""Pure-Python teleop logic: Xbox mapping, deadzone, quaternions, integrators./' rammp_teleop/rammp_teleop/logic.py
```

Confirm the only edits versus the source are the import path and the docstring first line (`diff` both files against XBOX).

- [ ] **Step 3: Run the tests**

Run: `cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test -q`
Expected: all tests in `test_logic.py` pass (the source file had 20+ tests; count them in the output).

- [ ] **Step 4: Commit**

```bash
git add .gitignore rammp_teleop
git commit -m "feat: rammp_teleop package skeleton with the Xbox logic port"
```

---

### Task 2: Quest pure-module port

**Files:**
- Create: `rammp_teleop/rammp_teleop/quest/__init__.py`, `transforms.py`, `mapping.py`, `safety.py`, `pose_source.py`, `scenarios.py`, `calibrate.py`
- Test: `rammp_teleop/test/test_quest_transforms.py`, `test_quest_mapping.py`, `test_quest_safety.py`, `test_quest_pose_source.py`, `test_quest_calibrate.py`, `test_quest_scenarios_build.py`

**Interfaces:**
- Produces (unchanged from QUEST): `MappingConfig(R_align, trans_smooth, rot_smooth, jump_pos_tol, jump_rot_tol, pos_scale, rot_frame, R_tool)`, `ClutchedDeltaMapper(cfg).update(ctrl_pose_4x4, grip, ee_pos, ee_rot) -> MapResult(pos, rot, engaged, rejected)`, `SafetyConfig(ws_min, ws_max, max_lin_step, max_ang_step)`, `SafetyFilter(cfg).filter(pos, rot) -> SafetyResult(pos, rot, clamped_velocity, clamped_workspace)`, `SafetyFilter.reset(pos, rot)`, `Buttons(grip, trigger, extra)`, `MockPoseSource(script).read() -> (pose4x4, Buttons)`, `MockPoseSource.done`, `MockPoseSource.reset()`, `default_script(dt)`, `parse_oculus_sample(transforms, buttons, hand) -> (pose|None, Buttons)`, `OculusPoseSource(hand, reader=None, ip_address=None)`, `scenarios.get(name, dt)`, `scenarios.SCENARIOS`, `calibrate.main`, `make_pose(pos, rot)`, `slerp_rotation`, `angle_between`.

- [ ] **Step 1: Copy the modules and tests**

```bash
QUEST=/home/swapnil/atdev/kinova-quest-teleop
mkdir -p rammp_teleop/rammp_teleop/quest
for m in transforms mapping safety pose_source scenarios calibrate; do
  cp "$QUEST/src/kinova_teleop/$m.py" rammp_teleop/rammp_teleop/quest/$m.py
done
cat > rammp_teleop/rammp_teleop/quest/__init__.py <<'EOF'
"""Quest 3 controller -> EE target mapping, ported from rammp-org/kinova-quest-teleop.

Pure numpy/scipy; nothing here imports ROS. ``oculus_reader`` is imported lazily
inside ``OculusPoseSource`` so this package imports on hosts without adb.
"""
EOF
for t in transforms mapping safety pose_source calibrate; do
  cp "$QUEST/tests/test_$t.py" rammp_teleop/test/test_quest_$t.py
done
sed -i 's/from kinova_teleop\./from rammp_teleop.quest./; s/from kinova_teleop import/from rammp_teleop.quest import/' rammp_teleop/test/test_quest_*.py
```

The ported modules use relative imports (`from .transforms import ...`), so they need no edits. Verify: `grep -n "kinova_teleop" rammp_teleop/rammp_teleop/quest/*.py rammp_teleop/test/*.py` prints nothing.

- [ ] **Step 2: Add a scenarios build test (the original scenario test needs the UDP loop, which is not ported)**

`rammp_teleop/test/test_quest_scenarios_build.py`:

```python
"""Every scripted scenario builds and has the shape MockPoseSource expects."""
import numpy as np
import pytest

from rammp_teleop.quest import scenarios
from rammp_teleop.quest.pose_source import MockPoseSource


@pytest.mark.parametrize("name", sorted(scenarios.SCENARIOS))
def test_scenario_builds_and_replays(name):
    frames = scenarios.get(name)
    assert len(frames) > 10
    src = MockPoseSource(script=frames)
    pose, btn = src.read()
    assert pose.shape == (4, 4)
    assert np.isclose(np.linalg.det(pose[:3, :3]), 1.0)
    assert isinstance(btn.grip, bool)
    assert 0.0 <= btn.trigger <= 1.0


def test_unknown_scenario_raises():
    with pytest.raises((KeyError, ValueError, SystemExit)):
        scenarios.get("does-not-exist")
```

If `scenarios.get` raises something other than those three for an unknown name, read `scenarios.py:219` and match its actual exception.

- [ ] **Step 3: Run the tests**

Run: `cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test -q`
Expected: all pass. The ported Quest tests numbered 13 mapping + 14 pose_source + 5 safety + 6 transforms + 5 calibrate in the source repo.

- [ ] **Step 4: Commit**

```bash
git add rammp_teleop
git commit -m "feat: port Quest mapping, safety, pose source, scenarios and calibration"
```

---

### Task 3: `QuestCommand` (the loop's tick without a driver)

**Files:**
- Create: `rammp_teleop/rammp_teleop/quest/command.py`
- Test: `rammp_teleop/test/test_quest_command.py`

**Interfaces:**
- Consumes: Task 2 classes.
- Produces: `QuestCommand(mapping: MappingConfig, safety: SafetyConfig, gripper_binary: bool = False, gripper_threshold: float = 0.5)` with `update(ctrl_pose, grip, ee_pos, ee_rot) -> tuple[np.ndarray, Rotation, bool]` (target pos, target rot, engaged), `gripper(trigger: float) -> float`, `resync()`, `stats: dict` with keys `rejected`, `vel_clamped`, `ws_clamped`.

- [ ] **Step 1: Write the failing tests**

`rammp_teleop/test/test_quest_command.py`:

```python
"""QuestCommand: mapper -> safety -> hold-on-release, driven by scripted controllers.

The 'arm' here is a loopback: the measured EE pose is whatever was commanded last
tick, so smoothing and clamps are the only lag.
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rammp_teleop.quest import scenarios
from rammp_teleop.quest.command import QuestCommand
from rammp_teleop.quest.mapping import MappingConfig
from rammp_teleop.quest.pose_source import MockPoseSource, default_script
from rammp_teleop.quest.safety import SafetyConfig

EE0 = np.array([0.45, 0.0, 0.35])
ROT0 = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])  # tool pointing down


def run(cmd, source, ee_pos=EE0, ee_rot=ROT0, sag=None):
    """Replay the whole script; return list of (pos, rot, engaged)."""
    out = []
    while not source.done:
        pose, btn = source.read()
        pos, rot, engaged = cmd.update(pose, btn.grip, ee_pos, ee_rot)
        out.append((pos.copy(), rot, engaged))
        ee_pos, ee_rot = pos.copy(), rot
        if sag is not None and not engaged:
            ee_pos = ee_pos + sag  # a compliant arm droops when not driven
    return out


def make(**kw):
    return QuestCommand(MappingConfig(trans_smooth=0.8, rot_smooth=0.8), SafetyConfig(), **kw)


def test_idle_before_engage_holds_first_ee_pose():
    out = run(make(), MockPoseSource(default_script()))
    idle = [o for o in out[:20]]
    assert all(not e for _, _, e in idle)
    for p, _, _ in idle:
        assert np.allclose(p, EE0)


def test_hold_target_does_not_chase_sag():
    out = run(make(), MockPoseSource(default_script()), sag=np.array([0.0, 0.0, -0.001]))
    idle = out[:20]
    for p, _, _ in idle:
        assert np.allclose(p, EE0), "freeze target must be latched, not re-read from a sagging EE"


def test_engaged_motion_follows_script_in_base_frame():
    out = run(make(), MockPoseSource(default_script()))
    # default_script moves +0.15 x, +0.05 z, yaws 30 deg while engaged (0.5 s .. 1.5 s at 60 Hz)
    p_end, r_end, engaged = out[89]
    assert engaged
    assert p_end[0] - EE0[0] > 0.10
    assert p_end[2] - EE0[2] > 0.03
    assert abs(p_end[1] - EE0[1]) < 0.01
    yaw = (ROT0.inv() * r_end).magnitude()
    assert yaw > np.deg2rad(20)


def test_release_freezes_last_commanded_target():
    out = run(make(), MockPoseSource(default_script()))
    after = out[95:]
    assert all(not e for _, _, e in after)
    p_ref = after[0][0]
    for p, _, _ in after:
        assert np.allclose(p, p_ref)


def test_jump_glitch_is_rejected():
    cmd = make()
    run(cmd, MockPoseSource(scenarios.get("jump_glitch")))
    assert cmd.stats["rejected"] > 0


def test_workspace_escape_is_clamped_inside_box():
    cmd = make()
    out = run(cmd, MockPoseSource(scenarios.get("workspace_escape")))
    assert cmd.stats["ws_clamped"] > 0
    lo, hi = SafetyConfig().ws_min, SafetyConfig().ws_max
    for p, _, _ in out:
        assert np.all(p >= lo - 1e-9) and np.all(p <= hi + 1e-9)


def test_velocity_spike_is_rate_limited():
    cmd = make()
    out = run(cmd, MockPoseSource(scenarios.get("velocity_spike")))
    assert cmd.stats["vel_clamped"] > 0
    for (p0, _, _), (p1, _, _) in zip(out, out[1:]):
        assert np.linalg.norm(p1 - p0) <= SafetyConfig().max_lin_step + 1e-9


def test_gripper_passthrough_and_binary():
    assert make().gripper(0.3) == pytest.approx(0.3)
    assert make().gripper(1.7) == pytest.approx(1.0)
    b = make(gripper_binary=True, gripper_threshold=0.5)
    assert b.gripper(0.49) == 0.0
    assert b.gripper(0.5) == 1.0


def test_resync_recaptures_reference_while_engaged():
    cmd = make()
    src = MockPoseSource(default_script())
    out = run(cmd, src)
    # Pretend the arm was moved elsewhere by someone else and the operator hits A.
    cmd.resync()
    elsewhere = np.array([0.5, 0.2, 0.4])
    pose, _ = MockPoseSource(default_script()).read()
    p, r, engaged = cmd.update(pose, True, elsewhere, ROT0)
    assert engaged
    assert np.allclose(p, elsewhere), "first tick after resync must not jump away from the measured EE"
```

- [ ] **Step 2: Run to verify failure**

Run: `cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test/test_quest_command.py -q`
Expected: ImportError on `rammp_teleop.quest.command`.

- [ ] **Step 3: Implement**

`rammp_teleop/rammp_teleop/quest/command.py`:

```python
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
    def __init__(self, mapping: MappingConfig, safety: SafetyConfig,
                 gripper_binary: bool = False, gripper_threshold: float = 0.5):
        self._mapping_cfg = mapping
        self.mapper = ClutchedDeltaMapper(mapping)
        self.safety = SafetyFilter(safety)
        self.gripper_binary = gripper_binary
        self.gripper_threshold = gripper_threshold
        self._hold_pos: np.ndarray | None = None
        self._hold_rot: Rotation | None = None
        self.stats = {"rejected": 0, "vel_clamped": 0, "ws_clamped": 0}

    def resync(self) -> None:
        """Forget the hold target and the clutch references.

        If the grip is held on the next tick the mapper sees a rising edge and
        re-captures both references, so the first output equals the measured EE.
        """
        self._hold_pos = None
        self._hold_rot = None
        self.mapper = ClutchedDeltaMapper(self._mapping_cfg)

    def gripper(self, trigger: float) -> float:
        if self.gripper_binary:
            return 1.0 if trigger >= self.gripper_threshold else 0.0
        return float(np.clip(trigger, 0.0, 1.0))

    def update(self, ctrl_pose: np.ndarray, grip: bool, ee_pos, ee_rot: Rotation
               ) -> tuple[np.ndarray, Rotation, bool]:
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
```

- [ ] **Step 4: Run the tests**

Run: `cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test -q`
Expected: all pass. If `test_engaged_motion_follows_script_in_base_frame` fails on the index, print `len(out)` (expect 120 frames for a 2 s script at 60 Hz) and pick the last engaged index (89) accordingly.

- [ ] **Step 5: Commit**

```bash
git add rammp_teleop
git commit -m "feat: QuestCommand, the teleop tick without a driver"
```

---

### Task 4: `TeleopNodeBase` and the Xbox node

**Files:**
- Create: `rammp_teleop/rammp_teleop/session.py`, `rammp_teleop/rammp_teleop/xbox_node.py`, `rammp_teleop/config/xbox.yaml`, `rammp_teleop/launch/xbox.launch.py`

**Interfaces:**
- Consumes: Task 1 `logic`.
- Produces: `TeleopNodeBase(Node)` constructor `__init__(self, name: str, *, default_controller: str, default_rate_hz: float)`; subclass hooks `tick_input(dt) -> bool` (consume input, handle buttons, return engaged), `compute_target(dt, engaged) -> Pose | Sequence[float] | None`, `gripper_target(dt, engaged) -> float | None`, `seed_from_state() -> None`; services for subclasses `ee_pose() -> tuple[Vec3, Quat] | None`, `joint_positions() -> list | None`, `gripper_position() -> float | None`, `resync(why)`, `publish_estop(engaged, reason)`, `publish_gripper(position)`, attribute `uses_pose: bool`; module functions `pose_msg(pos, quat_xyzw) -> Pose` and `run(node_factory) -> int`.

- [ ] **Step 1: Write `session.py`**

```python
"""Shared arm-session plumbing for every rammp_teleop input node.

Lifecycle, following kinova_gen3_ros2's streaming contract (docs/interface.md):

  1. wait for /ee_state (pose controllers) or /joint_states (joint controllers)
  2. /acquire_control            -> token, stamped on every setpoint (acquire SEIZES)
  3. /list_controllers           -> the controller must be available; take its channel
  4. create the setpoint publisher and let DDS discovery settle
  5. /open_stream                -> session; setpoints only count while it is open
  6. timer at rate_hz: subclass turns its input into a target, this publishes it
     EVERY tick, idle or not, so the session stays alive and the arm holds
  7. SIGINT/SIGTERM: /close_stream, /release_control

Subclasses implement tick_input / compute_target / gripper_target / seed_from_state.
"""
from __future__ import annotations

import signal
import time
from typing import Callable, Optional, Sequence, Union

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rammp_arm_interfaces.msg import (
    EeState,
    GripperSetpoint,
    GripperState,
    JointSetpoint,
    PoseSetpoint,
    StreamStatus,
)
from rammp_arm_interfaces.srv import CloseStream, ListControllers, OpenStream
from rammp_common_interfaces.msg import ControlStatus, EStop
from rammp_common_interfaces.srv import AcquireControl, ReleaseControl
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

POSE_CONTROLLERS = ("ee_pose_position", "ee_pose_impedance")
JOINT_CONTROLLERS = ("joint_position", "joint_impedance")

SETPOINT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

Target = Union[Pose, Sequence[float]]


def pose_msg(pos, quat_xyzw) -> Pose:
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in pos)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (
        float(v) for v in quat_xyzw
    )
    return msg


class TeleopNodeBase(Node):
    def __init__(self, name: str, *, default_controller: str, default_rate_hz: float) -> None:
        super().__init__(name)
        dp = self.declare_parameter
        self.controller: str = dp("controller", default_controller).value
        self.owner_id: str = dp("owner_id", name).value
        self.rate_hz: float = dp("rate_hz", default_rate_hz).value
        self.stream_timeout_s: float = dp("stream_timeout_s", 0.2).value
        self.state_wait_timeout_s: float = dp("state_wait_timeout_s", 10.0).value
        self.reopen_interval_s: float = dp("reopen_interval_s", 1.0).value
        self.gripper_cmd_speed: float = dp("gripper_cmd_speed", 0.5).value
        self.gripper_force: float = dp("gripper_force", 0.3).value

        if self.controller not in POSE_CONTROLLERS + JOINT_CONTROLLERS:
            raise ValueError(
                f"controller must be one of {POSE_CONTROLLERS + JOINT_CONTROLLERS}, got {self.controller!r}"
            )
        if self.stream_timeout_s <= 1.0 / self.rate_hz:
            raise ValueError("stream_timeout_s must exceed the publish period or the session will expire")
        self.uses_pose = self.controller in POSE_CONTROLLERS

        # --- session state ---------------------------------------------------
        self._token: Optional[np.ndarray] = None
        self._token_valid = False
        self._generation: Optional[int] = None
        self._owned_by_other = False
        self._reacquire_inflight = False
        self._last_reacquire_attempt = 0.0
        self._stream_open = False
        self._opened_at_ns = 0
        self._channel: Optional[str] = None
        self._setpoint_pub = None
        self._last_reopen_attempt = 0.0
        self._reopen_inflight = False

        # --- arm state ---------------------------------------------------------
        self._ee: Optional[EeState] = None
        self._q: Optional[list] = None
        self._gripper_pos: Optional[float] = None
        self._last_tick = time.monotonic()

        # --- ROS I/O -----------------------------------------------------------
        self.create_subscription(EeState, "ee_state", self._on_ee_state, qos_profile_sensor_data)
        self.create_subscription(JointState, "joint_states", self._on_joint_states, qos_profile_sensor_data)
        self.create_subscription(GripperState, "gripper_state", self._on_gripper_state, qos_profile_sensor_data)
        self.create_subscription(StreamStatus, "stream_status", self._on_stream_status, LATCHED_QOS)
        self.create_subscription(ControlStatus, "control_status", self._on_control_status, LATCHED_QOS)

        self._gripper_pub = self.create_publisher(GripperSetpoint, "/setpoint/gripper", SETPOINT_QOS)
        self._estop_pub = self.create_publisher(EStop, "/estop", 10)

        self._acquire = self.create_client(AcquireControl, "acquire_control")
        self._release = self.create_client(ReleaseControl, "release_control")
        self._list = self.create_client(ListControllers, "list_controllers")
        self._open = self.create_client(OpenStream, "open_stream")
        self._close = self.create_client(CloseStream, "close_stream")

        self._tick_timer = None

    # ------------------------------------------------------------------ subclass API

    def tick_input(self, dt: float) -> bool:
        """Consume the latest device input and handle its buttons. Return True while
        the operator is actively commanding (deadman held / clutch engaged)."""
        raise NotImplementedError

    def compute_target(self, dt: float, engaged: bool) -> Optional[Target]:
        """Return the absolute target to stream this tick (a Pose for pose controllers,
        seven joint values for joint controllers), or None to skip publishing."""
        raise NotImplementedError

    def gripper_target(self, dt: float, engaged: bool) -> Optional[float]:
        """Return a gripper position 0..1 to send this tick, or None."""
        return None

    def seed_from_state(self) -> None:
        """Re-seed the input's target from the measured arm state."""
        raise NotImplementedError

    def engaged_hint(self) -> str:
        """Short operator instruction used in log lines, e.g. 'hold LB'."""
        return "engage the input"

    # ------------------------------------------------------------------ arm state

    def ee_pose(self):
        if self._ee is None:
            return None
        p, o = self._ee.pose.position, self._ee.pose.orientation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    def joint_positions(self) -> Optional[list]:
        return self._q

    def gripper_position(self) -> Optional[float]:
        return self._gripper_pos

    # ------------------------------------------------------------------ callbacks

    def _on_ee_state(self, msg: EeState) -> None:
        self._ee = msg

    def _on_joint_states(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        q = [by_name.get(f"joint_{i}") for i in range(1, 8)]
        if all(v is not None for v in q):
            self._q = q

    def _on_gripper_state(self, msg: GripperState) -> None:
        self._gripper_pos = float(msg.position)

    def _on_stream_status(self, msg: StreamStatus) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if not msg.open and stamp_ns and stamp_ns < self._opened_at_ns:
            return  # latched 'closed' from before our open_stream; not news
        was = self._stream_open
        self._stream_open = bool(msg.open) and msg.controller == self.controller
        if was and not self._stream_open:
            self.get_logger().warn(
                f"stream closed by driver (open={msg.open}, controller={msg.controller!r}, "
                f"rejected={msg.rejected_count}); {self.engaged_hint()} to reopen"
            )
        elif not was and self._stream_open:
            self.get_logger().info(f"stream open on {self.controller} -> {list(msg.channels)}")

    def _on_control_status(self, msg: ControlStatus) -> None:
        if self._token is None:
            return
        if msg.estopped:
            self.get_logger().warn("arm is E-STOPPED", throttle_duration_sec=5.0)
        if not msg.arbitration_enabled or self._generation is None:
            return
        self._owned_by_other = bool(msg.owned) and msg.owner_id != self.owner_id
        if msg.generation < self._generation:
            return  # stale latched sample from before our acquire
        lost = msg.generation > self._generation or (msg.generation == self._generation and not msg.owned)
        if lost and self._token_valid:
            self._token_valid = False
            if self._owned_by_other:
                self.get_logger().error(
                    f"control seized by {msg.owner_id!r} (generation {msg.generation}); our token is dead. "
                    f"It will be re-acquired when you {self.engaged_hint()} once they release."
                )
            else:
                self.get_logger().warn(
                    f"ownership cleared (generation {msg.generation}, e-stop or revoke); "
                    f"{self.engaged_hint()} to re-acquire control and reopen the stream"
                )

    # ------------------------------------------------------------------ helpers

    def _call(self, client, request, what: str, timeout_s: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().error(f"{what}: service {client.srv_name} not available")
            return None
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        if not fut.done():
            self.get_logger().error(f"{what}: timed out after {timeout_s}s")
            return None
        return fut.result()

    def _have_state(self) -> bool:
        return self._ee is not None if self.uses_pose else self._q is not None

    def resync(self, why: str) -> None:
        self.seed_from_state()
        self.get_logger().info(f"target re-seeded from measured state ({why})")

    def publish_estop(self, engaged: bool, reason: str) -> None:
        msg = EStop()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.engaged = engaged
        msg.source = self.get_name()
        msg.reason = reason
        self._estop_pub.publish(msg)
        self.get_logger().warn(f"/estop {'ENGAGED' if engaged else 'cleared'}: {reason}")

    def publish_gripper(self, position: float) -> None:
        g = GripperSetpoint()
        g.position = float(position)
        g.speed = float(self.gripper_cmd_speed)
        g.force = float(self.gripper_force)
        g.token = self._token
        self._gripper_pub.publish(g)

    # ------------------------------------------------------------------ session setup

    def setup(self) -> bool:
        """Acquire, discover, open. Blocking (spins this node). False on failure."""
        log = self.get_logger()

        deadline = time.monotonic() + self.state_wait_timeout_s
        while not self._have_state():
            if time.monotonic() > deadline:
                log.error("no /ee_state or /joint_states received; is kinova_gen3_node running?")
                return False
            rclpy.spin_once(self, timeout_sec=0.2)
        log.info("arm state received")

        resp = self._call(self._acquire, AcquireControl.Request(owner_id=self.owner_id), "acquire_control")
        if resp is None or not resp.accepted:
            log.error(f"acquire_control refused: {getattr(resp, 'message', '')}")
            return False
        self._token = np.frombuffer(bytes(resp.token), dtype=np.uint8).copy()
        self._token_valid = True
        self._generation = int(resp.generation)
        log.info(f"control acquired as {self.owner_id!r} (generation {resp.generation})")

        resp = self._call(self._list, ListControllers.Request(), "list_controllers")
        if resp is None:
            return False
        caps = {c.name: c for c in resp.controllers}
        cap = caps.get(self.controller)
        if cap is None or not cap.available:
            log.error(f"controller {self.controller!r} not available; driver offers "
                      f"{[c.name for c in resp.controllers if c.available]}")
            return False
        self._channel = cap.channels[0]

        msg_type = PoseSetpoint if self.uses_pose else JointSetpoint
        self._setpoint_pub = self.create_publisher(msg_type, self._channel, SETPOINT_QOS)
        settle_deadline = time.monotonic() + 3.0
        while self._setpoint_pub.get_subscription_count() == 0 and time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._setpoint_pub.get_subscription_count() == 0:
            log.warn(f"no subscriber discovered on {self._channel} yet; opening anyway")

        self.resync("startup")
        if not self._open_stream_blocking():
            return False

        self._last_tick = time.monotonic()
        self._tick_timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        log.info(f"teleop ready on {self.controller} -> {self._channel} at {self.rate_hz:.0f} Hz")
        return True

    def _open_request(self) -> OpenStream.Request:
        req = OpenStream.Request()
        req.controller = self.controller
        req.timeout_s = float(self.stream_timeout_s)
        req.token = self._token
        return req

    def _open_stream_blocking(self) -> bool:
        resp = self._call(self._open, self._open_request(), "open_stream")
        if resp is None or not resp.accepted:
            self.get_logger().error(f"open_stream refused: {getattr(resp, 'message', '')}")
            return False
        self._stream_open = True
        self._opened_at_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f"stream opened on {self.controller}, channels {list(resp.channels)}")
        return True

    def _try_reopen_async(self) -> None:
        now = time.monotonic()
        if self._reopen_inflight or now - self._last_reopen_attempt < self.reopen_interval_s:
            return
        self._last_reopen_attempt = now
        if not self._open.service_is_ready():
            return
        self._reopen_inflight = True
        self.resync("reopen")
        fut = self._open.call_async(self._open_request())

        def done(f):
            self._reopen_inflight = False
            resp = f.result() if f.exception() is None else None
            if resp is not None and resp.accepted:
                self._stream_open = True
                self._opened_at_ns = self.get_clock().now().nanoseconds
                self.get_logger().info("stream reopened")
            else:
                self.get_logger().warn(f"reopen refused: {getattr(resp, 'message', f.exception())}",
                                       throttle_duration_sec=5.0)

        fut.add_done_callback(done)

    def _try_reacquire_async(self) -> None:
        """Re-acquire after an e-stop/revoke cleared ownership. Never seizes from another owner."""
        now = time.monotonic()
        if self._reacquire_inflight or now - self._last_reacquire_attempt < self.reopen_interval_s:
            return
        self._last_reacquire_attempt = now
        if self._owned_by_other:
            self.get_logger().warn("arm is owned by someone else; not seizing", throttle_duration_sec=5.0)
            return
        if not self._acquire.service_is_ready():
            return
        self._reacquire_inflight = True
        fut = self._acquire.call_async(AcquireControl.Request(owner_id=self.owner_id))

        def done(f):
            self._reacquire_inflight = False
            resp = f.result() if f.exception() is None else None
            if resp is not None and resp.accepted:
                self._token = np.frombuffer(bytes(resp.token), dtype=np.uint8).copy()
                self._generation = int(resp.generation)
                self._token_valid = True
                self._stream_open = False  # a new generation never carries an open session
                self.get_logger().info(f"control re-acquired (generation {resp.generation})")
            else:
                self.get_logger().warn(f"re-acquire refused: {getattr(resp, 'message', f.exception())}",
                                       throttle_duration_sec=5.0)

        fut.add_done_callback(done)

    def teardown(self) -> None:
        if self._token is None:
            return
        if self._tick_timer is not None:
            self._tick_timer.cancel()
        if self._stream_open:
            self._call(self._close, CloseStream.Request(token=self._token), "close_stream", timeout_s=2.0)
        if self._token_valid:
            self._call(self._release, ReleaseControl.Request(token=self._token), "release_control", timeout_s=2.0)
        self.get_logger().info("stream closed, control released")

    # ------------------------------------------------------------------ main loop

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self._last_tick, 0.0), 0.1)
        self._last_tick = now

        engaged = self.tick_input(dt)

        if not self._have_state():
            return
        if not self._token_valid:
            if engaged:
                self._try_reacquire_async()
            return
        if not self._stream_open:
            if engaged:
                self._try_reopen_async()
            return

        target = self.compute_target(dt, engaged)
        if target is not None:
            self._publish_target(target)

        g = self.gripper_target(dt, engaged)
        if g is not None:
            self.publish_gripper(g)

    def _publish_target(self, target: Target) -> None:
        if self.uses_pose:
            msg = PoseSetpoint()
            msg.pose = target
        else:
            msg = JointSetpoint()
            msg.values = [float(v) for v in target]
        msg.token = self._token
        self._setpoint_pub.publish(msg)


def run(node_factory: Callable[[], TeleopNodeBase], argv=None) -> int:
    """Entry point shared by every teleop executable.

    rclpy's default SIGINT handler shuts the context down before we get a chance to
    close the stream and release control, so handle the signals ourselves.
    """
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    stop = {"flag": False}

    def on_signal(signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    node = node_factory()
    code = 0
    try:
        if not node.setup():
            code = 1
        else:
            while rclpy.ok() and not stop["flag"]:
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            node.teardown()
        except Exception as exc:  # noqa: BLE001 - never mask the original exit path
            node.get_logger().error(f"teardown failed: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
```

- [ ] **Step 2: Write `xbox_node.py`**

```python
"""Xbox-controller teleop through the streaming tier.

Motion happens only while the deadman (LB) is held. On every deadman press AND
release the target is re-seeded from the measured state, so letting go stops the
arm where it is rather than where the integrator got to.
"""
from __future__ import annotations

import sys
import time
from typing import Optional

from sensor_msgs.msg import Joy

from rammp_teleop.logic import CartesianIntegrator, GripperIntegrator, JointIntegrator, XboxMap
from rammp_teleop.session import TeleopNodeBase, pose_msg, run


class XboxTeleopNode(TeleopNodeBase):
    def __init__(self) -> None:
        super().__init__("xbox_teleop", default_controller="ee_pose_position", default_rate_hz=50.0)
        dp = self.declare_parameter
        self.joy_topic: str = dp("joy_topic", "/joy").value
        self.deadzone: float = dp("deadzone", 0.15).value
        self.joy_timeout_s: float = dp("joy_timeout_s", 0.5).value

        max_linear = dp("max_linear_speed", 0.05).value        # m/s at full stick
        max_angular = dp("max_angular_speed", 0.3).value       # rad/s at full stick
        max_joint = dp("max_joint_speed", 0.2).value           # rad/s at full stick
        lead_m = dp("target_lead_m", 0.05).value               # leash; 0 disables
        lead_rad = dp("target_lead_rad", 0.2).value
        joint_lead = dp("joint_target_lead_rad", 0.1).value
        gripper_speed = dp("gripper_speed", 1.0).value         # travel fraction per second

        self.map = XboxMap(
            axis_left_x=dp("axis_left_x", 0).value,
            axis_left_y=dp("axis_left_y", 1).value,
            axis_lt=dp("axis_lt", 2).value,
            axis_right_x=dp("axis_right_x", 3).value,
            axis_right_y=dp("axis_right_y", 4).value,
            axis_rt=dp("axis_rt", 5).value,
            axis_dpad_x=dp("axis_dpad_x", 6).value,
            axis_dpad_y=dp("axis_dpad_y", 7).value,
            button_deadman=dp("button_deadman", 4).value,
            button_estop=dp("button_estop", 1).value,
            button_estop_clear=dp("button_estop_clear", 7).value,
            button_resync=dp("button_resync", 3).value,
        )

        self.cart = CartesianIntegrator(max_linear, max_angular, lead_m, lead_rad)
        self.joints = JointIntegrator(max_joint, joint_lead)
        self.gripper = GripperIntegrator(gripper_speed)

        self._joy: Optional[Joy] = None
        self._joy_rx_time = 0.0
        self._axes: list = []
        self._prev_buttons: list = []
        self._prev_select_step = 0
        self._deadman_prev = False

        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.get_logger().info("Hold LB to move. B = e-stop, Start = clear, Y = resync.")

    def engaged_hint(self) -> str:
        return "hold LB"

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg
        self._joy_rx_time = time.monotonic()

    # ------------------------------------------------------------------ hooks

    def tick_input(self, dt: float) -> bool:
        now = time.monotonic()
        joy_fresh = self._joy is not None and (now - self._joy_rx_time) <= self.joy_timeout_s
        axes = list(self._joy.axes) if joy_fresh else []
        buttons = list(self._joy.buttons) if joy_fresh else []
        prev = self._prev_buttons
        self._prev_buttons = buttons
        self._axes = axes

        def rising(idx: int) -> bool:
            return self.map.pressed(buttons, idx) and not self.map.pressed(prev, idx)

        if rising(self.map.button_estop):
            self.publish_estop(True, "xbox B button")
        if rising(self.map.button_estop_clear):
            self.publish_estop(False, "xbox Start button")
        if rising(self.map.button_resync):
            self.resync("Y button")

        deadman = joy_fresh and self.map.deadman(buttons)
        if deadman != self._deadman_prev:
            self.resync("deadman pressed" if deadman else "deadman released")
            self._deadman_prev = deadman
        return deadman

    def compute_target(self, dt: float, engaged: bool):
        if self.uses_pose:
            ee = self.ee_pose()
            if ee is None:
                return None
            ap, aq = ee
            if engaged:
                lin, ang = self.map.cartesian_command(self._axes, self.deadzone)
                self.cart.step(lin, ang, dt, ap, aq)
            return pose_msg(self.cart.position, self.cart.orientation)

        q = self.joint_positions()
        if q is None:
            return None
        if engaged:
            step = self.map.joint_select_step(self._axes)
            if step != 0 and self._prev_select_step == 0:
                self.joints.select_next(step)
                self.get_logger().info(f"jogging joint_{self.joints.selected + 1}")
            self._prev_select_step = step
            self.joints.step(self.map.joint_jog_command(self._axes, self.deadzone), dt, q)
        return list(self.joints.positions)

    def gripper_target(self, dt: float, engaged: bool):
        if not engaged:
            return None
        close, open_ = self.map.gripper_command(self._axes)
        return self.gripper.position if self.gripper.step(close, open_, dt) else None

    def seed_from_state(self) -> None:
        ee = self.ee_pose()
        q = self.joint_positions()
        if self.uses_pose and ee is not None:
            self.cart.reset(*ee)
        elif q is not None:
            self.joints.reset(q)
        g = self.gripper_position()
        if g is not None:
            self.gripper.reset(g)


def main(argv=None) -> int:
    return run(XboxTeleopNode, argv)


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: Write `config/xbox.yaml` and `launch/xbox.launch.py`**

`rammp_teleop/config/xbox.yaml`:

```yaml
# Parameters for xbox_teleop. Speeds are what FULL stick deflection commands.
# The driver also rate-limits the reference (--max-ref-speed); if that is lower
# than what these speeds imply, the target leads the arm and the leash
# (target_lead_*) is what keeps that lead bounded.
xbox_teleop:
  ros__parameters:
    controller: ee_pose_position      # ee_pose_position | ee_pose_impedance | joint_position | joint_impedance
    owner_id: xbox_teleop
    rate_hz: 50.0
    stream_timeout_s: 0.2             # must exceed 1/rate_hz
    joy_topic: /joy
    deadzone: 0.15
    joy_timeout_s: 0.5                # no /joy for this long => sticks read as neutral

    # --- max speeds ---
    max_linear_speed: 0.05            # m/s      (pose controllers)
    max_angular_speed: 0.3            # rad/s    (pose controllers)
    max_joint_speed: 0.2              # rad/s    (joint controllers)

    # --- leash: max distance the target may lead the measured state; 0 disables ---
    target_lead_m: 0.05
    target_lead_rad: 0.2
    joint_target_lead_rad: 0.1

    # --- gripper (RT closes, LT opens; only while LB held) ---
    gripper_speed: 1.0                # full travel per second at full trigger
    gripper_cmd_speed: 0.5            # GripperSetpoint.speed
    gripper_force: 0.3                # GripperSetpoint.force (current ceiling)

    # --- Xbox Series X layout under ros-humble-joy (verify: ros2 topic echo /joy) ---
    axis_left_x: 0
    axis_left_y: 1
    axis_lt: 2
    axis_right_x: 3
    axis_right_y: 4
    axis_rt: 5
    axis_dpad_x: 6
    axis_dpad_y: 7
    button_deadman: 4                 # LB
    button_estop: 1                   # B
    button_estop_clear: 7             # Start
    button_resync: 3                  # Y

joy_node:
  ros__parameters:
    device_id: 0
    deadzone: 0.05                    # teleop applies its own; keep joy's small
    autorepeat_rate: 50.0             # steady /joy stream so joy_timeout_s detects a lost pad
    coalesce_interval_ms: 1
```

`rammp_teleop/launch/xbox.launch.py`:

```python
"""joy_node + xbox_teleop. Run against an already-running kinova_gen3_node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("rammp_teleop"), "config", "xbox.yaml")
    params_file = LaunchConfiguration("params_file")
    controller = LaunchConfiguration("controller")
    device_id = LaunchConfiguration("device_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("controller", default_value="ee_pose_position",
                                  description="ee_pose_position | ee_pose_impedance | joint_position | joint_impedance"),
            DeclareLaunchArgument("device_id", default_value="0", description="/dev/input/js<N>"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[params_file, {"device_id": device_id}],
                output="screen",
            ),
            Node(
                package="rammp_teleop",
                executable="xbox_teleop",
                name="xbox_teleop",
                parameters=[params_file, {"controller": controller}],
                output="screen",
            ),
        ]
    )
```

- [ ] **Step 4: Syntax-check locally (rclpy is not installed here, so only compile)**

Run: `uv run python -m py_compile rammp_teleop/rammp_teleop/session.py rammp_teleop/rammp_teleop/xbox_node.py rammp_teleop/launch/xbox.launch.py && echo compiled`
Expected: `compiled`. The pure tests still pass: `cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test -q`.

- [ ] **Step 5: Commit**

```bash
git add rammp_teleop
git commit -m "feat: TeleopNodeBase session plumbing and the Xbox node"
```

---

### Task 5: Quest reader and Quest teleop nodes

**Files:**
- Create: `rammp_teleop/rammp_teleop/quest_reader.py`, `rammp_teleop/rammp_teleop/quest_node.py`, `rammp_teleop/config/quest.yaml`, `rammp_teleop/launch/quest.launch.py`
- Test: `rammp_teleop/test/test_quest_reader_encode.py`

**Interfaces:**
- Consumes: Task 3 `QuestCommand`, Task 2 `MockPoseSource`, `OculusPoseSource`, `default_script`, `make_pose`; Task 4 `TeleopNodeBase`, `pose_msg`, `run`.
- Produces: `quest_reader.encode_sample(pose4x4, buttons: Buttons, hand: str) -> tuple[tuple[Vec3, Quat], list[float], list[int]]` (position+xyzw quaternion, Joy axes `[trigger]`, Joy buttons `[grip, A, B]`), pure and tested; topics `/quest/<hand>/pose` (`geometry_msgs/PoseStamped`) and `/quest/joy` (`sensor_msgs/Joy`).

- [ ] **Step 1: Write the failing encode test**

`rammp_teleop/test/test_quest_reader_encode.py`:

```python
"""encode_sample is the only logic in quest_reader; it must not need ROS to test."""
import numpy as np
from scipy.spatial.transform import Rotation

from rammp_teleop.quest.pose_source import Buttons
from rammp_teleop.quest.transforms import make_pose
from rammp_teleop.quest_reader import encode_sample


def test_encode_pose_is_xyzw_and_position():
    rot = Rotation.from_euler("z", 90, degrees=True)
    pose = make_pose([0.1, 0.2, 0.3], rot)
    (pos, quat), axes, buttons = encode_sample(pose, Buttons(grip=True, trigger=0.4), "right")
    assert np.allclose(pos, [0.1, 0.2, 0.3])
    assert np.allclose(quat, rot.as_quat())  # scipy order is x, y, z, w
    assert axes == [0.4]
    assert buttons == [1, 0, 0]


def test_encode_right_hand_face_buttons():
    pose = np.eye(4)
    _, _, buttons = encode_sample(pose, Buttons(grip=False, trigger=0.0, extra={"A": True, "B": False}), "right")
    assert buttons == [0, 1, 0]


def test_encode_left_hand_uses_x_and_y():
    pose = np.eye(4)
    _, _, buttons = encode_sample(pose, Buttons(grip=False, trigger=0.0, extra={"X": True, "Y": True}), "left")
    assert buttons == [0, 1, 1]
```

- [ ] **Step 2: Run to verify failure**

Run: `cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test/test_quest_reader_encode.py -q`
Expected: ImportError. Note `quest_reader.py` imports rclpy at module top, which is not installed locally; to keep `encode_sample` testable put it above the ROS imports guarded as shown below.

- [ ] **Step 3: Write `quest_reader.py`**

```python
"""Republish a Meta Quest controller as ROS topics.

  /quest/<hand>/pose   geometry_msgs/PoseStamped   raw controller pose, headset tracking frame
  /quest/joy           sensor_msgs/Joy             axes=[index trigger 0..1], buttons=[grip, A, B]

Nothing is remapped here: the teleop node's R_align is the one calibration knob.
With ``mock:=true`` a scripted controller (engage, move, grip, release) loops
forever, so the whole pipeline runs with no headset and no adb.
"""
from __future__ import annotations

import sys

import numpy as np
from scipy.spatial.transform import Rotation

from rammp_teleop.quest.pose_source import Buttons, MockPoseSource, default_script


def encode_sample(pose: np.ndarray, btn: Buttons, hand: str):
    """(position, quat_xyzw), Joy axes, Joy buttons for one controller sample."""
    pos = tuple(float(v) for v in pose[:3, 3])
    quat = tuple(float(v) for v in Rotation.from_matrix(pose[:3, :3]).as_quat())
    a_key, b_key = ("A", "B") if hand == "right" else ("X", "Y")
    buttons = [int(bool(btn.grip)), int(bool(btn.extra.get(a_key, False))), int(bool(btn.extra.get(b_key, False)))]
    return (pos, quat), [float(btn.trigger)], buttons


def _build_source(hand: str, quest_ip: str, mock: bool):
    if mock:
        return MockPoseSource(default_script())
    from rammp_teleop.quest.pose_source import OculusPoseSource  # imports oculus_reader (needs adb)

    return OculusPoseSource(hand="r" if hand == "right" else "l", ip_address=quest_ip or None)


def main(argv=None) -> int:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Joy

    class QuestReaderNode(Node):
        def __init__(self) -> None:
            super().__init__("quest_reader")
            dp = self.declare_parameter
            self.hand: str = dp("hand", "right").value
            self.quest_ip: str = dp("quest_ip", "").value
            self.rate_hz: float = dp("rate_hz", 60.0).value
            self.mock: bool = dp("mock", False).value
            self.frame_id: str = dp("frame_id", "quest").value
            if self.hand not in ("right", "left"):
                raise ValueError(f"hand must be 'right' or 'left', got {self.hand!r}")
            self.source = _build_source(self.hand, self.quest_ip, self.mock)
            self._pose_pub = self.create_publisher(PoseStamped, f"/quest/{self.hand}/pose", 10)
            self._joy_pub = self.create_publisher(Joy, "/quest/joy", 10)
            self.create_timer(1.0 / self.rate_hz, self._tick)
            self.get_logger().info(
                f"quest_reader: {'MOCK scripted controller' if self.mock else 'Quest over adb'}, hand={self.hand}, "
                f"{self.rate_hz:.0f} Hz"
            )

        def _tick(self) -> None:
            if self.mock and self.source.done:
                self.source.reset()
            pose, btn = self.source.read()
            (pos, quat), axes, buttons = encode_sample(pose, btn, self.hand)
            stamp = self.get_clock().now().to_msg()

            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.frame_id
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = quat
            self._pose_pub.publish(ps)

            joy = Joy()
            joy.header.stamp = stamp
            joy.header.frame_id = self.frame_id
            joy.axes = axes
            joy.buttons = buttons
            self._joy_pub.publish(joy)

        def close(self) -> None:
            self.source.close()

    rclpy.init(args=argv)
    node = QuestReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run the encode test**

Run: `cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test/test_quest_reader_encode.py -q`
Expected: 3 passed.

- [ ] **Step 5: Write `quest_node.py`**

```python
"""Quest 3 controller teleop through the streaming tier.

Squeeze the grip to clutch in: the controller's motion from that moment is
applied to the EE pose from that moment (no absolute calibration; R_align is the
one rotation knob). Release to freeze. Index trigger drives the gripper. B
engages /estop; A re-captures the references.

Default controller is ee_pose_impedance, the same joint-impedance-with-IK law the
original UDP setup ran with --joint-impedance.
"""
from __future__ import annotations

import sys
import time
from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy

from rammp_teleop.logic import XboxMap
from rammp_teleop.quest.command import QuestCommand
from rammp_teleop.quest.mapping import MappingConfig
from rammp_teleop.quest.safety import SafetyConfig
from rammp_teleop.quest.transforms import make_pose
from rammp_teleop.session import TeleopNodeBase, pose_msg, run


def _euler_zyx(deg) -> Rotation:
    return Rotation.from_euler("ZYX", [float(v) for v in deg], degrees=True)


class QuestTeleopNode(TeleopNodeBase):
    def __init__(self) -> None:
        super().__init__("quest_teleop", default_controller="ee_pose_impedance", default_rate_hz=60.0)
        if not self.uses_pose:
            raise ValueError("quest_teleop streams EE poses; controller must be ee_pose_position or ee_pose_impedance")
        dp = self.declare_parameter
        self.hand: str = dp("hand", "right").value
        self.input_timeout_s: float = dp("input_timeout_s", 0.5).value
        self.button_grip: int = dp("button_grip", 0).value
        self.button_resync: int = dp("button_resync", 1).value      # A
        self.button_estop: int = dp("button_estop", 2).value        # B
        self.axis_trigger: int = dp("axis_trigger", 0).value

        mapping = MappingConfig(
            R_align=_euler_zyx(dp("r_align_euler_zyx_deg", [0.0, 0.0, 0.0]).value),
            trans_smooth=dp("trans_smooth", 0.8).value,
            rot_smooth=dp("rot_smooth", 0.8).value,
            jump_pos_tol=dp("jump_pos_tol", 0.05).value,
            jump_rot_tol=dp("jump_rot_tol", 0.5).value,
            pos_scale=dp("pos_scale", 1.0).value,
            rot_frame=dp("rot_frame", "base").value,
            R_tool=_euler_zyx(dp("r_tool_euler_zyx_deg", [0.0, 0.0, 0.0]).value),
        )
        safety = SafetyConfig(
            ws_min=np.array(dp("ws_min", [0.2, -0.4, 0.05]).value, float),
            ws_max=np.array(dp("ws_max", [0.75, 0.4, 0.7]).value, float),
            max_lin_step=dp("max_lin_step", 0.01).value,
            max_ang_step=dp("max_ang_step", 0.05).value,
        )
        self.cmd = QuestCommand(
            mapping, safety,
            gripper_binary=dp("gripper_binary", False).value,
            gripper_threshold=dp("gripper_threshold", 0.5).value,
        )

        self._ctrl_pose: Optional[np.ndarray] = None
        self._pose_rx_time = 0.0
        self._joy: Optional[Joy] = None
        self._joy_rx_time = 0.0
        self._prev_buttons: list = []
        self._trigger = 0.0
        self._last_gripper_sent: Optional[float] = None

        self.create_subscription(PoseStamped, f"/quest/{self.hand}/pose", self._on_pose, 10)
        self.create_subscription(Joy, "/quest/joy", self._on_joy, 10)
        self.get_logger().info("Squeeze GRIP to move. Trigger = gripper, B = e-stop, A = resync.")

    def engaged_hint(self) -> str:
        return "squeeze the grip"

    def _on_pose(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        self._ctrl_pose = make_pose([p.x, p.y, p.z], Rotation.from_quat([o.x, o.y, o.z, o.w]))
        self._pose_rx_time = time.monotonic()

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg
        self._joy_rx_time = time.monotonic()

    # ------------------------------------------------------------------ hooks

    def tick_input(self, dt: float) -> bool:
        now = time.monotonic()
        fresh = (
            self._ctrl_pose is not None and self._joy is not None
            and now - self._pose_rx_time <= self.input_timeout_s
            and now - self._joy_rx_time <= self.input_timeout_s
        )
        buttons = list(self._joy.buttons) if fresh else []
        axes = list(self._joy.axes) if fresh else []
        prev = self._prev_buttons
        self._prev_buttons = buttons

        def rising(idx: int) -> bool:
            return XboxMap.pressed(buttons, idx) and not XboxMap.pressed(prev, idx)

        if rising(self.button_estop):
            self.publish_estop(True, "quest B button")
        if rising(self.button_resync):
            self.resync("quest A button")

        self._trigger = float(axes[self.axis_trigger]) if 0 <= self.axis_trigger < len(axes) else 0.0
        return fresh and XboxMap.pressed(buttons, self.button_grip)

    def compute_target(self, dt: float, engaged: bool):
        ee = self.ee_pose()
        if ee is None or self._ctrl_pose is None:
            return None
        ee_pos = np.asarray(ee[0], float)
        ee_rot = Rotation.from_quat(ee[1])
        pos, rot, _ = self.cmd.update(self._ctrl_pose, engaged, ee_pos, ee_rot)
        return pose_msg(pos, rot.as_quat())

    def gripper_target(self, dt: float, engaged: bool):
        if not engaged:
            return None
        g = self.cmd.gripper(self._trigger)
        if self._last_gripper_sent is not None and abs(g - self._last_gripper_sent) < 1e-3:
            return None
        self._last_gripper_sent = g
        return g

    def seed_from_state(self) -> None:
        # The mapper captures the EE reference itself on the next grip rising
        # edge; all we do is forget the freeze target and the old references.
        self.cmd.resync()
        self._last_gripper_sent = None


def main(argv=None) -> int:
    return run(QuestTeleopNode, argv)


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 6: Write `config/quest.yaml` and `launch/quest.launch.py`**

`rammp_teleop/config/quest.yaml`:

```yaml
# Parameters for quest_teleop and quest_reader.
#
# r_align_euler_zyx_deg is a PER-SESSION calibration: the Quest tracking frame's
# yaw depends on how the headset was oriented at boot. Run
#   ros2 run rammp_teleop quest_calibrate
# and paste the printed triple here before flying. The value below is a stale
# example from a previous session, not a default that works.
quest_teleop:
  ros__parameters:
    controller: ee_pose_impedance     # compliant (joint impedance + in-loop IK); ee_pose_position is stiff
    owner_id: quest_teleop
    rate_hz: 60.0
    stream_timeout_s: 0.2             # must exceed 1/rate_hz
    hand: right
    input_timeout_s: 0.5              # no /quest/* for this long => grip reads as released (freeze)

    # --- controller -> base mapping ---
    r_align_euler_zyx_deg: [128.7663, -8.3652, 92.4841]
    trans_smooth: 0.8                 # first-order lag on the command, (0, 1]; 1.0 = raw
    rot_smooth: 0.8
    jump_pos_tol: 0.05                # m per tick; larger controller jumps are rejected
    jump_rot_tol: 0.5                 # rad per tick
    pos_scale: 1.0
    rot_frame: base                   # base | tool
    r_tool_euler_zyx_deg: [0.0, 0.0, 0.0]

    # --- safety: workspace box (base frame, m) and per-tick step caps ---
    ws_min: [0.0, -0.70, 0.05]
    ws_max: [1.05, 0.70, 1.00]
    max_lin_step: 0.01                # m per tick  (0.6 m/s at 60 Hz)
    max_ang_step: 0.05                # rad per tick (3 rad/s at 60 Hz)

    # --- gripper (index trigger, only while grip held) ---
    gripper_binary: false
    gripper_threshold: 0.5
    gripper_cmd_speed: 0.5            # GripperSetpoint.speed
    gripper_force: 0.3                # GripperSetpoint.force (current ceiling)

    # --- /quest/joy layout from quest_reader: buttons=[grip, A, B], axes=[trigger] ---
    button_grip: 0
    button_resync: 1                  # A
    button_estop: 2                   # B
    axis_trigger: 0

quest_reader:
  ros__parameters:
    hand: right
    quest_ip: ""                      # empty = USB; set for adb over Wi-Fi
    rate_hz: 60.0
    mock: false                       # true = scripted controller, no headset
```

`rammp_teleop/launch/quest.launch.py`:

```python
"""quest_reader + quest_teleop. Run against an already-running kinova_gen3_node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("rammp_teleop"), "config", "quest.yaml")
    params_file = LaunchConfiguration("params_file")
    controller = LaunchConfiguration("controller")
    hand = LaunchConfiguration("hand")
    quest_ip = LaunchConfiguration("quest_ip")
    mock = LaunchConfiguration("mock")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("controller", default_value="ee_pose_impedance",
                                  description="ee_pose_impedance | ee_pose_position"),
            DeclareLaunchArgument("hand", default_value="right", description="right | left"),
            DeclareLaunchArgument("quest_ip", default_value="", description="adb over Wi-Fi; empty = USB"),
            DeclareLaunchArgument("mock", default_value="false", description="scripted controller, no headset"),
            Node(
                package="rammp_teleop",
                executable="quest_reader",
                name="quest_reader",
                parameters=[params_file, {"hand": hand, "quest_ip": quest_ip, "mock": mock}],
                output="screen",
            ),
            Node(
                package="rammp_teleop",
                executable="quest_teleop",
                name="quest_teleop",
                parameters=[params_file, {"controller": controller, "hand": hand}],
                output="screen",
            ),
        ]
    )
```

- [ ] **Step 7: Compile-check and run all local tests**

Run: `uv run python -m py_compile rammp_teleop/rammp_teleop/quest_node.py rammp_teleop/rammp_teleop/quest_reader.py rammp_teleop/launch/quest.launch.py && echo compiled`
Run: `cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test -q`
Expected: `compiled`, all tests pass.

- [ ] **Step 8: Commit**

```bash
git add rammp_teleop
git commit -m "feat: Quest reader and Quest teleop nodes with launch and config"
```

---

### Task 6: Build, test and smoke on abra

**Files:**
- Create: `.hil.yml`

**Interfaces:**
- Consumes: everything above.
- Produces: a built `rammp_teleop` in `abra:~/ros2_ws/install`.

- [ ] **Step 1: Add `.hil.yml` and sync**

`.hil.yml`:

```yaml
default_target: dojo
sync:
  - {from: ., to: ros2_ws/src/rammp-teleop}
```

```bash
cd /home/swapnil/atdev/rammp-teleop
uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
```

- [ ] **Step 2: colcon build on abra**

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -c 'cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select rammp_teleop --symlink-install 2>&1 | tail -15'
```

Expected: `Summary: 1 package finished`. If `rammp_arm_interfaces` is not found, add `source ~/ros2_ws/install/setup.bash` before the build (the interfaces are already built in that workspace per abra's other packages).

- [ ] **Step 3: Run the pure tests on abra's Python (3.10, system numpy/scipy)**

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -c 'cd ~/ros2_ws/src/rammp-teleop/rammp_teleop && python3 -m pytest test -q 2>&1 | tail -5'
```

Expected: same pass count as locally.

- [ ] **Step 4: Import check of every executable inside the sourced workspace**

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -c 'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && python3 -c "import rammp_teleop.session, rammp_teleop.xbox_node, rammp_teleop.quest_node, rammp_teleop.quest_reader; print(\"imports ok\")" && ros2 pkg executables rammp_teleop'
```

Expected: `imports ok` and the four executables listed.

- [ ] **Step 5: Smoke the mock Quest reader and the no-arm exit paths (domain 42, no arm involved)**

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -c '
set -o pipefail
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
timeout 12 ros2 run rammp_teleop quest_reader --ros-args -p mock:=true > /tmp/qr.log 2>&1 &
sleep 3
timeout 6 ros2 topic hz /quest/joy --window 30 2>&1 | grep -m1 "average rate"
timeout 3 ros2 topic echo --once /quest/right/pose 2>&1 | head -12
wait
echo "--- reader log:"; tail -3 /tmp/qr.log
echo "--- quest_teleop with no arm (expect exit 1 after ~10 s):"
timeout 20 ros2 run rammp_teleop quest_teleop --ros-args -p state_wait_timeout_s:=3.0 2>&1 | tail -3; echo "exit=$?"
echo "--- xbox_teleop with no arm:"
timeout 20 ros2 run rammp_teleop xbox_teleop --ros-args -p state_wait_timeout_s:=3.0 2>&1 | tail -3; echo "exit=$?"
'
```

Expected: `average rate: ~60`, a PoseStamped with `frame_id: quest`, and both teleop nodes log `no /ee_state or /joint_states received` then exit 1 (the pipeline's `$?` is `tail`'s, so read the log line rather than the number; if you want the code, drop the `| tail -3`).

- [ ] **Step 6: Commit**

```bash
git add .hil.yml
git commit -m "chore: hil sync target for abra"
```

---

### Task 7: RAMMP module scaffolding (Dockerfile, fragments, Makefile, CI)

**Files:**
- Create: `Dockerfile`, `.dockerignore`, `rammp-alternative.xbox.yaml`, `rammp-alternative.quest.yaml`, `rammp-alternative.quest-mock.yaml`, `Makefile`, `scripts/validate_fragment.py`, `scripts/smoke.sh`, `tests/test_validate_fragment.py`, `.pre-commit-config.yaml`, `.github/workflows/build.yml`, `.github/workflows/lint.yml`, `CONTRIBUTING.md`

- [ ] **Step 1: Copy the template tooling verbatim**

```bash
TEMPLATE=/home/swapnil/atdev/rammp-module-template
mkdir -p scripts tests .github/workflows
cp "$TEMPLATE/scripts/validate_fragment.py" scripts/
cp "$TEMPLATE/scripts/smoke.sh" scripts/ && chmod +x scripts/smoke.sh
cp "$TEMPLATE/tests/test_validate_fragment.py" tests/
cp "$TEMPLATE/.pre-commit-config.yaml" .
cp "$TEMPLATE/.github/workflows/lint.yml" .github/workflows/
cp "$TEMPLATE/CONTRIBUTING.md" .
sed -i 's/rammp-module-template/rammp-teleop/g' CONTRIBUTING.md
```

Edit `scripts/smoke.sh`: replace the `PATTERN=` line with `PATTERN="${SMOKE_PATTERN:-quest_reader: MOCK scripted controller}"` and delete the two `EDIT-ME` comment lines above it. The smoke container must run the mock command, so also change the `docker run` line to append the mock launch command:

```bash
docker run -d --rm --name "$NAME" --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" "$IMAGE" \
  ros2 launch rammp_teleop quest.launch.py mock:=true >/dev/null
```

- [ ] **Step 2: Write the Dockerfile and .dockerignore**

`Dockerfile`:

```dockerfile
# syntax=docker/dockerfile:1
# rammp-teleop: operator-input nodes (Xbox pad, Meta Quest 3) on the fleet's
# Cyclone DDS graph. rammp-base owns ROS 2 Humble, Cyclone and the pinned
# rammp-interfaces-ros2 in /ros2_ws; this builds rammp_teleop into /module_ws.
# arm64/Jetson only, like every rammp-base image.
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

# joy: the Xbox pad. adb + git-lfs: the Quest (oculus_reader pushes an APK to
# the headset over adb; the APK ships through LFS, so lfs must be installed
# BEFORE the clone or you get a 132-byte pointer and a silent failure later).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-humble-joy android-tools-adb git git-lfs python3-pip python3-scipy \
 && rm -rf /var/lib/apt/lists/* \
 && git lfs install --system

RUN git clone --depth 1 https://github.com/rail-berkeley/oculus_reader.git /opt/oculus_reader \
 && pip3 install --no-cache-dir -e /opt/oculus_reader

WORKDIR /module_ws
COPY . /module_ws/src/rammp-teleop/

RUN . /opt/ros/humble/setup.sh \
 && . /ros2_ws/install/setup.sh \
 && apt-get update \
 && rosdep install --from-paths src --ignore-src -y \
 && rm -rf /var/lib/apt/lists/* \
 && colcon build \
 && rm -rf /module_ws/build /module_ws/log

# Default: the Xbox pad. Fragments override this per alternative. `ros2 launch`
# forwards SIGTERM to its children, which scripts/smoke.sh checks.
CMD ["ros2", "launch", "rammp_teleop", "xbox.launch.py"]
```

`.dockerignore`:

```
.git
Dockerfile
.dockerignore
*.md
**/__pycache__
**/*.pyc
build/
install/
log/
docs/
.hil.yml
```

- [ ] **Step 3: Write the three fragments**

`rammp-alternative.xbox.yaml`:

```yaml
# Xbox-pad teleop of the arm through kinova-gen3-ros2's streaming tier.
# privileged + /dev:/dev so joy_node can open /dev/input/js0. The arm must be
# up (this node acquires control and opens a stream; nothing moves without LB).
id: xbox_teleop
tier: experimental
kind: docker
container:
  image: ghcr.io/rammp-org/rammp-teleop:dev   # pin a release tag in a deployment
  network_mode: host
  ipc: host
  privileged: true
  volumes:
    - /dev:/dev
  environment:
    ROS_DOMAIN_ID: "0"
  command: ros2 launch rammp_teleop xbox.launch.py
publishes: [/setpoint/pose, /setpoint/gripper, /estop]
subscribes: [/ee_state, /joint_states, /gripper_state, /stream_status, /control_status]
```

`rammp-alternative.quest.yaml`:

```yaml
# Meta Quest 3 teleop: quest_reader (adb over USB, needs /dev/bus/usb) plus
# quest_teleop streaming poses on ee_pose_impedance. r_align in
# config/quest.yaml is per-session; run quest_calibrate first.
id: quest_teleop
tier: experimental
kind: docker
container:
  image: ghcr.io/rammp-org/rammp-teleop:dev
  network_mode: host
  ipc: host
  privileged: true
  volumes:
    - /dev:/dev
  environment:
    ROS_DOMAIN_ID: "0"
  command: ros2 launch rammp_teleop quest.launch.py
publishes: [/setpoint/pose, /setpoint/gripper, /estop, /quest/right/pose, /quest/joy]
subscribes: [/ee_state, /joint_states, /gripper_state, /stream_status, /control_status]
```

`rammp-alternative.quest-mock.yaml`:

```yaml
# Same interface, no headset: a scripted controller loops engage/move/grip/release.
id: quest_teleop_mock
tier: experimental
kind: docker
container:
  image: ghcr.io/rammp-org/rammp-teleop:dev
  network_mode: host
  ipc: host
  environment:
    ROS_DOMAIN_ID: "0"
  command: ros2 launch rammp_teleop quest.launch.py mock:=true
publishes: [/setpoint/pose, /setpoint/gripper, /estop, /quest/right/pose, /quest/joy]
subscribes: [/ee_state, /joint_states, /gripper_state, /stream_status, /control_status]
```

- [ ] **Step 4: Makefile and build workflow**

`Makefile`:

```make
IMAGE ?= ghcr.io/rammp-org/rammp-teleop
TAG ?= dev
ROS_DOMAIN_ID ?= 0

.PHONY: build run check smoke lint test

build:
	docker build -t $(IMAGE):$(TAG) .

run:
	docker run --rm --network host --ipc host --privileged -v /dev:/dev \
		-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) $(IMAGE):$(TAG)

check:
	uv run --with pyyaml python3 scripts/validate_fragment.py \
		rammp-alternative*.yaml

test:
	cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test -q
	uv run --with pyyaml --with pytest python -m pytest tests -q

smoke:
	./scripts/smoke.sh $(IMAGE):$(TAG)

lint:
	pre-commit run --all-files
```

`.github/workflows/build.yml`: copy `TEMPLATE/.github/workflows/build.yml`, then make these edits:
- in job `fragment`, replace `pip install pyyaml pytest` with `pip install pyyaml pytest numpy scipy` and `pytest tests -v` with `pytest tests -v && (cd rammp_teleop && python -m pytest test -v)`;
- job `smoke`: `runs-on: ubuntu-22.04-arm` (rammp-base is arm64 only);
- job `publish`: `runs-on: ubuntu-22.04-arm`, remove the `setup-qemu-action` step, and set `platforms: linux/arm64`.

- [ ] **Step 5: Validate**

Run: `make check`
Expected: three `ok` lines.
Run: `uv run --with pyyaml --with pytest python -m pytest tests -q`
Expected: the template's validator tests pass.
Run: `uv run python -c "import yaml; [yaml.safe_load(open(f)) for f in ['.github/workflows/build.yml', '.pre-commit-config.yaml']]; print('yaml ok')"`.

Docker build is not run here: the base image is arm64-only and abra is shared. Note that in the commit message.

- [ ] **Step 6: Commit**

```bash
git add Dockerfile .dockerignore rammp-alternative.*.yaml Makefile scripts tests .pre-commit-config.yaml .github CONTRIBUTING.md
git commit -m "chore: RAMMP module scaffolding (Dockerfile, fragments, Makefile, CI); image not yet built"
```

---

### Task 8: Docs and README

**Files:**
- Create: `README.md`, `docs/index.mdx`, `docs/_meta.js`, `docs/interface.md`

- [ ] **Step 1: Write the docs**

`docs/_meta.js`:

```js
// Section navigation for the RAMMP docs site (copied in by rammp-docs).
export default {
  index: 'Introduction',
  interface: 'Interface reference'
}
```

`docs/index.mdx`:

```mdx
---
title: Introduction
---

# Teleoperating the arm

Operator-input nodes for the Kinova Gen3: an Xbox pad and a Meta Quest 3
controller. Both drive the arm through
[`kinova-gen3-ros2`](https://github.com/rammp-org/kinova-gen3-ros2)'s streaming
tier: they acquire control, open a stream on a pose controller, and publish an
absolute target every tick. Nothing moves until the operator engages (LB on the
pad, grip squeeze on the Quest), and letting go holds the arm where it is.

| Input | Launch | Controller | How it makes a target |
|---|---|---|---|
| Xbox pad | `ros2 launch rammp_teleop xbox.launch.py` | `ee_pose_position` (stiff) | integrates stick velocities, leashed to the measured pose |
| Quest 3 | `ros2 launch rammp_teleop quest.launch.py` | `ee_pose_impedance` (compliant) | clutched delta of the controller pose, workspace box and step caps |

See the [interface reference](interface) for topics, parameters and the button maps.
```

`docs/interface.md`: write the following sections with the real values from the code: **Topics** (published: `/setpoint/pose` `rammp_arm_interfaces/PoseSetpoint`, `/setpoint/gripper` `GripperSetpoint`, `/estop` `rammp_common_interfaces/EStop`, Quest only: `/quest/<hand>/pose` `geometry_msgs/PoseStamped`, `/quest/joy` `sensor_msgs/Joy`; subscribed: `/ee_state`, `/joint_states`, `/gripper_state`, `/stream_status`, `/control_status`, `/joy` for Xbox); **Services used** (`/acquire_control`, `/list_controllers`, `/open_stream`, `/close_stream`, `/release_control`); **Session behaviour** (the seven numbered lifecycle steps from `session.py`'s docstring, plus recovery: stream closed or ownership lost is re-tried while engaged, never seizing from another owner); **Xbox controls** (the table from the original README: LB deadman, left stick x/y, right stick z/yaw, D-pad pitch/roll, RT/LT gripper, B e-stop, Start clear, Y resync, joint mode with D-pad joint select); **Quest controls** (grip clutch, trigger gripper, A resync, B e-stop, no clear on the headset); **Parameters** (one table per node, copied from `config/xbox.yaml` and `config/quest.yaml` with the comments as descriptions); **Calibration** (`ros2 run rammp_teleop quest_calibrate [--hand r|l] [--quest-ip IP]`, paste the triple into `r_align_euler_zyx_deg`); **Hardware-free** (`quest.launch.py mock:=true`).

`README.md`: title, one-paragraph purpose, the launch table from `docs/index.mdx`, a **Build on abra** block:

```bash
cd ~/ros2_ws
git clone git@github.com:rammp-org/rammp-teleop.git src/rammp-teleop
source /opt/ros/humble/setup.bash
colcon build --packages-select rammp_teleop --symlink-install
source install/setup.bash
```

a **Prerequisites** block (Xbox: `ros-humble-joy`; Quest: `sudo apt install android-tools-adb git-lfs`, `git lfs install`, clone `https://github.com/rail-berkeley/oculus_reader.git`, `pip install -e` it, USB-connect the headset and accept the adb prompt), a **Tests** block (`make test`), a **Container** block (`make build`, `make smoke`, fragments), and a pointer to `docs/interface.md`.

- [ ] **Step 2: Check the markdown renders sanely**

Run: `grep -c "" README.md docs/interface.md docs/index.mdx` (non-zero line counts) and `uv run python -c "print(open('docs/_meta.js').read().count('index'))"` (1).

- [ ] **Step 3: Commit**

```bash
git add README.md docs
git commit -m "docs: README and interface reference for the Xbox and Quest nodes"
```

---

### Task 9: Local sheppy manifest edit and profiles

**Files:**
- Modify: DEPLOY `december_2026/sheppy-manifest.yaml` (append a node; uncommitted)
- Create: DEPLOY `december_2026/profiles/teleop-xbox.yaml`, `december_2026/profiles/teleop-quest.yaml` (uncommitted)

- [ ] **Step 1: Append the teleop node to the manifest**

Append at the end of `DEPLOY/december_2026/sheppy-manifest.yaml` (same indentation as the other `nodes:` entries):

```yaml

  # LOCAL EDIT -- runs rammp-teleop from ~/ros2_ws/src on abra. Do not merge:
  # sheppy has no local-path mechanism, so these are `process` alternatives that
  # source the workspace. Replace with the docker fragments from rammp-teleop
  # (rammp-alternative.{xbox,quest,quest-mock}.yaml) once an image is published.
  - name: teleop
    description: Operator input -> /setpoint/pose and /setpoint/gripper through the arm's streaming tier
    select: single
    alternatives:
      - id: xbox_local
        kind: process
        command: "bash -lc 'source /opt/ros/humble/setup.bash && source $HOME/ros2_ws/install/setup.bash && exec ros2 launch rammp_teleop xbox.launch.py'"
        publishes:
          - /setpoint/pose
          - /setpoint/gripper
          - /estop
        subscribes:
          - /ee_state
          - /joint_states
          - /gripper_state
          - /stream_status
          - /control_status
      - id: quest3_local
        kind: process
        command: "bash -lc 'source /opt/ros/humble/setup.bash && source $HOME/ros2_ws/install/setup.bash && exec ros2 launch rammp_teleop quest.launch.py'"
        publishes:
          - /setpoint/pose
          - /setpoint/gripper
          - /estop
          - /quest/right/pose
          - /quest/joy
        subscribes:
          - /ee_state
          - /joint_states
          - /gripper_state
          - /stream_status
          - /control_status
      - id: quest3_mock_local
        kind: process
        command: "bash -lc 'source /opt/ros/humble/setup.bash && source $HOME/ros2_ws/install/setup.bash && exec ros2 launch rammp_teleop quest.launch.py mock:=true'"
        publishes:
          - /setpoint/pose
          - /setpoint/gripper
          - /estop
          - /quest/right/pose
          - /quest/joy
        subscribes:
          - /ee_state
      - id: teleop_off
        kind: process
        command: "bash -lc 'sleep infinity'"
        publishes: []
```

- [ ] **Step 2: Add two profiles**

`DEPLOY/december_2026/profiles/teleop-xbox.yaml`:

```yaml
description: Everything real, Xbox pad on the arm (LOCAL, not for merge)
selections:
  scene_camera: gemini336l_docker
  wrist_camera: oak_d_pro_docker
  foxglove: foxglove_bridge_docker
  arm: kinova_gen3_driver
  planner: curobo_planner
  teleop: xbox_local
```

`DEPLOY/december_2026/profiles/teleop-quest.yaml`: same with `teleop: quest3_local` and description `Everything real, Quest 3 on the arm (LOCAL, not for merge)`.

- [ ] **Step 3: Validate with sheppy's own loader (workstation, no ROS needed)**

```bash
cd /home/swapnil/atdev/rammp-deployments
uv run --with 'sheppy @ git+https://github.com/rammp-org/sheppy@main' --with pyyaml python scripts/validate_manifest.py december_2026/sheppy-manifest.yaml 2>&1 | tail -5
git status --short
```

Expected: the validator reports the manifest OK with no warnings (warnings are failures there). `git status` shows only the manifest modified and `profiles/` untracked. If the validator refuses `kind: process` with `publishes` or complains about a missing `machines` entry, read its message and adjust; do not add `params:` to a `process` alternative (sheppy warns, the validator fails).

- [ ] **Step 4: Sync the manifest and profiles to abra without touching the rest of its checkout**

abra's `~/rammp-deployments` has its own untracked `december_2026/profiles/`; list it first so nothing is overwritten by accident:

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec --target dojo -- bash -c 'ls ~/rammp-deployments/december_2026/profiles/; git -C ~/rammp-deployments status --short'
```

Then build a scratch sync dir (keeps the user's repo free of a `.hil.yml`):

```bash
S=/tmp/claude-1000/-home-swapnil-atdev-rammp-teleop/45d075a5-4ad6-4bb6-8f02-adefc30cd64b/scratchpad/deploy-sync
mkdir -p "$S/profiles"
cp /home/swapnil/atdev/rammp-deployments/december_2026/sheppy-manifest.yaml "$S/"
cp /home/swapnil/atdev/rammp-deployments/december_2026/profiles/teleop-*.yaml "$S/profiles/"
cat > "$S/.hil.yml" <<'EOF'
default_target: dojo
sync:
  - {from: sheppy-manifest.yaml, to: rammp-deployments/december_2026/sheppy-manifest.yaml}
  - {from: profiles/teleop-xbox.yaml, to: rammp-deployments/december_2026/profiles/teleop-xbox.yaml}
  - {from: profiles/teleop-quest.yaml, to: rammp-deployments/december_2026/profiles/teleop-quest.yaml}
EOF
cd "$S" && uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
```

If `hil sync` only syncs directories, sync the scratch dir to `rammp-deployments/december_2026/` after renaming `$S` contents to match (`sheppy-manifest.yaml`, `profiles/`) and confirm with `git -C ~/rammp-deployments diff --stat` on abra that only the manifest changed and only the two profiles were added.

- [ ] **Step 5: Confirm on abra, without starting anything**

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec --target dojo -- bash -c 'cd ~/rammp-deployments && git diff --stat && ls december_2026/profiles/ && grep -n "name: teleop" december_2026/sheppy-manifest.yaml'
```

Expected: the manifest diff, both profiles present, the teleop node found. Do **not** run `sheppy up`.

No commit: this edit is intentionally uncommitted in DEPLOY.

---

### Task 10: Final verification and handoff

- [ ] **Step 1: Full local test run**

Run: `make test && make check`
Expected: all pure tests pass in `rammp_teleop/test` and `tests`, three fragments `ok`.

- [ ] **Step 2: Re-sync and rebuild on abra after the docs/scaffolding commits**

```bash
cd /home/swapnil/atdev/rammp-teleop
uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -c 'cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select rammp_teleop --symlink-install 2>&1 | tail -3 && source install/setup.bash && ros2 launch rammp_teleop quest.launch.py --show-args 2>&1 | head -20'
```

Expected: build OK and the launch args listed (proves the launch file installs and parses).

- [ ] **Step 3: Push the branch**

```bash
git log --oneline
git push -u origin main
```

- [ ] **Step 4: Report** what is verified (pure tests, colcon build, mock reader rate, no-arm exit paths, manifest validation), what is not (Docker image build, any run against the arm, the Quest over real adb), and the two user actions left: `sudo apt install android-tools-adb git-lfs` plus the `oculus_reader` clone on abra, and a `sheppy up teleop-xbox --manifest ~/rammp-deployments/december_2026/sheppy-manifest.yaml` when the cell is free.
