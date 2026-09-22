# rammp-teleop design

Date: 2026-09-21. Status: approved in conversation; scope is xbox + Quest 3 only.

## Goal

A RAMMP module repo holding operator-input teleop for the Kinova Gen3 arm on
the December 2026 deployment. Two inputs ship now, an Xbox pad and a Meta
Quest 3 controller. Both drive the arm through `kinova-gen3-ros2`'s streaming
tier (token session, `PoseSetpoint` on `/setpoint/pose`, `GripperSetpoint` on
`/setpoint/gripper`). The package builds in a colcon workspace on abra today
and in a rammp-base Docker image later; both ways of running are wired from
the start.

Out of scope: SpaceMouse (the layout leaves room for it), gain streaming to
the driver (no ROS surface exists), any change to the arm driver.

## Sources being merged

- `abra:~/ros2_ws/src/kinova-xbox-teleop` (one local commit, never pushed):
  a working ament_python Xbox teleop. Its session handling (acquire, list
  controllers, open stream, reopen/re-acquire, teardown), integrators with a
  leash, deadman reseeding, gripper, e-stop, and pure-logic tests are the
  base of this package. Pulled to the workstation for the port.
- `rammp-org/kinova-quest-teleop@main` (0bc99be; abra's checkout is stale):
  the Quest supervisor. `mapping.py` (`ClutchedDeltaMapper`), `safety.py`
  (`SafetyFilter`), `transforms.py`, `pose_source.py` (`parse_oculus_sample`,
  `OculusPoseSource`, `MockPoseSource`, scripted scenarios) and `calibrate.py`
  port over with their tests. The UDP protocol, driver client, loop pacing and
  gain packets do not.

The working Quest setup ran the driver-side socket server with
`--joint-impedance`. The ROS driver's `ee_pose_impedance` streaming controller
is that same law (JointImpedanceMode with in-loop IK) with the same tuned
gains (Kq 80/30, zeta 0.5, in driver v1.0.0 which the arm image pins). The
Quest node therefore streams pose setpoints on `ee_pose_impedance`.

## Repo layout

```
rammp-teleop/
  rammp_teleop/                  ament_python package (repo root is the colcon src root)
    package.xml  setup.py  setup.cfg  resource/rammp_teleop
    rammp_teleop/
      __init__.py
      logic.py                   deadzone, quaternion helpers, Cartesian/Joint/Gripper integrators  (from xbox)
      session.py                 TeleopNodeBase: arm session + setpoint/gripper/estop publishing (from xbox node)
      xbox_node.py               XboxTeleopNode(TeleopNodeBase): /joy -> integrators -> target
      quest_reader.py            QuestReaderNode: oculus_reader -> /quest/<hand>/pose + /quest/joy
      quest_node.py              QuestTeleopNode(TeleopNodeBase): clutched delta mapping + safety -> target
      quest/
        __init__.py  transforms.py  mapping.py  safety.py  pose_source.py  scenarios.py  calibrate.py   (ported)
    launch/xbox.launch.py  launch/quest.launch.py
    config/xbox.yaml  config/quest.yaml
    test/test_logic.py  test/test_quest_*.py  test/test_quest_reader_parse.py
  Dockerfile  Makefile  scripts/validate_fragment.py  scripts/smoke.sh  tests/test_validate_fragment.py
  rammp-alternative.xbox.yaml  rammp-alternative.quest.yaml  rammp-alternative.quest-mock.yaml
  docs/interface.md  docs/index.mdx  docs/_meta.js
  .hil.yml  .pre-commit-config.yaml  .github/workflows/{build,lint}.yml  .gitignore  .dockerignore
  README.md  CONTRIBUTING.md
```

One package, not three: the session code is shared, and switching input is
one launch file. Device-specific imports never happen at build time. `joy` is
an exec-only dependency started by the launch file. `oculus_reader` is
imported lazily inside `QuestReaderNode`, so the package builds and the Xbox
node runs without it or adb present. scipy is needed by the Quest mapper and
is on abra (1.15.3).

## Shared: `TeleopNodeBase` (session.py)

Extracted from `XboxTeleopNode` with behaviour unchanged:

1. wait for `/ee_state` (or `/joint_states` for `joint_position`), timeout
   `state_wait_timeout_s`;
1. `/acquire_control(owner_id)` -> token (seizes; documented);
1. `/list_controllers` -> confirm `controller` available, take its channel;
1. create the setpoint publisher (`PoseSetpoint` or `JointSetpoint`) with
   BEST_EFFORT/KEEP_LAST/1, wait up to 3 s for a subscriber;
1. `/open_stream(controller, stream_timeout_s, token)`;
1. timer at `rate_hz` calls `tick(dt)`; the subclass returns the target
   (`Pose` or joint list, or `None` to publish nothing this tick); the base
   publishes it with the token every tick, including while the input is
   idle, so the session stays alive and the arm holds;
1. shutdown (`SIGINT`/`SIGTERM`, own handler): `/close_stream`,
   `/release_control`.

Also in the base: `/control_status` and `/stream_status` watching with the
generation logic from the Xbox node (lost ownership, seized by other owner,
stream closed by driver), `try_reopen`/`try_reacquire` throttled by
`reopen_interval_s` and only when the subclass reports the operator is
engaged, `publish_gripper(position)` (speed/force from params, token),
`publish_estop(engaged, reason)`, and `resync(why)` which the subclass
implements by re-seeding its target from measured state.

Parameters (all nodes): `controller`, `owner_id`, `rate_hz`,
`stream_timeout_s` (> 1/rate_hz, validated), `state_wait_timeout_s`,
`reopen_interval_s`, `gripper_cmd_speed`, `gripper_force`.

## Xbox

`xbox.launch.py` starts `joy_node` (ros-humble-joy, SDL) and `xbox_teleop`
with `config/xbox.yaml`; launch args `params_file`, `controller`, `device_id`.
Node behaviour, controls and parameters are exactly those of
`kinova_xbox_teleop` with one change: there is no LB deadman. The
self-centering sticks are the deadman; the arm moves while any stick, D-pad
direction or trigger is deflected past the deadzone and holds otherwise, with
the target re-seeded from the measured state on each deflected/released edge.
Sticks and D-pad map to base-frame increments, RT/LT gripper, B e-stop, Start
clear, Y resync, `joint_position` jog mode. Default controller `ee_pose_position`
at 50 Hz, the tested configuration.

## Quest

Two nodes, so the raw controller stream is a normal ROS topic that can be
echoed and recorded, and the teleop node has no headset dependency.

### `quest_reader`

Wraps `OculusReader` (rail-berkeley `oculus_reader`, installed out of tree
with git-lfs for the APK, `adb` on PATH). Polls at `rate_hz` (60) and
publishes:

- `/quest/<hand>/pose` `geometry_msgs/PoseStamped`, frame_id `quest`, the
  controller pose in the headset tracking frame (4x4 -> position + xyzw
  quaternion, converted from the wxyz used internally);
- `/quest/joy` `sensor_msgs/Joy`: `axes = [trigger 0..1]`,
  `buttons = [grip, A, B]` for the selected hand.

Decoding is `parse_oculus_sample` from the original repo (missing transform
=> no pose published this tick and grip reported released). Parameters:
`hand` (`right` default), `quest_ip` (empty = USB), `rate_hz`, `mock`
(true = `MockPoseSource` with `default_script()`, no headset, no adb).

### `quest_teleop`

Subscribes to the two topics above plus the arm state. Each tick at
`rate_hz` (60):

1. if the pose is older than `input_timeout_s`, grip is treated as released;
1. `ClutchedDeltaMapper.update(ctrl_pose, grip, ee_pos, ee_rot)` with the
   measured EE from `/ee_state` (the mapper captures the EE reference on the
   grip rising edge, so engage never jumps);
1. engaged: `SafetyFilter.filter(raw_pos, raw_rot)` (workspace box clamp
   then per-tick linear/angular step clamp) -> target;
   not engaged: publish the latched freeze target (last safe target, or the
   EE pose at first freeze) and `SafetyFilter.reset(ee)`; this is the
   original loop's behaviour so a compliant arm does not chase its own sag;
1. gripper: trigger 0..1 -> `publish_gripper` (binary option kept);
1. buttons: B rising edge -> e-stop engaged; A rising edge -> resync
   (re-capture references, clear freeze target). E-stop clear is not on the
   headset; use the Xbox pad or `ros2 topic pub /estop`.

Parameters mirror `TeleopConfig` minus driver/gains: `r_align_euler_zyx_deg`,
`ws_min`, `ws_max`, `trans_smooth`, `rot_smooth`, `jump_pos_tol`,
`jump_rot_tol`, `pos_scale`, `rot_frame`, `r_tool_euler_zyx_deg`,
`max_lin_step`, `max_ang_step`, `gripper_binary`, `gripper_threshold`,
`input_timeout_s`. Default controller `ee_pose_impedance`. The checked-in
`r_align` is a per-session calibration and the config file says so.

`quest_calibrate` console script: the original three-gesture Procrustes
solve, reading the headset directly, printing the euler triple to paste into
`config/quest.yaml`.

`quest.launch.py` starts both nodes with `config/quest.yaml`; args
`params_file`, `controller`, `hand`, `quest_ip`, `mock`.

## Container and fragments

`Dockerfile`: `FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6`; apt
`ros-humble-joy android-tools-adb git-lfs python3-scipy`; clone
`oculus_reader` with `git lfs install` before the clone and `pip install -e`;
`COPY . /module_ws/src/rammp-teleop`, `rosdep install`, `colcon build`,
strip build/log. Command is a `ros2 launch`, which forwards SIGTERM to its
children (checked by `scripts/smoke.sh`).

Fragments (sheppy `alternative` schema + `tier: experimental`), all
`network_mode: host`, `ipc: host`, `privileged: true`, `volumes: [/dev:/dev]`
for the pad or the headset USB, `environment: {ROS_DOMAIN_ID: "0"}`:

- `rammp-alternative.xbox.yaml` id `xbox_teleop`, command
  `ros2 launch rammp_teleop xbox.launch.py`;
- `rammp-alternative.quest.yaml` id `quest_teleop`, command
  `ros2 launch rammp_teleop quest.launch.py`;
- `rammp-alternative.quest-mock.yaml` id `quest_teleop_mock`, command
  `ros2 launch rammp_teleop quest.launch.py mock:=true`, no devices.

All declare `publishes: [/setpoint/pose, /setpoint/gripper, /estop]` and
`subscribes: [/ee_state, /joint_states, /gripper_state, /stream_status, /control_status]` (Quest adds `/quest/*`).

`Makefile`: `build`, `run`, `check` (validate all `rammp-alternative*.yaml`),
`smoke`, `lint`, as in the template. CI: `build.yml` (fragment validation +
`pytest`, docker build + smoke on `ubuntu-22.04-arm`, publish to GHCR on
`dev`/`main`/tags) and `lint.yml` (pre-commit).

## Deployment (local, uncommitted)

`~/atdev/rammp-deployments/december_2026/sheppy-manifest.yaml` gains a machine
and a node, using sheppy's native `launch_file` kind (which runs
`exec ros2 launch <package> <file> k:=v` after sourcing the machine's
`ros_setup`):

```yaml
machines:
  - name: abra
    host: abra
    user: abra
    ros_setup: ~/ros2_ws/install/setup.bash

  - name: teleop
    description: Operator input -> /setpoint/pose and /setpoint/gripper through the arm's streaming tier
    select: single
    alternatives:
      - id: xbox_local
        kind: launch_file
        machine: abra
        package: rammp_teleop
        launch_file: xbox.launch.py
        publishes: [/setpoint/pose, /setpoint/gripper, /estop]
        subscribes: [/ee_state, /joint_states, /gripper_state, /stream_status, /control_status]
      - id: quest3_local        # same, quest.launch.py
      - id: quest3_mock_local   # same, params: {mock: true} -> mock:=true
```

plus `december_2026/profiles/teleop-xbox.yaml` and `teleop-quest.yaml`
selecting the real arm, planner, cameras, foxglove and the teleop input. A
profile that does not want teleop simply leaves the node unselected; there is
no "off" alternative. Sheppy has no local-path mechanism, so sourcing the abra
workspace through `ros_setup` is how the local checkout is reached; the
workspace's `install/setup.bash` chains to `/opt/ros/humble`, and sheppyd on
abra already carries `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` from the shell
that started it. The docker fragments above replace these alternatives once an
image is published. The edit is synced to abra's `~/rammp-deployments` checkout
as files (abra's tracked tree is clean; the change is recoverable with
`git checkout`).

`.hil.yml` in this repo syncs the checkout to `abra:~/ros2_ws/src/rammp-teleop`.
The old `kinova-xbox-teleop` package on abra stays until the new one builds;
its removal is the user's call.

## Verification

- Workstation: `uv run --with numpy --with scipy --with pytest pytest rammp_teleop/test` for every pure-logic module (ported xbox tests, ported
  Quest mapping/safety/transforms/pose-source/scenario tests, new tests for
  the freeze/engage logic in `quest_node` factored as a pure class), `make check` for the fragments.
- abra: `colcon build --packages-select rammp_teleop`, `colcon test`;
  `ros2 launch rammp_teleop quest.launch.py mock:=true` under
  `ROS_DOMAIN_ID=42` publishes `/quest/joy` and `/quest/right/pose` at ~60 Hz
  and the teleop node exits 1 after `state_wait_timeout_s` with no arm;
  `ros2 run rammp_teleop xbox_teleop` likewise exits cleanly with no arm.
- No `sheppy up` on abra as part of this work: another stack is live there.
  Hardware runs are the user's.

## Prerequisites on abra (sudo, user runs)

```
sudo apt install android-tools-adb git-lfs
git lfs install && git clone https://github.com/rail-berkeley/oculus_reader.git ~/oculus_reader && pip install -e ~/oculus_reader
```

Xbox needs nothing new: `ros-humble-joy` 3.3 is installed.
