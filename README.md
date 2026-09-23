# rammp-teleop

Operator-input teleop for the RAMMP Kinova Gen3: an **Xbox pad** and a **Meta
Quest 3** controller, each a ROS 2 node that drives the arm through
[`kinova-gen3-ros2`](https://github.com/rammp-org/kinova-gen3-ros2)'s streaming
tier. The node acquires control, opens a stream on a pose controller, and streams
an absolute target every tick. Nothing moves until the operator engages (a stick
deflection on the pad, grip squeeze on the Quest), and letting go holds the arm
where it is.

| Input    | Launch                                     | Controller                      | Target                                                          |
| -------- | ------------------------------------------ | ------------------------------- | --------------------------------------------------------------- |
| Xbox pad | `ros2 launch rammp_teleop xbox.launch.py`  | `ee_twist` (stiff)              | stick deflection streamed as a base-frame twist                 |
| Quest 3  | `ros2 launch rammp_teleop quest.launch.py` | `ee_pose_impedance` (compliant) | clutched delta of the controller pose, workspace box, step caps |

Each launch file also starts the device reader (`joy_node` for the pad,
`quest_reader` for the headset). Full topic, parameter and button reference:
[`docs/interface.md`](docs/interface.md).

## Build on abra (Humble)

```bash
cd ~/ros2_ws
git clone git@github.com:rammp-org/rammp-teleop.git src/rammp-teleop
source /opt/ros/humble/setup.bash
colcon build --packages-select rammp_teleop --symlink-install
source install/setup.bash
```

`rammp_arm_interfaces` / `rammp_common_interfaces` v1.0.0 must already be in the
workspace.

## Prerequisites

- **Xbox:** `ros-humble-joy` (installed on abra). Plug the pad in; `ros2 topic echo /joy`
  to confirm the axis layout.

- **Quest:** adb and the rail-berkeley reader. git-lfs must be installed **before**
  the clone or the headset APK arrives as a 132-byte pointer and fails silently later.

  ```bash
  sudo apt install android-tools-adb git-lfs
  git lfs install
  git clone https://github.com/rail-berkeley/oculus_reader.git ~/oculus_reader
  pip install -e ~/oculus_reader
  ```

  Connect the headset over USB and accept the debugging prompt inside it. The
  reader and the calibration tool defeat the headset's proximity sensor on
  connect (`keep_awake`, adb `prox_close`) so it keeps tracking while not worn;
  a headset reboot clears that, and the next connect re-sends it. To hand the
  headset back to normal use:

  ```bash
  adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable
  ```

  Then calibrate the controller-to-base rotation once per session:

  ```bash
  ros2 run rammp_teleop quest_calibrate
  ```

  and paste the printed `r_align_euler_zyx_deg` into `rammp_teleop/config/quest.yaml`.

## Run

Against a running `kinova_gen3_node`:

```bash
ros2 launch rammp_teleop xbox.launch.py                      # ee_twist
ros2 launch rammp_teleop xbox.launch.py controller:=joint_velocity
ros2 launch rammp_teleop quest.launch.py                     # ee_pose_impedance
ros2 launch rammp_teleop quest.launch.py mock:=true          # scripted controller, no headset
```

Sanity checks: `ros2 topic echo /stream_status` (open: true, the controller you
asked for) and `ros2 topic echo --qos-reliability best_effort /setpoint/twist`.

## Tests

Pure-Python logic tests, no ROS needed:

```bash
make test
```

On abra's Python, disable the outdated ament pytest plugins:
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest rammp_teleop/test`.

## Container

Built on `rammp-base` (arm64 only). `make build`, `make smoke` (runs the mock Quest
pipeline and checks it exits on SIGTERM), `make check` (validates the fragments).
The three fragments `rammp-alternative.{xbox,quest,quest-mock}.yaml` are what
`rammp-deployments` transcribes into its manifest.
