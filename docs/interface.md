# Interface reference

One ROS 2 package, `rammp_teleop`, with one launch file per input. Every teleop
node is a client of `kinova_gen3_node`'s streaming tier; the arm driver must
already be running.

## Topics

Published by every teleop node:

| Topic | Type | Notes |
|---|---|---|
| `/setpoint/pose` | `rammp_arm_interfaces/PoseSetpoint` | absolute tool pose, base frame, every tick while the stream is open (pose controllers) |
| `/setpoint/joint_position` | `rammp_arm_interfaces/JointSetpoint` | instead of the pose topic when a joint controller is selected (Xbox only) |
| `/setpoint/gripper` | `rammp_arm_interfaces/GripperSetpoint` | on change, carrying the arm's token |
| `/estop` | `rammp_common_interfaces/EStop` | on button press |

Published by `quest_reader` only:

| Topic | Type | Notes |
|---|---|---|
| `/quest/<hand>/pose` | `geometry_msgs/PoseStamped` | raw controller pose in the headset tracking frame, `frame_id: quest` |
| `/quest/joy` | `sensor_msgs/Joy` | `axes = [index trigger 0..1]`, `buttons = [grip, A, B]` (`[grip, X, Y]` for the left hand) |

Subscribed:

| Topic | Type | Used for |
|---|---|---|
| `/ee_state` | `rammp_arm_interfaces/EeState` | seeding and leashing the target (pose controllers) |
| `/joint_states` | `sensor_msgs/JointState` | seeding and leashing (joint controllers) |
| `/gripper_state` | `rammp_arm_interfaces/GripperState` | seeding the gripper target |
| `/stream_status` | `rammp_arm_interfaces/StreamStatus` | noticing a stream the driver closed |
| `/control_status` | `rammp_common_interfaces/ControlStatus` | noticing lost or seized ownership, e-stop |
| `/joy` | `sensor_msgs/Joy` | Xbox only, from `joy_node` |
| `/quest/<hand>/pose`, `/quest/joy` | see above | Quest only, from `quest_reader` |

## Services used

`/acquire_control`, `/list_controllers`, `/open_stream`, `/close_stream`,
`/release_control`. Note that acquire **seizes**: do not start a teleop node while
an orchestrator owns the arm.

## Session behaviour

1. Wait for `/ee_state` (or `/joint_states`) so the target starts from reality.
2. `/acquire_control` with `owner_id` -> token, stamped on every setpoint.
3. `/list_controllers` -> the selected controller must be `available`; its channel
   names the setpoint topic.
4. Create the setpoint publisher and wait for the driver to subscribe (DDS
   discovery must settle before the stream opens, or the first setpoints go
   nowhere and the session expires).
5. `/open_stream {controller, timeout_s, token}`.
6. Publish the target every tick, also while the operator is idle, so the arm holds
   and the session stays alive.
7. On SIGINT/SIGTERM: `/close_stream`, `/release_control`.

Recovery: an e-stop or `/revoke_control` clears ownership and tears down the
session. The node sees it on `/control_status` and `/stream_status`; the next
engage re-acquires (only if nobody else owns the arm) and reopens the stream.

Setpoint QoS is BEST_EFFORT / KEEP_LAST / depth 1. `stream_timeout_s` must exceed
`1 / rate_hz`.

## Xbox controls

Nothing moves unless **LB (deadman) is held**. On every LB press and release the
target is re-seeded from the measured state, so letting go stops the arm where it is.

| Input | `ee_pose_position` (default) | `joint_position` |
|---|---|---|
| Left stick up/down | +x / -x (base frame) | jog selected joint +/- |
| Left stick left/right | +y / -y | |
| Right stick up/down | +z / -z | |
| Right stick left/right | yaw about +z (left = CCW) | |
| D-pad up/down | pitch about y | |
| D-pad left/right | roll about x | previous / next joint |
| RT / LT | close / open gripper (incremental) | same |
| **B** | publish `/estop engaged: true` | same |
| **Start** | publish `/estop engaged: false` | same |
| **Y** | re-seed target from measured state | same |

Cartesian increments are applied in the base frame (world-aligned), matching
`PoseSetpoint`'s frame and `/ee_state`'s `LOCAL_WORLD_ALIGNED` convention.

Joy layout this was written against (Xbox Series X over USB, `ros-humble-joy` 3.3):
axes `[LX, LY, LT, RX, RY, RT, DpadX, DpadY]` with sticks +1 = left/up and triggers
+1 = released; buttons `[A, B, X, Y, LB, RB, Back, Start, Guide, LS, RS, Share]`.
All indices are parameters if your pad differs.

## Quest controls

| Input | Action |
|---|---|
| **Grip squeeze** | clutch: hold to move; the controller's motion from the squeeze is applied to the EE pose from the squeeze |
| **Index trigger** | gripper, analog 0..1 (or binary with `gripper_binary`) |
| **A** | re-seed: forget the freeze target and references (next squeeze re-captures) |
| **B** | publish `/estop engaged: true` |

There is no e-stop clear on the headset: use the Xbox pad's Start or
`ros2 topic pub --once /estop rammp_common_interfaces/msg/EStop "{engaged: false, source: operator}"`.

Translation is always base frame. Rotation acts about base axes (`rot_frame: base`)
or the tool's own axes (`rot_frame: tool`). Controller jumps larger than
`jump_pos_tol` / `jump_rot_tol` per tick are dropped. The target is clamped to the
workspace box, then rate-limited to `max_lin_step` / `max_ang_step` per tick. When
released, the last safe target is held (not the measured pose, which sags on a
compliant controller).

## Parameters

### `xbox_teleop` (config/xbox.yaml)

| Parameter | Default | Meaning |
|---|---|---|
| `controller` | `ee_pose_position` | `ee_pose_position`, `ee_pose_impedance`, `joint_position`, `joint_impedance` |
| `owner_id` | `xbox_teleop` | name sent to `/acquire_control` |
| `rate_hz` / `stream_timeout_s` | 50 / 0.2 | publish rate and `open_stream` deadline |
| `joy_topic` | `/joy` | |
| `deadzone` | 0.15 | stick deadzone, rescaled so full deflection still gives max speed |
| `joy_timeout_s` | 0.5 | no `/joy` for this long => sticks neutral, deadman released |
| `max_linear_speed` / `max_angular_speed` | 0.05 m/s / 0.3 rad/s | at full stick |
| `max_joint_speed` | 0.2 rad/s | joint jog at full stick |
| `target_lead_m` / `target_lead_rad` | 0.05 / 0.2 | leash: how far the target may lead the measured pose; 0 disables |
| `joint_target_lead_rad` | 0.1 | leash per joint |
| `gripper_speed` | 1.0 | full travel per second at full trigger |
| `gripper_cmd_speed` / `gripper_force` | 0.5 / 0.3 | `GripperSetpoint.speed` / `.force` |
| `axis_*`, `button_*` | see file | joy indices |
| `state_wait_timeout_s` | 10 | give up if no arm state arrives |
| `reopen_interval_s` | 1.0 | throttle for reopen / re-acquire attempts |

### `quest_teleop` (config/quest.yaml)

| Parameter | Default | Meaning |
|---|---|---|
| `controller` | `ee_pose_impedance` | or `ee_pose_position` |
| `rate_hz` / `stream_timeout_s` | 60 / 0.2 | |
| `hand` | `right` | which controller |
| `input_timeout_s` | 0.5 | no `/quest/*` for this long => grip reads as released |
| `r_align_euler_zyx_deg` | per session | controller-frame to base-frame rotation; see Calibration |
| `trans_smooth` / `rot_smooth` | 0.8 | first-order lag on the command, (0, 1]; 1.0 = raw |
| `jump_pos_tol` / `jump_rot_tol` | 0.05 m / 0.5 rad | per-tick jump rejection |
| `pos_scale` | 1.0 | controller motion to EE motion |
| `rot_frame` / `r_tool_euler_zyx_deg` | `base` / `[0,0,0]` | rotation frame and controller-body to tool-body axes (tool mode) |
| `ws_min` / `ws_max` | see file | workspace box, base frame, m |
| `max_lin_step` / `max_ang_step` | 0.01 m / 0.05 rad | per tick (0.6 m/s, 3 rad/s at 60 Hz) |
| `gripper_binary` / `gripper_threshold` | false / 0.5 | |
| `gripper_cmd_speed` / `gripper_force` | 0.5 / 0.3 | |
| `button_grip` / `button_resync` / `button_estop` / `axis_trigger` | 0 / 1 / 2 / 0 | indices into `/quest/joy` |

### `quest_reader`

| Parameter | Default | Meaning |
|---|---|---|
| `hand` | `right` | |
| `quest_ip` | `""` | empty = USB; set for adb over Wi-Fi |
| `rate_hz` | 60 | |
| `mock` | false | scripted controller, no headset, no adb |
| `frame_id` | `quest` | |

## Calibration

The Quest's tracking frame yaw depends on how the headset was oriented at boot,
so `r_align_euler_zyx_deg` is a per-session value:

```bash
ros2 run rammp_teleop quest_calibrate            # USB
ros2 run rammp_teleop quest_calibrate --quest-ip 192.168.1.50
```

Move the hand forward, left and up as prompted; paste the printed triple into
`config/quest.yaml` (or pass a `params_file`).

## Hardware-free

```bash
ros2 launch rammp_teleop quest.launch.py mock:=true
```

`quest_reader` replays a scripted engage / move / grip / release loop, so the whole
pipeline can be exercised against a simulated or mock arm.
