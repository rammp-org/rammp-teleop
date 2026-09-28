"""Xbox-controller teleop through the streaming tier, as velocities.

The sticks command a base-frame tool twist (`ee_twist`) or one joint's rate
(`joint_velocity`). On `ee_twist` the X button toggles between translate mode
(sticks move, D-pad tilts) and rotate mode (sticks turn the tool about the base
axes, no translation); the switch only applies while the sticks are at rest.
The driver's velocity mode integrates the velocities, so there is no
target to run ahead of the arm and nothing to leash. The self-centering sticks
are the deadman: zero velocity is streamed whenever they are at rest. The
gripper is the one position target, integrated from the triggers and re-seeded
from the measured gripper state on every deflected/released edge. Holding Back
for `home_hold_s` sends the arm to `home_joints` through the driver's planner;
the move runs only while Back stays held (release, or any stick input, cancels).

Feeding-test extras (see README "Teach and replay"): A and RB are two taught
waypoint slots, "food" and "mouth". Y + A / Y + RB saves the current joints into
a slot; holding A / RB for `home_hold_s` goes there through the planner, with the
same hold-to-run rule as Back. Holding LB runs the scoop macro on the twist
stream: stab straight down, then lift while tilting the tines up about the
horizontal axis perpendicular to the fork (`fork_axis_tool` says which way the
fork points in the tool frame). Releasing LB, or any stick input, stops it.
"""

from __future__ import annotations

import sys
import time
from typing import Optional

from sensor_msgs.msg import Joy

from rammp_teleop.feeding import (
    HoldButton,
    ScoopMacro,
    ScoopParams,
    WaypointStore,
    quat_rotate,
    tilt_axis,
)
from rammp_teleop.logic import GripperIntegrator, JointSelector, XboxMap
from rammp_teleop.session import TeleopNodeBase, run, twist_msg

ZERO3 = (0.0, 0.0, 0.0)


class XboxTeleopNode(TeleopNodeBase):
    def __init__(self) -> None:
        super().__init__(
            "xbox_teleop", default_controller="ee_twist", default_rate_hz=50.0
        )
        if self.controller not in ("ee_twist", "joint_velocity"):
            raise ValueError(
                "xbox_teleop streams velocities; controller must be ee_twist or joint_velocity"
            )
        dp = self.declare_parameter
        self.joy_topic: str = dp("joy_topic", "/joy").value
        self.deadzone: float = dp("deadzone", 0.15).value
        self.joy_timeout_s: float = dp("joy_timeout_s", 0.5).value

        self.max_linear: float = dp("max_linear_speed", 0.05).value  # m/s at full stick
        self.max_angular: float = dp(
            "max_angular_speed", 0.3
        ).value  # rad/s at full stick
        self.max_joint: float = dp("max_joint_speed", 0.2).value  # rad/s at full stick
        gripper_speed = dp("gripper_speed", 1.0).value  # travel fraction per second
        self.home_joints: list = list(dp("home_joints", [0.0] * 7).value)
        self.home_hold_s: float = dp("home_hold_s", 2.0).value
        # Hold-to-go buttons: Back = home, A = "food" slot, RB = "mouth" slot.
        self.move_buttons = {
            "home": dp("button_home", 6).value,
            "food": dp("button_slot_food", 0).value,
            "mouth": dp("button_slot_mouth", 5).value,
        }
        self._holds = {n: HoldButton(self.home_hold_s) for n in self.move_buttons}
        self._move_name: Optional[str] = None  # slot whose hold started the move
        self.button_shift: int = dp("button_shift", 3).value  # Y (+ slot = save)
        self.slots = WaypointStore(
            dp("waypoints_file", "~/.rammp_teleop/waypoints.json").value
        )
        self.button_scoop: int = dp("button_scoop", 4).value  # LB
        self.fork_axis_tool: list = list(dp("fork_axis_tool", [0.0, 0.0, 1.0]).value)
        self.scoop_params = ScoopParams(
            depth_m=dp("scoop_depth_m", 0.03).value,
            tilt_deg=dp("scoop_tilt_deg", 20.0).value,
            lift_m=dp("scoop_lift_m", 0.05).value,
            linear_speed=dp("scoop_linear_speed", 0.03).value,
            angular_speed=dp("scoop_angular_speed", 0.3).value,
        )
        self._scoop: Optional[ScoopMacro] = None
        self._scoop_armed = False  # LB seen down while stopped: run once per press

        self.map = XboxMap(
            axis_left_x=dp("axis_left_x", 0).value,
            axis_left_y=dp("axis_left_y", 1).value,
            axis_lt=dp("axis_lt", 2).value,
            axis_right_x=dp("axis_right_x", 3).value,
            axis_right_y=dp("axis_right_y", 4).value,
            axis_rt=dp("axis_rt", 5).value,
            axis_dpad_x=dp("axis_dpad_x", 6).value,
            axis_dpad_y=dp("axis_dpad_y", 7).value,
            button_estop=dp("button_estop", 1).value,
            button_estop_clear=dp("button_estop_clear", 7).value,
            button_resync=dp("button_resync", 3).value,
            button_mode=dp("button_mode", 2).value,
        )
        self.rotate = False  # ee_twist: False = translate mode, True = rotate mode

        self.joints = JointSelector()
        self.gripper = GripperIntegrator(gripper_speed)

        self._joy: Optional[Joy] = None
        self._joy_rx_time = 0.0
        self._axes: list = []
        self._prev_buttons: list = []
        self._prev_select_step = 0
        self._active_prev = False

        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.get_logger().info(
            f"Sticks move the arm. X = translate/rotate, hold Back {self.home_hold_s:.0f}s "
            "= home, B = e-stop, Start = clear, Y = re-seed the gripper."
        )
        self.get_logger().info(
            f"Teach: Y+A / Y+RB save 'food' / 'mouth'; hold A / RB {self.home_hold_s:.0f}s "
            f"= go there; hold LB = scoop ({self.scoop_params}). "
            f"Slots on file: {sorted(self.slots.slots)} ({self.slots.path})"
        )

    def engaged_hint(self) -> str:
        return "move a stick"

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg
        self._joy_rx_time = time.monotonic()

    # ------------------------------------------------------------------ hooks

    def tick_input(self, dt: float) -> bool:
        now = time.monotonic()
        joy_fresh = (
            self._joy is not None and (now - self._joy_rx_time) <= self.joy_timeout_s
        )
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
        shift = self.map.pressed(buttons, self.button_shift)
        for name, idx in self.move_buttons.items():
            if shift and name != "home" and rising(idx):
                self._save_slot(name)
            if self._holds[name].update(
                self.map.pressed(buttons, idx) and not shift, now
            ):
                self._start_move(name)
        if self._move_name is not None:
            if not self._homing:
                self._move_name = None
            elif not self.map.pressed(buttons, self.move_buttons[self._move_name]):
                self.cancel_home(f"{self._move_name} button released")
                self._move_name = None
        if rising(self.map.button_mode) and self.uses_twist:
            if self._active_prev:
                self.get_logger().warn("release the sticks to change mode")
            else:
                self.rotate = not self.rotate
                self.get_logger().info(
                    f"mode: {'rotate' if self.rotate else 'translate'}"
                )

        sticks = joy_fresh and self.map.is_active(axes, self.deadzone, self.rotate)
        scoop_held = self.map.pressed(buttons, self.button_scoop) and not shift
        self._tick_scoop(scoop_held, sticks)

        active = sticks or scoop_held
        if active != self._active_prev:
            self.resync("sticks deflected" if active else "sticks released")
            self._active_prev = active
        return active

    # ------------------------------------------------------------------ teach / replay

    def _save_slot(self, name: str) -> None:
        q = self.joint_positions()
        if q is None:
            self.get_logger().warn(f"cannot save '{name}': no /joint_states yet")
            return
        self.slots.save(name, q)
        pose = self.ee_pose()
        self.get_logger().info(
            f"saved '{name}' = {[round(v, 4) for v in q]}"
            + (f" (tool at {[round(v, 3) for v in pose[0]]})" if pose else "")
        )

    def _start_move(self, name: str) -> None:
        joints = self.home_joints if name == "home" else self.slots.get(name)
        if self._active_prev:
            self.get_logger().warn(f"release the sticks to go to '{name}'")
        elif joints is None:
            self.get_logger().warn(f"'{name}' is not taught yet (Y + button saves it)")
        elif len(joints) != 7:
            self.get_logger().warn(f"'{name}' must hold 7 values; not moving")
        else:
            self._move_name = name
            self.go_home(joints, label=f"go-to '{name}'")

    def _tick_scoop(self, held: bool, sticks: bool) -> None:
        log = self.get_logger()
        if not held:
            self._scoop_armed = False
            if self._scoop is not None:
                log.warn(f"scoop stopped ({self._scoop.phase}): LB released")
                self._scoop = None
            return
        if self._scoop is not None:
            if sticks or not self._stream_open:
                why = "stick input" if sticks else "stream closed"
                log.warn(f"scoop stopped ({self._scoop.phase}): {why}")
                self._scoop = None
            elif self._scoop.done:
                log.info("scoop done")
                self._scoop = None
            return
        if self._scoop_armed:
            return
        # LB just went down (or the stream just opened under it): start once.
        if sticks or self._homing or not self.uses_twist:
            self._scoop_armed = True  # needs a fresh press once the arm is free
            if not self.uses_twist:
                log.warn("scoop needs controller=ee_twist")
            return
        if not self._stream_open:
            return  # tick_input returns active, which reopens the stream; retry next tick
        pose = self.ee_pose()
        if pose is None:
            log.warn("cannot scoop: no /ee_state yet")
            self._scoop_armed = True
            return
        axis = tilt_axis(self.fork_axis_tool, pose[1])
        if axis is None:
            log.warn("cannot scoop: the fork is vertical, no tilt axis")
            self._scoop_armed = True
            return
        fork = quat_rotate(pose[1], self.fork_axis_tool)
        self._scoop = ScoopMacro(self.scoop_params, axis)
        self._scoop_armed = True
        log.info(
            f"scoop: fork points {[round(v, 2) for v in fork]} in base, tilting about "
            f"{[round(v, 2) for v in axis]}; stab {self._scoop.t_stab:.1f}s then "
            f"lift/tilt {self._scoop.t_end - self._scoop.t_stab:.1f}s"
        )

    def compute_target(self, dt: float, engaged: bool):
        if self.uses_twist:
            if self._scoop is not None:
                return twist_msg(*self._scoop.step(dt))
            lin, ang = (
                self.map.cartesian_command(self._axes, self.deadzone, self.rotate)
                if engaged
                else (ZERO3, ZERO3)
            )
            return twist_msg(
                [v * self.max_linear for v in lin], [v * self.max_angular for v in ang]
            )

        rate = 0.0
        if engaged:
            step = self.map.joint_select_step(self._axes)
            if step != 0 and self._prev_select_step == 0:
                self.joints.select_next(step)
                self.get_logger().info(f"jogging joint_{self.joints.selected + 1}")
            self._prev_select_step = step
            rate = self.map.joint_jog_command(self._axes, self.deadzone)
        return self.joints.velocities(rate, self.max_joint)

    def gripper_target(self, dt: float, engaged: bool):
        if not engaged:
            return None
        close, open_ = self.map.gripper_command(self._axes)
        return self.gripper.position if self.gripper.step(close, open_, dt) else None

    def seed_from_state(self) -> None:
        g = self.gripper_position()
        if g is not None:
            self.gripper.reset(g)


def main(argv=None) -> int:
    return run(XboxTeleopNode, argv)


if __name__ == "__main__":
    sys.exit(main())
