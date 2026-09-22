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
    def __init__(
        self, name: str, *, default_controller: str, default_rate_hz: float
    ) -> None:
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
            raise ValueError(
                "stream_timeout_s must exceed the publish period or the session will expire"
            )
        self.uses_pose = self.controller in POSE_CONTROLLERS

        # Set by the signal handler in run(); every blocking wait in setup()
        # checks it so SIGINT/SIGTERM during startup ends the node promptly.
        self.stop_requested = False

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
        self.create_subscription(
            EeState, "ee_state", self._on_ee_state, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, "joint_states", self._on_joint_states, qos_profile_sensor_data
        )
        self.create_subscription(
            GripperState,
            "gripper_state",
            self._on_gripper_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            StreamStatus, "stream_status", self._on_stream_status, LATCHED_QOS
        )
        self.create_subscription(
            ControlStatus, "control_status", self._on_control_status, LATCHED_QOS
        )

        self._gripper_pub = self.create_publisher(
            GripperSetpoint, "/setpoint/gripper", SETPOINT_QOS
        )
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
        """Short operator instruction used in log lines, e.g. 'move a stick'."""
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
            self.get_logger().info(
                f"stream open on {self.controller} -> {list(msg.channels)}"
            )

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
        lost = msg.generation > self._generation or (
            msg.generation == self._generation and not msg.owned
        )
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
        self.get_logger().warn(
            f"/estop {'ENGAGED' if engaged else 'cleared'}: {reason}"
        )

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
            if self.stop_requested:
                return False
            if time.monotonic() > deadline:
                log.error(
                    "no /ee_state or /joint_states received; is kinova_gen3_node running?"
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.2)
        log.info("arm state received")

        resp = self._call(
            self._acquire,
            AcquireControl.Request(owner_id=self.owner_id),
            "acquire_control",
        )
        if self.stop_requested:
            return False
        if resp is None or not resp.accepted:
            log.error(f"acquire_control refused: {getattr(resp, 'message', '')}")
            return False
        self._token = np.frombuffer(bytes(resp.token), dtype=np.uint8).copy()
        self._token_valid = True
        self._generation = int(resp.generation)
        log.info(
            f"control acquired as {self.owner_id!r} (generation {resp.generation})"
        )

        resp = self._call(self._list, ListControllers.Request(), "list_controllers")
        if resp is None:
            return False
        caps = {c.name: c for c in resp.controllers}
        cap = caps.get(self.controller)
        if cap is None or not cap.available:
            log.error(
                f"controller {self.controller!r} not available; driver offers "
                f"{[c.name for c in resp.controllers if c.available]}"
            )
            return False
        self._channel = cap.channels[0]

        msg_type = PoseSetpoint if self.uses_pose else JointSetpoint
        self._setpoint_pub = self.create_publisher(
            msg_type, self._channel, SETPOINT_QOS
        )
        settle_deadline = time.monotonic() + 3.0
        while (
            self._setpoint_pub.get_subscription_count() == 0
            and time.monotonic() < settle_deadline
        ):
            if self.stop_requested:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._setpoint_pub.get_subscription_count() == 0:
            log.warn(f"no subscriber discovered on {self._channel} yet; opening anyway")

        self.resync("startup")
        if not self._open_stream_blocking():
            return False

        self._last_tick = time.monotonic()
        self._tick_timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        log.info(
            f"teleop ready on {self.controller} -> {self._channel} at {self.rate_hz:.0f} Hz"
        )
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
            self.get_logger().error(
                f"open_stream refused: {getattr(resp, 'message', '')}"
            )
            return False
        self._stream_open = True
        self._opened_at_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"stream opened on {self.controller}, channels {list(resp.channels)}"
        )
        return True

    def _try_reopen_async(self) -> None:
        now = time.monotonic()
        if (
            self._reopen_inflight
            or now - self._last_reopen_attempt < self.reopen_interval_s
        ):
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
                self.get_logger().warn(
                    f"reopen refused: {getattr(resp, 'message', f.exception())}",
                    throttle_duration_sec=5.0,
                )

        fut.add_done_callback(done)

    def _try_reacquire_async(self) -> None:
        """Re-acquire after an e-stop/revoke cleared ownership. Never seizes from another owner."""
        now = time.monotonic()
        if (
            self._reacquire_inflight
            or now - self._last_reacquire_attempt < self.reopen_interval_s
        ):
            return
        self._last_reacquire_attempt = now
        if self._owned_by_other:
            self.get_logger().warn(
                "arm is owned by someone else; not seizing", throttle_duration_sec=5.0
            )
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
                self._stream_open = (
                    False  # a new generation never carries an open session
                )
                self.get_logger().info(
                    f"control re-acquired (generation {resp.generation})"
                )
            else:
                self.get_logger().warn(
                    f"re-acquire refused: {getattr(resp, 'message', f.exception())}",
                    throttle_duration_sec=5.0,
                )

        fut.add_done_callback(done)

    def teardown(self) -> None:
        if self._token is None:
            return
        if self._tick_timer is not None:
            self._tick_timer.cancel()
        if self._stream_open:
            self._call(
                self._close,
                CloseStream.Request(token=self._token),
                "close_stream",
                timeout_s=2.0,
            )
        if self._token_valid:
            self._call(
                self._release,
                ReleaseControl.Request(token=self._token),
                "release_control",
                timeout_s=2.0,
            )
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
    node = node_factory()

    def on_signal(signum, _frame):
        node.stop_requested = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    code = 0
    try:
        if not node.setup():
            code = 1
        else:
            while rclpy.ok() and not node.stop_requested:
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
