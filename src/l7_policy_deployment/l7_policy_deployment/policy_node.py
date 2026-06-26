from __future__ import annotations

from enum import Enum
import math
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import torch

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from l6_rl_training.env import L6AkpfEnv
from l6_rl_training.geometry import SCENARIOS
from l6_rl_training.train_ppo import ActorCritic


class MissionState(Enum):
    WAITING_DEPS = "WAITING_DEPS"
    WAITING_FCU = "WAITING_FCU"
    SETTING_GUIDED = "SETTING_GUIDED"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    NAVIGATING = "NAVIGATING"
    HOVER = "HOVER"
    LANDING = "LANDING"
    DONE = "DONE"
    FAILSAFE = "FAILSAFE"


class L7PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("l7_policy_node")
        self.scenario_name = self.declare_parameter("scenario", "S1_single_front_obstacle").value
        self.policy_path = Path(self.declare_parameter("policy_path", "/tmp/l6_akpf_bc_pass/policy.pt").value)
        self.raw_cmd_topic = self.declare_parameter("raw_cmd_topic", "/l5/raw_cmd_vel").value
        self.odom_topic = self.declare_parameter("odom_topic", "/mavros/local_position/local").value
        self.pose_topic = self.declare_parameter("pose_topic", "/mavros/local_position/pose").value
        self.state_topic = self.declare_parameter("state_topic", "/mavros/state").value
        self.shield_active_topic = self.declare_parameter("shield_active_topic", "/l5/shield_active").value
        self.publish_rate_hz = max(5.0, float(self.declare_parameter("publish_rate_hz", 20.0).value))
        self.takeoff_alt = float(self.declare_parameter("takeoff_alt", 2.0).value)
        self.goal_radius = float(self.declare_parameter("goal_radius", 0.50).value)
        self.max_xy_speed = float(self.declare_parameter("max_xy_speed", 0.35).value)
        self.max_z_speed = float(self.declare_parameter("max_z_speed", 0.22).value)
        self.z_gain = float(self.declare_parameter("z_gain", 0.45).value)
        self.hover_s = float(self.declare_parameter("hover_s", 3.0).value)
        self.mission_timeout_s = float(self.declare_parameter("mission_timeout_s", 220.0).value)
        self.auto_mission = bool(self.declare_parameter("auto_mission", True).value)
        self.require_guided = bool(self.declare_parameter("require_guided", True).value)
        self.policy_speed_scale = float(self.declare_parameter("policy_speed_scale", 0.50).value)

        if self.scenario_name not in SCENARIOS:
            raise RuntimeError(f"unknown L7 scenario: {self.scenario_name}")
        self.env, self.model = self._load_policy()
        self.env.scenario_name = self.scenario_name
        self.env.scenario = SCENARIOS[self.scenario_name]
        self.env.goal_radius = self.goal_radius

        velocity_qos = QoSProfile(depth=10)
        velocity_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        velocity_qos.durability = DurabilityPolicy.VOLATILE
        self.cmd_pub = self.create_publisher(TwistStamped, self.raw_cmd_topic, velocity_qos)

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_sub = self.create_subscription(State, self.state_topic, self._state_cb, state_qos)
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        odom_qos.durability = DurabilityPolicy.VOLATILE
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, odom_qos)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, odom_qos)
        self.shield_sub = self.create_subscription(Bool, self.shield_active_topic, self._shield_cb, 10)
        self.status_pub = self.create_publisher(String, "/l7/policy_status", 10)

        self.arming_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_cli = self.create_client(CommandTOL, "/mavros/cmd/land")

        self.current_state = State()
        self.current_pos = np.zeros(3, dtype=np.float64)
        self.current_vel = np.zeros(3, dtype=np.float64)
        self.have_state = False
        self.have_odom = False
        self.shield_active = False
        self.mission_origin: Optional[np.ndarray] = None
        self.last_pose_time = None
        self.state = MissionState.WAITING_DEPS
        self.state_entry_time = self.get_clock().now()
        self.mission_start_time = self.state_entry_time
        self.last_request_time = self.state_entry_time - Duration(seconds=10.0)
        self.last_log_time = self.state_entry_time - Duration(seconds=10.0)
        self.takeoff_sent = False
        self.land_sent = False
        self.pending_set_mode = None
        self.pending_arm = None
        self.pending_takeoff = None
        self.inference_ms = 0.0
        self.min_goal_dist = math.inf

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._loop)
        self.get_logger().info(
            f"L7 policy node started scenario={self.scenario_name} policy={self.policy_path} "
            f"raw_cmd={self.raw_cmd_topic} auto_mission={self.auto_mission}"
        )

    def _load_policy(self):
        checkpoint = torch.load(self.policy_path, map_location="cpu")
        config = checkpoint["config"]
        env = L6AkpfEnv(
            scenario=self.scenario_name,
            observation_mode=config["obs_mode"],
            use_shield=config["use_shield"],
            seed=0,
        )
        model = ActorCritic(checkpoint["obs_size"], checkpoint["action_size"])
        model.load_state_dict(checkpoint["model_state"])
        model.eval()
        return env, model

    def _state_cb(self, msg: State) -> None:
        self.current_state = msg
        self.have_state = True

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_pos = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=np.float64,
        )
        self.current_vel = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
            dtype=np.float64,
        )
        self.last_pose_time = self.get_clock().now()
        self.have_odom = True

    def _pose_cb(self, msg: PoseStamped) -> None:
        next_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        stamp = self.get_clock().now()
        if self.have_odom and self.last_pose_time is not None:
            dt = (stamp - self.last_pose_time).nanoseconds * 1e-9
            if 1e-3 < dt < 1.0:
                self.current_vel = (next_pos - self.current_pos) / dt
        self.current_pos = next_pos
        self.last_pose_time = stamp
        self.have_odom = True

    def _shield_cb(self, msg: Bool) -> None:
        self.shield_active = bool(msg.data)

    def _loop(self) -> None:
        self._handle_futures()
        if self.state not in {MissionState.DONE, MissionState.FAILSAFE}:
            if (self.get_clock().now() - self.mission_start_time).nanoseconds * 1e-9 > self.mission_timeout_s:
                self.get_logger().error(f"L7 mission timeout {self.mission_timeout_s:.1f}s")
                self._enter(MissionState.FAILSAFE)

        if self.state == MissionState.WAITING_DEPS:
            if self._deps_ready():
                self._capture_origin()
                self._enter(MissionState.NAVIGATING if not self.auto_mission else MissionState.WAITING_FCU)
            else:
                self._log_waiting()
        elif self.state == MissionState.WAITING_FCU:
            if self.current_state.connected:
                self._enter(MissionState.SETTING_GUIDED)
        elif self.state == MissionState.SETTING_GUIDED:
            if self.current_state.mode == "GUIDED":
                self._enter(MissionState.ARMING)
            elif self._request_due() and self.pending_set_mode is None:
                self._call_set_mode("GUIDED")
        elif self.state == MissionState.ARMING:
            if self.current_state.armed:
                self._capture_origin()
                self._enter(MissionState.TAKEOFF)
            elif self._request_due() and self.pending_arm is None:
                self._call_arm(True)
        elif self.state == MissionState.TAKEOFF:
            if not self.takeoff_sent:
                self._call_takeoff(self.takeoff_alt)
                self.takeoff_sent = True
            rel_z = self.current_pos[2] - self._origin()[2]
            if rel_z >= self.takeoff_alt * 0.92:
                self._enter(MissionState.NAVIGATING)
        elif self.state == MissionState.NAVIGATING:
            self._run_policy()
        elif self.state == MissionState.HOVER:
            self._publish_cmd(0.0, 0.0, 0.0)
            if (self.get_clock().now() - self.state_entry_time).nanoseconds * 1e-9 >= self.hover_s:
                self._enter(MissionState.LANDING)
        elif self.state == MissionState.LANDING:
            self._publish_cmd(0.0, 0.0, 0.0)
            if not self.land_sent:
                self._call_land()
                self.land_sent = True
            if self.current_pos[2] - self._origin()[2] < 0.20 or not self.current_state.armed:
                self._enter(MissionState.DONE)
        elif self.state == MissionState.FAILSAFE:
            self._publish_cmd(0.0, 0.0, 0.0)
            if self.auto_mission and not self.land_sent and self.land_cli.service_is_ready():
                self._call_land()
                self.land_sent = True

    def _deps_ready(self) -> bool:
        if not self.have_odom:
            return False
        if self.auto_mission:
            return (
                self.have_state
                and self.arming_cli.service_is_ready()
                and self.set_mode_cli.service_is_ready()
                and self.takeoff_cli.service_is_ready()
                and self.land_cli.service_is_ready()
            )
        return True if not self.require_guided else self.have_state

    def _run_policy(self) -> None:
        if self.require_guided and (not self.current_state.connected or self.current_state.mode != "GUIDED"):
            self._publish_cmd(0.0, 0.0, 0.0)
            self._publish_status("not_guided")
            return

        rel = self.current_pos - self._origin()
        goal_xy = np.array(SCENARIOS[self.scenario_name].goal_xy, dtype=np.float64)
        goal_dist = float(np.linalg.norm(goal_xy - rel[:2]))
        self.min_goal_dist = min(self.min_goal_dist, goal_dist)
        if goal_dist <= self.goal_radius:
            self.get_logger().info(
                f"L7 goal reached pos=({rel[0]:.2f},{rel[1]:.2f},{rel[2]:.2f}) "
                f"goal_dist={goal_dist:.2f} min_goal_dist={self.min_goal_dist:.2f} "
                f"inference_ms={self.inference_ms:.2f}"
            )
            self._enter(MissionState.HOVER)
            return

        self.env.pos = rel[:2].copy()
        self.env.vel = self.current_vel[:2].copy()
        self.env.last_shield_active = self.shield_active
        obs = self.env._make_obs()
        t0 = self.get_clock().now()
        with torch.no_grad():
            mean, _ = self.model(torch.as_tensor(obs[None, :], dtype=torch.float32))
        self.inference_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6
        action = np.clip(mean.numpy()[0], -1.0, 1.0)
        vx = float(action[0] * self.policy_speed_scale)
        vy = float(action[1] * self.policy_speed_scale)
        xy = math.hypot(vx, vy)
        if xy > self.max_xy_speed:
            vx *= self.max_xy_speed / xy
            vy *= self.max_xy_speed / xy
        target_z = self._origin()[2] + self.takeoff_alt
        vz = float(np.clip(self.z_gain * (target_z - self.current_pos[2]), -self.max_z_speed, self.max_z_speed))
        self._publish_cmd(vx, vy, vz)
        self._publish_status(
            f"nav goal_dist={goal_dist:.2f} cmd=({vx:.2f},{vy:.2f},{vz:.2f}) "
            f"inference_ms={self.inference_ms:.2f}"
        )

    def _publish_cmd(self, vx: float, vy: float, vz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.cmd_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = f"{self.state.value} {text}"
        self.status_pub.publish(msg)
        if (self.get_clock().now() - self.last_log_time).nanoseconds * 1e-9 >= 1.0:
            self.last_log_time = self.get_clock().now()
            self.get_logger().info(msg.data)

    def _capture_origin(self) -> None:
        if self.mission_origin is None and self.have_odom:
            self.mission_origin = self.current_pos.copy()
            self.get_logger().info(
                f"Captured mission_origin=({self.mission_origin[0]:.2f},"
                f"{self.mission_origin[1]:.2f},{self.mission_origin[2]:.2f})"
            )

    def _origin(self) -> np.ndarray:
        if self.mission_origin is None:
            self._capture_origin()
        return self.mission_origin if self.mission_origin is not None else np.zeros(3, dtype=np.float64)

    def _enter(self, next_state: MissionState) -> None:
        self.state = next_state
        self.state_entry_time = self.get_clock().now()
        self.last_log_time = self.state_entry_time - Duration(seconds=10.0)
        self.get_logger().info(f"State -> {next_state.value}")

    def _request_due(self) -> bool:
        return (self.get_clock().now() - self.last_request_time).nanoseconds * 1e-9 >= 2.0

    def _call_set_mode(self, mode: str) -> None:
        req = SetMode.Request()
        req.custom_mode = mode
        self.pending_set_mode = self.set_mode_cli.call_async(req)
        self.last_request_time = self.get_clock().now()
        self.get_logger().info(f"Requested mode: {mode}")

    def _call_arm(self, arm: bool) -> None:
        req = CommandBool.Request()
        req.value = arm
        self.pending_arm = self.arming_cli.call_async(req)
        self.last_request_time = self.get_clock().now()
        self.get_logger().info(f"Requested arming: {arm}")

    def _call_takeoff(self, altitude: float) -> None:
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        self.pending_takeoff = self.takeoff_cli.call_async(req)
        self.last_request_time = self.get_clock().now()
        self.get_logger().info(f"Takeoff command sent to {altitude:.2f} m")

    def _call_land(self) -> None:
        req = CommandTOL.Request()
        self.land_cli.call_async(req)
        self.last_request_time = self.get_clock().now()
        self.get_logger().info("Land command sent")

    def _handle_futures(self) -> None:
        if self.pending_set_mode is not None and self.pending_set_mode.done():
            self.get_logger().info(f"Set mode response: {self.pending_set_mode.result().mode_sent}")
            self.pending_set_mode = None
        if self.pending_arm is not None and self.pending_arm.done():
            self.get_logger().info(f"Arm response: {self.pending_arm.result().success}")
            self.pending_arm = None
        if self.pending_takeoff is not None and self.pending_takeoff.done():
            self.get_logger().info(f"Takeoff response: {self.pending_takeoff.result().success}")
            self.pending_takeoff = None

    def _log_waiting(self) -> None:
        if (self.get_clock().now() - self.last_log_time).nanoseconds * 1e-9 < 2.0:
            return
        self.last_log_time = self.get_clock().now()
        self.get_logger().info(
            "Waiting deps: "
            f"state={self.have_state} odom={self.have_odom} "
            f"arming={self.arming_cli.service_is_ready()} set_mode={self.set_mode_cli.service_is_ready()} "
            f"takeoff={self.takeoff_cli.service_is_ready()} land={self.land_cli.service_is_ready()}"
        )


def main() -> None:
    rclpy.init()
    node = L7PolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
