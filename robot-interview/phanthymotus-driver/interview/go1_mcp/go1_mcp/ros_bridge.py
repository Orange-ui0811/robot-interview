"""The MCP process is a ROS 2 client. It does not create a MockGo1SDK."""
import argparse
from collections import Counter, OrderedDict
from dataclasses import replace
import json
import logging
from math import isfinite
import os
from pathlib import Path
import sys
from threading import Event, Lock
from time import monotonic

from ament_index_python.packages import get_package_share_directory
from go1_interfaces.action import MoveFor
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .driver import Config, Driver, TOPIC_PATHS, Unavailable, moving_joint_names, skeleton_sample, topic_out
from .http_server import Server

STREAM_PERIOD_S = 0.05


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class RosBridge(Node):
    def __init__(self, config, urdf):
        super().__init__("go1_mcp_bridge")
        self._config = config
        self._joint_order = moving_joint_names(urdf)
        self._lock = Lock()
        self._samples = {}
        self._orientations = OrderedDict()
        self._streaming = set()
        self._next_skeleton_at = float("-inf")
        self._skeleton_counts = Counter() if os.environ.get("GO1_TRACE_SKELETON") == "1" else None
        self._last_skeleton_rejection = None
        self._last_skeleton_stamp_ms = None
        self._stream_publishers = {
            name: self.create_publisher(String, topic_out(config.namespace, name)[0]["topic"], 10)
            for name in TOPIC_PATHS
        }
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Imu, "/imu", self._on_imu, 10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self.create_subscription(String, "/go1/state", self._on_state, 10)
        self._move_client = ActionClient(self, MoveFor, "/go1/move_for")
        self._stop_client = self.create_client(Trigger, "/go1/stop")
        self.create_timer(STREAM_PERIOD_S, self._publish_streams)
        if self._skeleton_counts is not None:
            self.get_logger().info(f"Tracing Go1 skeleton stream from {__file__}")
            self.create_timer(1.0, self._report_skeleton, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _report_skeleton(self):
        counts = self._skeleton_counts
        with self._lock:
            enabled = "joints" in self._streaming
        matched = self._stream_publishers["joints"].get_subscription_count()
        self.get_logger().info(
            f"Go1 skeleton trace: enabled={enabled} subscribers={matched} "
            f"joint_callbacks={counts['callbacks']} rejected={counts['rejected']} "
            f"prepared={counts['prepared']} published={counts['published']} "
            f"published_with_match={counts['matched']} last_stamp_ms={self._last_skeleton_stamp_ms} "
            f"last_rejection={self._last_skeleton_rejection!r}"
        )

    def set_stream(self, name, enabled):
        with self._lock:
            if enabled:
                self._streaming.add(name)
            else:
                self._streaming.discard(name)

    def _save(self, name, data):
        with self._lock:
            self._samples[name] = (monotonic(), data)

    def _on_joints(self, message):
        if self._skeleton_counts is not None:
            self._skeleton_counts["callbacks"] += 1
        now = monotonic()
        with self._lock:
            publish_skeleton = "joints" in self._streaming and now >= self._next_skeleton_at
        # The previous frame gives its matching IMU callback time to update the cache.
        if publish_skeleton and self._publish_stream("joints"):
            self._next_skeleton_at = now + STREAM_PERIOD_S
            self.get_logger().info("Published first Go1 skeleton sample from /joint_states", once=True)
        self._save("joints", message)

    def _on_imu(self, message):
        q = message.orientation
        omega = message.angular_velocity
        if not all(isfinite(value) for value in (q.w, q.x, q.y, q.z, omega.x, omega.y, omega.z)):
            return
        stamp = stamp_ns(message.header)
        with self._lock:
            self._orientations[stamp] = [q.w, q.x, q.y, q.z]
            if len(self._orientations) > 20:
                self._orientations.popitem(last=False)
        self._save("imu", {
            "simulated": True, "fresh": True, "available": True, "timestamp_ms": stamp // 1_000_000,
            "quaternion_wxyz": [q.w, q.x, q.y, q.z],
            "angular_velocity_rad_s": [omega.x, omega.y, omega.z], "linear_acceleration_m_s2": None,
        })

    def _on_battery(self, message):
        if message.location != "simulated" or not isfinite(message.percentage):
            return
        self._save("battery", {
            "simulated": True, "fresh": True, "available": message.present,
            "timestamp_ms": stamp_ns(message.header) // 1_000_000,
            "percentage": message.percentage, "soc_pct": 100.0 * message.percentage,
        })

    def _on_state(self, message):
        try:
            state = json.loads(message.data)
            if state.get("simulated") is True:
                state.update(available=True, fresh=True, timestamp_ms=state["stamp_ns"] // 1_000_000)
                self._save("loco_state", state)
        except (ValueError, AttributeError, KeyError, TypeError):
            self.get_logger().warning("Invalid simulator state message", throttle_duration_sec=1.0)

    def read(self, name):
        with self._lock:
            received, result = self._samples.get(name, (None, None))
            if received is None or monotonic() - received > self._config.state_timeout_s:
                raise Unavailable(f"no fresh ROS 2 {name} sample")
            if name != "joints":
                return result
            stamp = stamp_ns(result.header)
            orientation = self._orientations.get(stamp)
        try:
            return skeleton_sample(result.name, result.position, result.velocity, self._joint_order,
                                   stamp // 1_000_000, orientation)
        except ValueError as error:
            raise Unavailable(str(error)) from error

    def _publish_streams(self):
        with self._lock:
            streams = self._streaming - {"joints"}
        for name in streams:
            self._publish_stream(name)

    def _publish_stream(self, name):
        counts = self._skeleton_counts if name == "joints" else None
        try:
            data = self.read(name)
        except Unavailable as error:
            if counts is not None:
                counts["rejected"] += 1
                self._last_skeleton_rejection = str(error)
            return False
        message = String(data=json.dumps(data, allow_nan=False))
        publisher = self._stream_publishers[name]
        if counts is not None:
            counts["prepared"] += 1
            matched = publisher.get_subscription_count()
        publisher.publish(message)
        if counts is not None:
            counts["published"] += 1
            if matched:
                counts["matched"] += 1
            self._last_skeleton_stamp_ms = data["timestamp_ms"]
        return True

    def _await(self, future, operation, on_timeout=None):
        ready = Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(self._config.rpc_timeout_s):
            if on_timeout:
                future.add_done_callback(on_timeout)
            raise Unavailable(f"ROS 2 {operation} timed out")
        try:
            return future.result()
        except Exception as error:
            raise Unavailable(f"ROS 2 {operation} failed") from error

    def move(self, vx, vy, wz, duration_s):
        if not self._move_client.server_is_ready():
            raise Unavailable("ROS 2 MoveFor action server unavailable")
        goal = MoveFor.Goal(vx=vx, vy=vy, wz=wz, duration_s=duration_s)

        def cancel_late(future):
            try:
                late_goal = future.result()
                if late_goal.accepted:
                    late_goal.cancel_goal_async()
            except Exception:
                self.get_logger().warning("Could not cancel a late MoveFor goal response")

        handle = self._await(self._move_client.send_goal_async(goal), "goal acceptance", cancel_late)
        if not handle.accepted:
            raise Unavailable("MoveFor goal rejected (busy, stop hold, disconnected or invalid duration)")
        # The simulator enforces a finite lease even if this HTTP client disconnects.
        return bytes(handle.goal_id.uuid).hex()

    def stop(self):
        if not self._stop_client.service_is_ready():
            raise Unavailable("ROS 2 stop service unavailable")
        response = self._await(self._stop_client.call_async(Trigger.Request()), "stop")
        if not response.success:
            raise Unavailable(response.message)

    def close(self):
        self._move_client.destroy()
        self.destroy_node()


def main():
    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser(description="Phanthy Motus MCP bridge for the Go1 ROS 2 simulator")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--port", type=int)
    parser.add_argument("--core-url", default=os.environ.get("AGENT_CORE_URL", ""))
    parser.add_argument("--advertise-host", default="127.0.0.1")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    config = Config(**json.loads(args.config.read_text(encoding="utf-8"))) if args.config else Config()
    if args.port is not None:
        config = replace(config, port=args.port)
    urdf = (Path(get_package_share_directory("go1_description")) / "urdf/go1_model.urdf").read_text(
        encoding="utf-8"
    )
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = None
    server = None
    try:
        node = RosBridge(config, urdf)
        executor.add_node(node)
        core_url = "" if args.core_url == "disabled" else args.core_url
        server = Server(Driver(node, config, urdf), config, core_url, args.advertise_host)
        server.start()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if server is not None:
            server.stop()
        if node is not None:
            node.close()
        rclpy.try_shutdown()
