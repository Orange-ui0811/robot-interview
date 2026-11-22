"""ROS 2 messages and callbacks for the single MockGo1SDK."""
from collections import deque
from dataclasses import dataclass
import json
from math import isfinite
from pathlib import Path
from time import time_ns

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from go1_interfaces.action import MoveFor
from nav_msgs.msg import Odometry, Path as RosPath
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .model import Go1Model, JOINT_NAMES
from .sdk import ACTIVE, CommandError, Limits, MockGo1SDK, ZERO


@dataclass
class ActionRun:
    token: int
    handle: object
    done: Future
    reported_elapsed: float = -1.0


class Go1Node(Node):
    def __init__(self):
        super().__init__("go1_mock")
        model_path = Path(get_package_share_directory("go1_description")) / "urdf/go1_model.urdf"
        defaults = Limits()
        limits = Limits(**{
            name: self.declare_parameter(name, getattr(defaults, name)).value
            for name in ("max_linear_mps", "max_yaw_rad_s", "watchdog_s", "max_duration_s", "stop_hold_s")
        })
        self.sdk = MockGo1SDK(Go1Model.load(model_path), limits)
        publish_hz = self.declare_parameter("publish_hz", 50.0).value
        if not isfinite(publish_hz) or not 10 <= publish_hz <= 200:
            raise ValueError("publish_hz must be in [10, 200]")
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu", 10)
        self._battery_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self._state_pub = self.create_publisher(String, "/go1/state", 10)
        self._event_pub = self.create_publisher(String, "/go1/command_events", 10)
        self._path_pub = self.create_publisher(RosPath, "/go1/path", 1)
        self._tf = TransformBroadcaster(self)
        self._path = deque(maxlen=600)
        self._last_path_position = None

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Twist, "/go1/cmd_vel", self._on_twist, command_qos)
        self.create_subscription(TwistStamped, "/go1/cmd_vel_stamped", self._on_twist_stamped, command_qos)
        self.create_service(Trigger, "/go1/stop", self._on_stop)
        self.create_service(Trigger, "/go1/status", self._on_status)
        self._reservation = None
        self._run = None
        self._actions = ActionServer(
            self, MoveFor, "/go1/move_for", self._execute,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self._admit_goal, handle_accepted_callback=self._accepted, cancel_callback=self._cancel,
        )
        self.sdk.connect()
        self.create_timer(1 / publish_hz, self._tick, clock=self._steady)
        self.create_timer(0.2, self._publish_path, clock=self._steady)
        self.get_logger().info("Go1 kinematic simulation active. Topics carry simulated values.")

    @staticmethod
    def _twist_values(twist):
        return twist.linear.x, twist.linear.y, twist.angular.z

    def _command_event(self, source, **details):
        self._event_pub.publish(String(data=json.dumps({
            "simulated": True, "stamp_ns": self.get_clock().now().nanoseconds,
            "source": source, **details,
        })))

    def _apply_twist(self, twist, age, source):
        try:
            if not all(isfinite(v) for v in (twist.linear.z, twist.angular.x, twist.angular.y)):
                raise CommandError("non-finite Twist")
            speed = self.sdk.move(*self._twist_values(twist), command_age_s=age)
            self._command_event(source, velocity=speed)
        except CommandError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=1.0)

    def _on_twist(self, twist, info):
        sent_ns = info.get("source_timestamp", 0)
        # Some RMW implementations cannot provide the source timestamp.
        receive_ns = time_ns()
        age = max(0.0, (receive_ns - sent_ns) / 1e9) if sent_ns > 0 else 0.0
        if sent_ns > receive_ns + 100_000_000:
            age = float("inf")
        self._apply_twist(twist, age, "cmd_vel")

    def _on_twist_stamped(self, message):
        if message.header.frame_id not in ("", "trunk"):
            self.get_logger().warning("TwistStamped frame must be trunk", throttle_duration_sec=1.0)
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        self._apply_twist(message.twist, age if stamp_ns > 0 else float("inf"), "cmd_vel_stamped")

    def _on_stop(self, _request, response):
        self.sdk.stop("stop service")
        self._command_event("stop_service", velocity=ZERO)
        response.success = True
        response.message = "motion stopped"
        return response

    def _on_status(self, _request, response):
        response.success = True
        response.message = json.dumps(self.sdk.get_state().state_json())
        return response

    def _admit_goal(self, request):
        if self._reservation is not None or self._run is not None:
            return GoalResponse.REJECT
        try:
            self.sdk.validate_velocity(request.vx, request.vy, request.wz)
            self._reservation = self.sdk.reserve_action(request.duration_s)
        except CommandError as error:
            self.get_logger().warning(f"MoveFor goal rejected: {error}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accepted(self, handle):
        run = ActionRun(self._reservation, handle, Future())
        self._reservation = None
        self._run = run
        req = handle.request
        goal_id = bytes(handle.goal_id.uuid).hex()
        if self.sdk.begin_action(run.token, req.vx, req.vy, req.wz, goal_id=goal_id):
            self._command_event(
                "move_for", velocity=self.sdk.validate_velocity(req.vx, req.vy, req.wz),
                duration_s=req.duration_s, task=run.token, goal_id=goal_id,
            )
        handle.execute()

    async def _execute(self, handle):
        run = self._run
        task = await run.done
        if task.status == "succeeded":
            handle.succeed()
        elif task.status == "canceled":
            handle.canceled()
        else:
            handle.abort()
        self._run = None
        return MoveFor.Result(success=task.status == "succeeded", message=task.message)

    def _cancel(self, handle):
        run = self._run
        if run is None or run.handle is not handle or not self.sdk.cancel(run.token):
            return CancelResponse.REJECT
        self._command_event("move_for_cancel", velocity=ZERO, task=run.token)
        return CancelResponse.ACCEPT

    def _advance_action(self, task):
        run = self._run
        if run is None or task is None or task.token != run.token or run.done.done():
            return
        if task.status == "running" and task.elapsed_s - run.reported_elapsed >= 0.1:
            run.handle.publish_feedback(MoveFor.Feedback(elapsed_s=task.elapsed_s))
            run.reported_elapsed = task.elapsed_s
        if task.status not in ACTIVE:
            # The cancel service first accepts, then transitions the goal to CANCELING.
            if task.status == "canceled" and not run.handle.is_cancel_requested:
                return
            run.done.set_result(task)

    @staticmethod
    def _orientation(target, values):
        target.x, target.y, target.z, target.w = values

    def _tick(self):
        state = self.sdk.get_state()
        stamp = self.get_clock().now().to_msg()
        self._advance_action(state.task)

        joints = JointState()
        joints.header.stamp = stamp
        joints.header.frame_id = "trunk"
        joints.name = list(JOINT_NAMES)
        joints.position = list(state.joint_position)
        joints.velocity = list(state.joint_velocity)
        self._joint_pub.publish(joints)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "trunk"
        pose = odom.pose.pose
        pose.position.x, pose.position.y, pose.position.z = state.x, state.y, state.height
        self._orientation(pose.orientation, state.orientation_xyzw)
        velocity = odom.twist.twist
        velocity.linear.x, velocity.linear.y, velocity.angular.z = state.velocity
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = "trunk"
        transform.transform.translation.x = state.x
        transform.transform.translation.y = state.y
        transform.transform.translation.z = state.height
        transform.transform.rotation = pose.orientation
        self._tf.sendTransform(transform)

        imu = Imu()
        imu.header = joints.header
        imu.orientation = pose.orientation
        imu.angular_velocity.z = state.velocity[2]
        imu.linear_acceleration_covariance[0] = -1.0
        self._imu_pub.publish(imu)

        battery = BatteryState()
        battery.header = joints.header
        battery.present = True
        battery.percentage = state.battery_percentage
        battery.location = "simulated"
        for field in ("voltage", "temperature", "current", "charge", "capacity", "design_capacity"):
            setattr(battery, field, float("nan"))
        self._battery_pub.publish(battery)

        data = state.state_json()
        data["stamp_ns"] = stamp.sec * 1_000_000_000 + stamp.nanosec
        self._state_pub.publish(String(data=json.dumps(data)))

        position = (state.x, state.y)
        if self._last_path_position is None or (
            (position[0] - self._last_path_position[0]) ** 2
            + (position[1] - self._last_path_position[1]) ** 2 >= 0.03 ** 2
        ):
            self._last_path_position = position
            self._path.append(PoseStamped(header=odom.header, pose=pose))

    def _publish_path(self):
        path = RosPath()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = list(self._path)
        self._path_pub.publish(path)

    def close(self):
        self.sdk.disconnect()
        self._actions.destroy()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = Go1Node()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.close()
        rclpy.try_shutdown()
