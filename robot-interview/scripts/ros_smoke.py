"""Headless integration test of the installed ROS 2 packages and actual HTTP bridge."""
from collections import deque
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
from time import monotonic
from urllib.error import URLError
from urllib.request import ProxyHandler, Request, build_opener

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped
from go1_interfaces.action import MoveFor
from go1_mock.model import Go1Model, JOINT_NAMES, LEGS, STAND
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path as RosPath
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

HTTP = build_opener(ProxyHandler({}))


def check(condition, label):
    if not condition:
        raise AssertionError(label)


def probe_odometry(seconds):
    """Check that an independent ROS process can receive an odometry sample."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "/odom", "nav_msgs/msg/Odometry", "--once", "--field", "child_frame_id",
             "--qos-reliability", "reliable", "--qos-depth", "10"],
            capture_output=True, text=True, timeout=seconds,
        )
    except subprocess.TimeoutExpired:
        return False, "timed out"
    output = (result.stdout or result.stderr).strip()
    return result.returncode == 0 and "trunk" in result.stdout.splitlines(), (
        f"exit={result.returncode}, output={output!r}"
    )


def probe_skeleton(topic):
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", topic, "std_msgs/msg/String", "--once", "--field", "data", "--full-length",
             "--qos-reliability", "reliable", "--qos-depth", "10"],
            capture_output=True, text=True, timeout=5,
        )
    except subprocess.TimeoutExpired:
        return "timed out"
    if result.returncode == 0 and "sensor/skeleton" in result.stdout:
        return "received skeleton"
    output = (result.stdout or result.stderr).strip()
    return f"exit={result.returncode}, output={output[:300]!r}"


def mcp(url, method, params=None):
    body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}).encode()
    try:
        with HTTP.open(Request(url, data=body, headers={"Content-Type": "application/json"}), timeout=3) as res:
            response = json.load(res)
    except URLError:
        return None
    check("result" in response, response)
    return response["result"]


def mcp_tool(url, name, **args):
    result = mcp(url, "tools/call", {"name": name, "arguments": args})
    if result is None:
        return None
    check(not result.get("isError"), result)
    return json.loads(result["content"][0]["text"])


def integration(port, log_path):
    url = f"http://127.0.0.1:{port}/mcp"
    urdf_path = Path(get_package_share_directory("go1_description")) / "urdf/go1_model.urdf"
    model = Go1Model.load(urdf_path)
    node = rclpy.create_node("go1_smoke")
    executor = SingleThreadedExecutor()
    steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
    subscriptions = {}
    latest = {}
    parents = {}
    static_frames = set()
    transforms = {}
    telemetry = {
        "joints": (JointState, "/joint_states"),
        "odom": (Odometry, "/odom"),
        "imu": (Imu, "/imu"),
        "battery": (BatteryState, "/battery_state"),
        "state": (String, "/go1/state"),
        "path": (RosPath, "/go1/path"),
    }
    skeleton_topic = "/go1_mock/state/joints"
    expected = {"trunk": {"odom"}}
    for leg in LEGS:
        expected.update({f"{leg}_hip": {"trunk"}, f"{leg}_thigh": {f"{leg}_hip"},
                         f"{leg}_calf": {f"{leg}_thigh"}, f"{leg}_foot": {f"{leg}_calf"}})

    def done(future, label, seconds=5, details=None):
        executor.spin_until_future_complete(future, timeout_sec=seconds)
        if not future.done():
            suffix = f" ({details()})" if details is not None else ""
            raise AssertionError(f"timed out: {label}{suffix}")
        return future.result()

    def await_condition(predicate, label, seconds=5, details=None):
        if predicate():
            return
        ready = Future()

        def poll():
            if predicate():
                ready.set_result(True)

        timer = node.create_timer(0.01, poll, clock=steady_clock)
        try:
            done(ready, label, seconds, details)
        finally:
            node.destroy_timer(timer)

    def tf_cb(message, static):
        for transform in message.transforms:
            child = transform.child_frame_id
            parents.setdefault(child, set()).add(transform.header.frame_id)
            transforms[child] = transform
            if static:
                static_frames.add(child)

    def save(name, msg_type, topic):
        subscriptions[topic] = node.create_subscription(
            msg_type, topic, lambda msg: latest.__setitem__(name, msg), 10,
        )

    def publisher_counts(topic):
        get_matched = getattr(subscriptions[topic], "get_publisher_count", None)
        matched = get_matched() if get_matched is not None else None
        return node.count_publishers(topic), matched

    def skeleton_details():
        # These snapshots precede the CLI probe, which creates a second subscriber.
        with log_path.open(encoding="utf-8", errors="replace") as log:
            traces = deque((line.partition("Go1 skeleton trace: ")[2].strip()
                            for line in log if "Go1 skeleton trace: " in line), maxlen=2)
        return (
            f"buffered frames={len(skeleton_frames)}, discovered, matched publishers={publisher_counts(skeleton_topic)}, "
            f"bridge snapshots before CLI={list(traces)}, independent CLI={probe_skeleton(skeleton_topic)}"
        )

    def startup_details():
        missing = {
            topic: publisher_counts(topic)
            for name, (_, topic) in telemetry.items() if name not in latest
        }
        return (
            f"missing telemetry (discovered, matched publishers)={missing}; "
            f"/tf publishers={publisher_counts('/tf')}; /tf_static publishers={publisher_counts('/tf_static')}; "
            f"missing TF frames={sorted(expected.keys() - parents.keys())}; "
            f"extra TF frames={sorted(parents.keys() - expected.keys())}; "
            f"cmd_vel subscribers={cmd.get_subscription_count()}; "
            f"cmd_vel_stamped subscribers={stamped.get_subscription_count()}"
        )

    def state():
        return json.loads(latest["state"].data)

    def pose():
        pos = latest["odom"].pose.pose.position
        return pos.x, pos.y

    def is_zero():
        twist = latest["odom"].twist.twist
        return (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)

    def stable():
        await_condition(is_zero, "stopped odometry")
        first = pose()
        settled_at = monotonic() + 0.22
        await_condition(lambda: monotonic() >= settled_at, "stationary observation window")
        check(math.dist(first, pose()) < 1e-7, "translation while stopped")

    def task_status(expected):
        return (state()["task"] or {}).get("status") == expected

    try:
        first_odometry = Future()

        def on_odometry(message):
            latest["odom"] = message
            if not first_odometry.done():
                first_odometry.set_result(message)

        subscriptions["/odom"] = node.create_subscription(Odometry, "/odom", on_odometry, 10)
        executor.add_node(node)
        first = done(first_odometry, "smoke subscriber's first /odom sample", 10,
                     details=lambda: f"discovered, matched publishers={publisher_counts('/odom')}")
        check(first.child_frame_id == "trunk", "smoke odometry frame")
        print("Smoke subscriber received its first /odom sample.", flush=True)

        subscriptions["/tf"] = node.create_subscription(TFMessage, "/tf", lambda msg: tf_cb(msg, False), 100)
        subscriptions["/tf_static"] = node.create_subscription(
            TFMessage, "/tf_static", lambda msg: tf_cb(msg, True),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        for name, (msg_type, topic) in telemetry.items():
            if name != "odom":
                save(name, msg_type, topic)
        cmd = node.create_publisher(Twist, "/go1/cmd_vel", 1)
        stamped = node.create_publisher(TwistStamped, "/go1/cmd_vel_stamped", 1)
        await_condition(
            lambda: telemetry.keys() <= latest.keys() and expected.keys() <= parents.keys()
            and cmd.get_subscription_count() and stamped.get_subscription_count(),
            "ROS telemetry and TF samples", 10, details=startup_details,
        )
        actions = ActionClient(node, MoveFor, "/go1/move_for")
        stop = node.create_client(Trigger, "/go1/stop")
        status = node.create_client(Trigger, "/go1/status")
        check(actions.wait_for_server(timeout_sec=10), "MoveFor server missing")
        check(stop.wait_for_service(timeout_sec=10) and status.wait_for_service(timeout_sec=10), "Trigger missing")
        check(done(status.call_async(Trigger.Request()), "startup /go1/status reply").success,
              "startup status service failed")
        print("ROS startup: status service replied, telemetry and TF received.", flush=True)
        initialized = mcp(url, "initialize", {"protocolVersion": "2024-11-05"})
        check(initialized and initialized["protocolVersion"] == "2024-11-05", "MCP initialize")
        tools = mcp(url, "tools/list")
        check(tools and {entry["name"] for entry in tools["tools"]} == {
            "loco", "loco_state", "battery", "imu", "joints", "model",
        }, "MCP tool discovery")
        check(mcp_tool(url, "model")["urdf"] == urdf_path.read_text(encoding="utf-8"), "MCP model")
        check(mcp_tool(url, "joints", action="read")["data"]["simulated"], "MCP joints")
        check(mcp_tool(url, "imu", action="read")["data"]["available"], "MCP IMU")
        check(mcp_tool(url, "battery", action="read")["data"]["soc_pct"] > 0, "MCP battery")

        check(parents == expected and static_frames == {f"{leg}_foot" for leg in LEGS}, "TF tree")
        tf_publishers = {p.node_name for p in node.get_publishers_info_by_topic("/tf")}
        check(tf_publishers == {"go1_mock", "go1_state_publisher"}, f"unexpected TF publishers: {tf_publishers}")
        initial = latest["joints"]
        check(tuple(initial.name) == JOINT_NAMES and tuple(initial.position) == STAND, "static pose")
        check(pose() == (0.0, 0.0) and latest["odom"].child_frame_id == "trunk", "initial odometry")
        check(latest["battery"].location == "simulated" and latest["imu"].linear_acceleration_covariance[0] == -1,
              "simulated sensor contract")
        skeleton_frames = deque(maxlen=2)
        subscriptions[skeleton_topic] = node.create_subscription(
            String, skeleton_topic, lambda msg: skeleton_frames.append(json.loads(msg.data)), 10,
        )
        stream = mcp_tool(url, "joints", action="start")
        check(stream and stream["state"] == "running" and stream["topic_out"] == [
            {"topic": skeleton_topic, "format": "sensor/skeleton"},
        ], "MCP joints stream start")
        await_condition(
            lambda: len(skeleton_frames) == 2 and skeleton_frames[1]["timestamp_ms"] > skeleton_frames[0]["timestamp_ms"],
            "MCP skeleton topic frames",
            details=skeleton_details,
        )
        for frame in skeleton_frames:
            check(frame["format"] == "sensor/skeleton" and frame["simulated"] and
                  {joint["name"] for joint in frame["joints"]} == set(JOINT_NAMES), "skeleton topic payload")
        print("MCP skeleton topic: received consecutive Go1 joint frames.", flush=True)

        feedback = []
        goal = done(actions.send_goal_async(
            MoveFor.Goal(vx=0.2, duration_s=3.0), feedback_callback=lambda msg: feedback.append(msg.feedback.elapsed_s)
        ), "accept 3 second goal")
        check(goal.accepted, "3 second goal rejected")
        await_condition(lambda: pose()[0] > 0.04, "3 second goal updates Odometry")
        angles = latest["joints"].position
        check(any(abs(a - b) > 0.005 for a, b in zip(angles, STAND)), "leg animation")
        check(all(lo <= q <= hi for q, (lo, hi) in zip(angles, model.joint_limits)), "joint limits")
        # A conflicting topic and a second action must leave ownership with the first goal.
        conflict_twist = Twist()
        conflict_twist.linear.y = 0.3
        cmd.publish(conflict_twist)
        conflict = done(actions.send_goal_async(MoveFor.Goal(vx=0.1, duration_s=1.0)), "conflicting goal")
        check(not conflict.accepted, "competing Action accepted")
        result = done(goal.get_result_async(), "3 second action result")
        check(result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success, "MoveFor success")
        stable()
        x, y = pose()
        error = abs(x - 0.6)
        print(f"ROS 2 3 s scenario: x={x:.9f} m, y={y:.9f} m, error={error:.6f} m", flush=True)
        check(error < 0.02 and abs(y) < 0.01, "0.6 m displacement or action arbitration")
        translation = transforms["trunk"].transform.translation
        check(math.dist((translation.x, translation.y), (x, y)) < 0.01, "translation agrees with trunk TF")
        check(len(feedback) >= 5 and feedback == sorted(feedback), "monotonic Action feedback")
        check(len(latest["path"].poses) >= 2 and state()["task"]["status"] == "succeeded", "path and task status")
        battery = latest["battery"].percentage
        check(0.84 < battery < 0.86, "mock battery percentage")

        for bad_goal in (MoveFor.Goal(vx=float("nan"), duration_s=1.0),
                         MoveFor.Goal(vx=0.1, duration_s=-1.0)):
            check(not done(actions.send_goal_async(bad_goal), "invalid goal rejection").accepted,
                  "invalid Action accepted")
        for value in (float("nan"), float("inf")):
            msg = Twist()
            msg.linear.x = value
            cmd.publish(msg)
            stable()
        old_message = TwistStamped()
        old_message.header.frame_id = "trunk"
        old_message.header.stamp.sec = node.get_clock().now().to_msg().sec - 5
        old_message.twist.linear.x = 0.2
        stamped.publish(old_message)
        stable()

        start_x = pose()[0]
        forward = Twist()
        forward.linear.x = 0.2
        cmd.publish(forward)
        await_condition(lambda: pose()[0] > start_x + 0.02, "Twist moves robot")
        await_condition(lambda: state()["stop_reason"] == "watchdog timeout", "watchdog motion halt")
        stable()
        check(0.04 < pose()[0] - start_x < 0.12, "watchdog displacement")

        turn = TwistStamped()
        turn.header.frame_id = "trunk"
        turn.header.stamp = node.get_clock().now().to_msg()
        turn.twist.angular.z = 0.6
        stamped.publish(turn)
        await_condition(lambda: state()["pose"]["yaw_rad"] > 0.03, "fresh TwistStamped turns robot")
        check(done(stop.call_async(Trigger.Request()), "stop service").success, "service failed")
        stable()
        orientation = transforms["trunk"].transform.rotation
        check(abs(2 * math.atan2(orientation.z, orientation.w) - state()["pose"]["yaw_rad"]) < 0.01,
              "heading agrees with trunk TF")
        status_response = done(status.call_async(Trigger.Request()), "status service")
        check(json.loads(status_response.message)["stop_reason"] == "stop service", "stop reason")

        cancel_goal = done(actions.send_goal_async(MoveFor.Goal(vx=0.2, duration_s=4.0)), "cancel goal")
        check(cancel_goal.accepted, "cancel scenario rejected")
        await_condition(lambda: task_status("running"), "goal running before cancel")
        cancel = done(cancel_goal.cancel_goal_async(), "cancel response")
        check(bool(cancel.goals_canceling), "cancel rejected")
        result = done(cancel_goal.get_result_async(), "canceled result")
        check(result.status == GoalStatus.STATUS_CANCELED and not result.result.success, "cancel terminal status")
        stable()

        abort_goal = done(actions.send_goal_async(MoveFor.Goal(vx=0.2, duration_s=4.0)), "abort goal")
        check(abort_goal.accepted, "abort scenario rejected")
        await_condition(lambda: not is_zero(), "moving before external stop")
        check(done(stop.call_async(Trigger.Request()), "external stop").success, "external stop failed")
        result = done(abort_goal.get_result_async(), "aborted result")
        check(result.status == GoalStatus.STATUS_ABORTED and not result.result.success, "external stop result")
        stable()

        previous = pose()
        move = mcp_tool(url, "loco", action="move", vx=0.2, duration=3.0)
        check(move and move["state"] == "accepted", "MCP MoveFor acceptance")
        await_condition(lambda: task_status("running") and math.dist(previous, pose()) > 0.04,
                        "MCP changes the ROS pose")
        read = mcp_tool(url, "loco_state", action="read")["data"]
        check(read["owner"] == "action" and math.dist(pose(), (read["pose"]["x_m"], read["pose"]["y_m"])) < 0.12,
              "MCP state must reflect ROS pose")
        check(read["task"]["goal_id"] == move["goal_id"], "MCP status refers to the accepted MoveFor goal")
        check(mcp_tool(url, "loco", action="stop")["applied"]["stopped"], "MCP stop")
        await_condition(lambda: task_status("aborted"), "MCP stop updates ROS action status")
        stable()
        print("ROS 2 TF, messages, Topic, Trigger, Action and shared MCP motion passed.", flush=True)
    finally:
        executor.shutdown()
        node.destroy_node()


def main():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    log_dir = Path(__file__).resolve().parents[1] / "go1_ros2_ws" / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = Path(tempfile.mkdtemp(prefix="go1-ros-smoke-", dir=log_dir)) / "launch.log"
    print(f"Headless launch log: {log_path}", flush=True)
    try:
        run_launch(port, log_path)
    except Exception:
        if log_path.exists():
            print(f"Last launch log lines from {log_path}", file=sys.stderr)
            with log_path.open(encoding="utf-8", errors="replace") as log:
                sys.stderr.writelines(deque(log, maxlen=100))
        raise


def run_launch(port, log_path):
    with log_path.open("w", encoding="utf-8") as log:
        launch = subprocess.Popen(
            ["ros2", "launch", "--noninteractive", "go1_bringup", "go1_demo.launch.py", "rviz:=false",
             f"mcp_port:={port}", "core_url:=disabled"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            env={**os.environ, "GO1_TRACE_SKELETON": "1"},
        )
        try:
            odometry_ready, probe_details = probe_odometry(20)
            check(odometry_ready, f"independent /odom startup probe failed ({probe_details})")
            print("Simulator ready: independent ROS CLI received /odom.", flush=True)
            rclpy.init()
            integration(port, log_path)
            check(launch.poll() is None, "launch exited unexpectedly")
        finally:
            rclpy.try_shutdown()
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
                raise AssertionError("launch did not stop within 15 seconds")
    contents = log_path.read_text(encoding="utf-8", errors="replace")
    children = set(re.findall(r"process started with pid \[(\d+)\]", contents))
    clean_exits = set(re.findall(r"process has finished cleanly \[pid (\d+)\]", contents))
    check(len(children) >= 3, "expected simulator, state publisher and MCP process")
    check(not any(Path(f"/proc/{pid}").exists() for pid in children), "launch left child processes running")
    check(launch.returncode == 0, f"launch returned {launch.returncode}")
    check(children <= clean_exits, f"launch child processes did not exit cleanly: {sorted(children - clean_exits)}")
    check("escalating to" not in contents, "launch escalated shutdown beyond SIGINT")
    print("All launch child processes exited cleanly.", flush=True)


if __name__ == "__main__":
    main()
