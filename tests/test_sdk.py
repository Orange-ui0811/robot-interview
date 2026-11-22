from concurrent.futures import ThreadPoolExecutor
from hashlib import sha1
import math
from threading import enumerate as threads
from xml.etree import ElementTree as ET

import pytest

from go1_mock.model import Go1Model, JOINT_NAMES, LEGS, STAND
from go1_mock.sdk import CommandError, MockGo1SDK, ZERO


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


@pytest.fixture
def robot(urdf_path):
    clock = Clock()
    sdk = MockGo1SDK(Go1Model.load(urdf_path), clock=clock)
    sdk.connect()
    return sdk, clock


def test_pinned_urdf_and_static_tree(urdf_path):
    data = urdf_path.read_bytes()
    git_blob = b"blob " + str(len(data)).encode() + b"\0" + data
    assert sha1(git_blob).hexdigest() == "6de9c5c2e594d440dae7fbc823fb270743f51b9f"
    root = ET.fromstring(data)
    links = {link.attrib["name"] for link in root.findall("link")}
    children = {j.find("child").attrib["link"]: j.find("parent").attrib["link"]
                for j in root.findall("joint")}
    assert len(links) == 17
    assert len(children) == len(root.findall("joint")) == 16
    assert links - children.keys() == {"trunk"}
    assert not root.findall(".//mesh")
    for leg in LEGS:
        assert [children[f"{leg}_{segment}"] for segment in ("hip", "thigh", "calf", "foot")] == [
            "trunk", f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf",
        ]
    actuated = [j for j in root.findall("joint") if j.attrib["type"] == "revolute"]
    assert tuple(j.attrib["name"] for j in actuated) == JOINT_NAMES
    assert len(actuated) == 12
    assert [j.find("axis").attrib["xyz"] for j in actuated] == ["1 0 0", "0 1 0", "0 1 0"] * 4


def test_legal_initial_pose_and_mock_sensors(robot):
    sdk, _ = robot
    names, positions, velocities = sdk.get_joint_states()
    assert names == JOINT_NAMES and positions == STAND
    assert len(velocities) == 12 and not any(velocities)
    assert all(lo < q < hi for q, (lo, hi) in zip(positions, sdk.model.joint_limits))
    assert sdk.get_state().height == pytest.approx(0.331699, abs=1e-6)
    assert sdk.get_imu() == {
        "simulated": True, "orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
        "angular_velocity_rad_s": ZERO, "linear_acceleration_m_s2": None,
    }
    assert sdk.get_battery() == {"simulated": True, "percentage": 0.85}


def test_forward_three_seconds_and_stationary_after(robot):
    sdk, clock = robot
    token = sdk.reserve_action(3)
    assert sdk.begin_action(token, 0.2, 0.0, 0.0, goal_id="test-move-for-id")
    clock.now = 1.2
    assert sdk.get_state().task.elapsed_s == pytest.approx(1.2)
    clock.now = 3.04
    state = sdk.get_state()
    error = abs(state.x - 0.6)
    print(f"3 s scenario: x={state.x:.9f} m, y={state.y:.9f} m, error={error:.3e} m")
    assert error < 1e-10 and state.y == 0 and state.yaw == 0
    assert state.task.status == "succeeded" and state.task.elapsed_s == 3
    assert state.state_json()["task"]["goal_id"] == "test-move-for-id"
    assert state.velocity == ZERO
    clock.now = 4.04
    assert sdk.get_state().x == state.x


@pytest.mark.parametrize(("velocity", "pose"), [
    ((0.2, 0, 0), (0.08, 0, 0)),
    ((-0.2, 0, 0), (-0.08, 0, 0)),
    ((0, 0.2, 0), (0, 0.08, 0)),
    ((0, -0.2, 0), (0, -0.08, 0)),
    ((0, 0, 0.8), (0, 0, 0.32)),
])
def test_body_directions(robot, velocity, pose):
    sdk, clock = robot
    sdk.move(*velocity)
    clock.now = 0.4
    state = sdk.get_state()
    assert (state.x, state.y, state.yaw) == pytest.approx(pose)


def test_body_to_odom_and_exact_combined_motion(robot):
    sdk, clock = robot
    token = sdk.reserve_action(2)
    sdk.begin_action(token, 0, 0, math.pi / 4)
    clock.now = 2
    assert sdk.get_state().yaw == pytest.approx(math.pi / 2)
    sdk.move(0.2, 0.1, 0.5)
    clock.now = 2.4
    x = 0.2 * (math.cos(0.2) - 1) / 0.5 - 0.1 * math.sin(0.2) / 0.5
    y = 0.2 * math.sin(0.2) / 0.5 + 0.1 * (math.cos(0.2) - 1) / 0.5
    state = sdk.get_state()
    assert (state.x, state.y, state.yaw) == pytest.approx((x, y, math.pi / 2 + 0.2))


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_rejected(robot, bad):
    sdk, _ = robot
    with pytest.raises(CommandError, match="finite"):
        sdk.move(bad, 0, 0)
    with pytest.raises(CommandError, match="duration"):
        sdk.reserve_action(bad)
    assert sdk.get_state().velocity == ZERO


def test_limit_stale_order_and_watchdog(robot):
    sdk, clock = robot
    assert sdk.move(1e308, 1e308, 10) == pytest.approx((0.6 / math.sqrt(2), 0.6 / math.sqrt(2), 1.5))
    clock.now = 0.1
    sdk.move(0.2, 0, 0)
    with pytest.raises(CommandError, match="out-of-order"):
        sdk.move(0.1, 0, 0, command_age_s=0.2)
    with pytest.raises(CommandError, match="stale"):
        sdk.move(0.1, 0, 0, command_age_s=0.5)
    before_timeout = sdk.get_state()
    clock.now = 2
    after = sdk.get_state()
    assert after.x - before_timeout.x == pytest.approx(0.1 * math.cos(before_timeout.yaw))
    assert after.y - before_timeout.y == pytest.approx(0.1 * math.sin(before_timeout.yaw))
    assert after.velocity == ZERO and after.stop_reason == "watchdog timeout"
    clock.now = 3
    assert sdk.get_state().x == after.x


def test_freshness_consumes_remaining_lease(robot):
    sdk, clock = robot
    clock.now = 1
    sdk.move(0.2, 0, 0, command_age_s=0.4)
    clock.now = 2
    assert sdk.get_state().x == pytest.approx(0.02)


def test_stop_rejects_queued_velocity_after_hold_expires(robot):
    sdk, clock = robot
    clock.now = 1.0
    sdk.stop()
    clock.now = 1.3
    with pytest.raises(CommandError, match="predates stop hold"):
        sdk.move(0.2, 0, 0, command_age_s=0.2)
    assert sdk.get_state().velocity == ZERO
    assert sdk.move(0.2, 0, 0, command_age_s=0.02)[0] == pytest.approx(0.2)


def test_goal_ownership_conflict_cancel_and_pending_stop(robot):
    sdk, clock = robot
    token = sdk.reserve_action(2)
    with pytest.raises(CommandError, match="owns"):
        sdk.reserve_action(1)
    with pytest.raises(CommandError, match="owns"):
        sdk.move(0.1, 0, 0)
    sdk.begin_action(token, 0.2, 0, 0)
    clock.now = 0.25
    assert sdk.cancel(token)
    assert sdk.get_state().task.status == "canceled"
    assert sdk.get_state().x == pytest.approx(0.05)
    with pytest.raises(CommandError, match="hold"):
        sdk.move(0.2, 0, 0)
    clock.now = 0.5
    pending = sdk.reserve_action(3)
    sdk.stop("stop service")
    assert not sdk.begin_action(pending, 0.2, 0, 0)
    assert sdk.get_state().task.status == "aborted"
    assert sdk.get_state().stop_reason == "stop service"


def test_only_one_concurrent_action_can_reserve(robot):
    sdk, _ = robot

    def request(_):
        try:
            return sdk.reserve_action(1)
        except CommandError:
            return None

    with ThreadPoolExecutor(max_workers=8) as pool:
        assert sum(result is not None for result in pool.map(request, range(32))) == 1


def test_zero_twist_interrupts_action_and_disconnect_leaves_no_worker(robot):
    sdk, clock = robot
    before = set(threads())
    token = sdk.reserve_action(1)
    sdk.begin_action(token, 0.2, 0, 0)
    clock.now = 0.2
    sdk.move(0, 0, 0)
    frozen = sdk.get_state()
    assert frozen.task.status == "aborted" and frozen.velocity == ZERO
    sdk.disconnect()
    clock.now = 10
    state = sdk.get_state()
    assert not state.connected and state.x == frozen.x and set(threads()) == before
    with pytest.raises(CommandError, match="disconnected"):
        sdk.move(0.2, 0, 0)
    with pytest.raises(CommandError, match="disconnected"):
        sdk.reserve_action(1)


@pytest.mark.parametrize("velocity", [(0.55, 0.2, 0.8), (0, 0, 1.5), (-0.6, 0, 0), (0, -0.6, 0)])
def test_trot_continuous_and_within_limits(robot, velocity):
    sdk, clock = robot
    token = sdk.reserve_action(2)
    sdk.begin_action(token, *velocity)
    prior = STAND
    maximum = [0.0] * len(STAND)
    for step in range(1, 361):
        clock.now = step / 100
        _, angles, speeds = sdk.get_joint_states()
        for index, (q, (lo, hi), old, speed) in enumerate(zip(angles, sdk.model.joint_limits, prior, speeds)):
            assert lo <= q <= hi
            assert abs(q - old) < 0.17 and math.isfinite(speed)
            maximum[index] = max(maximum[index], abs(q - STAND[index]))
        prior = angles
    assert all(max(maximum[i:i + 3]) >= 0.08 for i in range(0, 12, 3))
    assert prior == STAND
