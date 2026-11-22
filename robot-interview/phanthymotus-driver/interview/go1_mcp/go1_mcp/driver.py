"""Tool metadata and dispatch for the Phanthy Motus Go1 mock driver."""
from dataclasses import dataclass
import math
import re
from time import time_ns
from xml.etree import ElementTree as ET


class Unavailable(RuntimeError):
    """The ROS endpoint or its sample is unavailable."""


class ToolNotFound(ValueError):
    pass


TOPIC_PATHS = {
    "battery": ("state/battery", "data/json"),
    "imu": ("state/imu", "data/json"),
    "joints": ("state/joints", "sensor/skeleton"),
    "loco_state": ("state/loco", "data/json"),
}
MODEL_URI = "go1://model/urdf"


@dataclass(frozen=True)
class Config:
    host: str = "127.0.0.1"
    port: int = 15795
    driver_id: str = "go1-mock-driver"
    name: str = "Go1 Kinematic Simulator"
    namespace: str = "go1_mock"
    default_duration_s: float = 0.5
    max_duration_s: float = 30.0
    state_timeout_s: float = 1.0
    rpc_timeout_s: float = 2.0
    register_refresh_s: float = 30.0

    def __post_init__(self):
        if self.host != "127.0.0.1" or type(self.port) is not int or not 0 <= self.port <= 65535:
            raise ValueError("MCP HTTP requires an IPv4 loopback listener and a valid port")
        if not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", self.namespace):
            raise ValueError("namespace must be a single ROS name")
        durations = (
            self.default_duration_s, self.max_duration_s, self.state_timeout_s,
            self.rpc_timeout_s, self.register_refresh_s,
        )
        if any(type(value) not in (int, float) or not math.isfinite(value)
               or value <= 0 for value in durations):
            raise ValueError("MCP durations must be positive and finite")
        if self.default_duration_s > self.max_duration_s:
            raise ValueError("default duration exceeds maximum")


def now_ms():
    return time_ns() // 1_000_000


def topic_out(namespace, name):
    path, fmt = TOPIC_PATHS[name]
    return [{"topic": f"/{namespace}/{path}", "format": fmt}]


def moving_joint_names(urdf):
    return tuple(j.attrib["name"] for j in ET.fromstring(urdf).findall("joint")
                 if j.attrib["type"] != "fixed")


def skeleton_sample(names, positions, velocities, order, timestamp_ms, orientation_wxyz):
    if len(names) != len(order) or len(positions) != len(names) or set(names) != set(order):
        raise ValueError("JointState must contain all Go1 joints exactly once")
    incoming = {name: index for index, name in enumerate(names)}
    joints = []
    for index, name in enumerate(order):
        source = incoming[name]
        position = positions[source]
        if not math.isfinite(position):
            raise ValueError("JointState contains a non-finite position")
        velocity = velocities[source] if len(velocities) == len(names) else None
        joints.append({"idx": index, "name": name, "q": position, "q_rad": position,
                       "dq": velocity if velocity is not None and math.isfinite(velocity) else None,
                       "tau": None, "ddq_rad_s2": None, "mode": None, "temperature_c": None})
    return {"simulated": True, "timestamp_ms": timestamp_ms, "control_level": "KINEMATIC_SIM",
            "fresh": True, "available": True, "format": "sensor/skeleton",
            "joints": joints, "imu_quat": orientation_wxyz}


def number(args, name, default=0.0):
    value = args.get(name, default)
    try:
        if type(value) in (int, float) and math.isfinite(value):
            return float(value)
    except OverflowError:
        pass
    raise ValueError(f"{name} must be a finite number")


class SensorPlugin:
    def __init__(self, name, backend, namespace):
        self.name = name
        self._backend = backend
        self._outputs = topic_out(namespace, name)

    def get_tool(self):
        return {
            "name": self.name, "type": "sensor", "multiInstance": False,
            "description": f"Go1 simulated {self.name} state",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info", "read", "get"]},
            }},
            "topic_out": self._outputs,
        }

    def start(self):
        self._backend.set_stream(self.name, True)

    def stop(self):
        self._backend.set_stream(self.name, False)

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "running", "simulated": True, "topic_out": self._outputs}
        if action == "stop":
            self.stop()
            return {"state": "idle", "simulated": True, "topic_out": self._outputs}
        if action in ("info", "read", "get", self.name):
            try:
                data = self._backend.read(self.name)
            except Unavailable:
                if action != "info":
                    raise
                return {"state": "unavailable", "simulated": True, "topic_out": self._outputs}
            return {"state": "running", "data": data, "topic_out": self._outputs}
        raise ValueError(f"unsupported {self.name} action")


class ModelPlugin:
    name = "model"

    def __init__(self, urdf):
        self.urdf = urdf

    def get_tool(self):
        return {"name": self.name, "type": "resource", "description": "Simplified Unitree Go1 URDF",
                "inputSchema": {"type": "object", "properties": {}}, "topic_out": []}

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        return {"urdf": self.urdf, "simulated": True}


class LocoPlugin:
    name = "loco"

    def __init__(self, backend, config):
        self._backend = backend
        self._config = config

    def get_tool(self):
        return {
            "name": self.name, "type": "actuator", "multiInstance": False,
            "description": "Move or stop the Go1 kinematic simulator through ROS 2",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum": ["move", "stop", "info", "start"]},
                    "vx": {"type": "number", "description": "Forward velocity in m/s"},
                    "vy": {"type": "number", "description": "Leftward velocity in m/s"},
                    "vyaw": {"type": "number", "description": "Yaw rate in degrees/s"},
                    "duration": {"type": "number", "minimum": 0, "maximum": self._config.max_duration_s,
                                 "description": "Seconds, zero or omitted uses the default"},
                },
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "duration"], "description": "Start a timed move"},
                    "stop": {"params": [], "description": "Stop immediately"},
                    "info": {"params": [], "description": "Read motion and task state"},
                    "start": {"params": [], "description": "Check tool readiness"},
                },
            },
            "topic_out": [],
        }

    def start(self):
        pass

    def stop(self):
        pass  # Node shutdown owns simulation shutdown. This hook only releases plugin resources.

    def dispatch(self, action, args):
        result = {"ok": True, "card": self.name, "action": action, "simulated": True, "timestamp_ms": now_ms()}
        if action == "move":
            duration = number(args, "duration") or self._config.default_duration_s
            if not 0 < duration <= self._config.max_duration_s:
                raise ValueError("duration out of range")
            speed = (number(args, "vx"), number(args, "vy"), math.radians(number(args, "vyaw")))
            if any(abs(value) > 3.4028234e38 for value in (*speed, duration)):
                raise ValueError("command exceeds ROS float32 range")
            goal_id = self._backend.move(*speed, duration)
            result.update(state="accepted", goal_id=goal_id)
        elif action == "stop":
            self._backend.stop()
            result.update(applied={"stopped": True})
        elif action in ("info", "start"):
            try:
                data = self._backend.read("loco_state")
            except Unavailable:
                return {"ok": False, "state": "unavailable", "simulated": True}
            result.update(state="ready", data=data)
        else:
            raise ValueError("unsupported loco action")
        return result


class Driver:
    def __init__(self, backend, config, urdf):
        self.model = ModelPlugin(urdf)
        plugins = [LocoPlugin(backend, config), self.model]
        plugins.extend(SensorPlugin(name, backend, config.namespace) for name in TOPIC_PATHS)
        self._plugins = {plugin.name: plugin for plugin in plugins}

    def start(self):
        for plugin in self._plugins.values():
            plugin.start()

    def stop(self):
        for plugin in self._plugins.values():
            plugin.stop()

    def get_tool(self, name):
        return self._plugin(name).get_tool()

    def get_tools(self):
        return [plugin.get_tool() for plugin in self._plugins.values()]

    def _plugin(self, name):
        try:
            return self._plugins[name]
        except KeyError as error:
            raise ToolNotFound(f"Unknown tool: {name}") from error

    def dispatch(self, name, arguments):
        plugin = self._plugin(name)
        action = arguments.get("action", name)
        if not isinstance(action, str):
            raise ValueError("action must be a string")
        return plugin.dispatch(action, arguments)
