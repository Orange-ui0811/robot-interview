"""Thread-safe planar kinematics with command leases and a single state owner."""
from dataclasses import asdict, dataclass, replace
import math
from threading import RLock
from time import monotonic
from typing import Callable

from .model import Go1Model, JOINT_NAMES, STAND

ZERO = (0.0, 0.0, 0.0)
ACTIVE = ("pending", "running")


class CommandError(ValueError):
    """A command violated a numeric, connection or ownership constraint."""


@dataclass(frozen=True)
class Limits:
    max_linear_mps: float = 0.6
    max_yaw_rad_s: float = 1.5
    watchdog_s: float = 0.5
    max_duration_s: float = 30.0
    stop_hold_s: float = 0.15

    def __post_init__(self):
        values = (self.max_linear_mps, self.max_yaw_rad_s, self.watchdog_s, self.max_duration_s)
        if any(not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError("motion limits, watchdog and max duration must be finite and positive")
        if not math.isfinite(self.stop_hold_s) or self.stop_hold_s < 0:
            raise ValueError("stop hold must be finite and nonnegative")


@dataclass(frozen=True)
class Task:
    token: int
    status: str
    elapsed_s: float
    duration_s: float
    message: str
    goal_id: str | None = None


@dataclass(frozen=True)
class Snapshot:
    connected: bool
    x: float
    y: float
    yaw: float
    height: float
    velocity: tuple[float, float, float]
    joint_position: tuple[float, ...]
    joint_velocity: tuple[float, ...]
    owner: str
    stop_reason: str
    command_age_s: float | None
    task: Task | None
    battery_percentage: float = 0.85

    @property
    def orientation_xyzw(self):
        return (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))

    def state_json(self):
        vx, vy, wz = self.velocity
        mode = "standing"
        if vx or vy:
            mode = "walking"
        elif wz:
            mode = "turning"
        return {
            "simulated": True, "connected": self.connected, "mode": mode,
            "pose": {"x_m": self.x, "y_m": self.y, "z_m": self.height, "yaw_rad": self.yaw},
            "velocity_body_mps": {"forward": vx, "lateral": vy}, "yaw_speed_rad_s": wz,
            "owner": self.owner, "stop_reason": self.stop_reason, "command_age_s": self.command_age_s,
            "task": asdict(self.task) if self.task else None,
        }


class MockGo1SDK:
    def __init__(self, model: Go1Model, limits: Limits = Limits(), clock: Callable[[], float] = monotonic):
        self.model = model
        self.limits = limits
        self._clock = clock
        self._lock = RLock()
        self._now = clock()
        self._connected = False
        self._x = self._y = self._yaw = 0.0
        self._velocity = ZERO
        self._joint_position = STAND
        self._joint_velocity = (0.0,) * len(STAND)
        self._phase = 0.0
        self._gait_drive = ZERO
        self._owner = "idle"
        self._deadline = self._last_command = None
        self._hold_until = self._now
        self._stop_reason = "disconnected"
        self._task = None
        self._next_token = 0
        self._task_start = None

    def connect(self):
        with self._lock:
            self._advance()
            if not self._connected:
                self._connected = True
                self._stop_reason = "ready"

    def disconnect(self):
        with self._lock:
            self.stop("disconnected")
            self._connected = False

    def validate_velocity(self, vx, vy, wz):
        if not all(math.isfinite(value) for value in (vx, vy, wz)):
            raise CommandError("velocity must be finite")
        scale = max(abs(vx), abs(vy), self.limits.max_linear_mps)
        norm = math.hypot(vx / scale, vy / scale)
        if norm > self.limits.max_linear_mps / scale:
            vx = vx / scale / norm * self.limits.max_linear_mps
            vy = vy / scale / norm * self.limits.max_linear_mps
        yaw_limit = self.limits.max_yaw_rad_s
        return (vx, vy, min(yaw_limit, max(-yaw_limit, wz)))

    def validate_duration(self, duration_s):
        if not math.isfinite(duration_s) or not 0 < duration_s <= self.limits.max_duration_s:
            raise CommandError(f"duration must be in (0, {self.limits.max_duration_s}] seconds")

    def _check_start(self):
        if not self._connected:
            raise CommandError("SDK disconnected")
        if self._task and self._task.status in ACTIVE:
            raise CommandError("MoveFor owns motion")
        if self._now < self._hold_until:
            raise CommandError("stop hold is active")

    def move(self, vx, vy, wz, *, command_age_s=0.0):
        velocity = self.validate_velocity(vx, vy, wz)
        if not math.isfinite(command_age_s) or not 0 <= command_age_s < self.limits.watchdog_s:
            raise CommandError("stale velocity command")
        with self._lock:
            self._advance()
            if not self._connected:
                raise CommandError("SDK disconnected")
            if velocity == ZERO:
                self.stop("zero velocity command")
                return velocity
            self._check_start()
            command_time = self._now - command_age_s
            if self._last_command is not None and command_time < self._last_command:
                raise CommandError("out-of-order velocity command")
            if command_time < self._hold_until:
                raise CommandError("velocity command predates stop hold")
            self._set_motion(velocity, "topic", command_time, command_time + self.limits.watchdog_s)
            return velocity

    def reserve_action(self, duration_s):
        self.validate_duration(duration_s)
        with self._lock:
            self._advance()
            self._check_start()
            self._next_token += 1
            self._halt("action reserved")
            self._owner = "action"
            self._task = Task(self._next_token, "pending", 0.0, duration_s, "accepted")
            return self._task.token

    def begin_action(self, token, vx, vy, wz, *, goal_id=None):
        velocity = self.validate_velocity(vx, vy, wz)
        with self._lock:
            self._advance()
            if not self._task or self._task.token != token or self._task.status != "pending":
                return False
            self._task_start = self._now
            self._task = replace(self._task, status="running", message="executing", goal_id=goal_id)
            self._set_motion(velocity, "action", self._now, self._now + self._task.duration_s)
            return True

    def cancel(self, token):
        with self._lock:
            self._advance()
            if not self._task or self._task.token != token or self._task.status not in ACTIVE:
                return False
            self._end_action("canceled", "action canceled")
            self._halt("action canceled", hold=True)
            return True

    def stop(self, reason="stop requested"):
        with self._lock:
            self._advance()
            if self._task and self._task.status in ACTIVE:
                self._end_action("aborted", reason)
            self._halt(reason, hold=True)

    def _set_motion(self, velocity, owner, command_time, deadline):
        self._velocity = velocity
        self._owner = owner
        self._deadline = deadline
        self._last_command = command_time
        self._stop_reason = ""

    def _halt(self, reason, *, hold=False):
        self._velocity = ZERO
        self._owner = "idle"
        self._deadline = None
        self._stop_reason = reason
        if hold:
            self._hold_until = self._now + self.limits.stop_hold_s

    def _end_action(self, status, message):
        self._task = replace(self._task, status=status, message=message)
        self._task_start = None

    def _advance(self):
        now = max(self._now, self._clock())
        # Split the integration at the lease expiry, even when callbacks run late.
        if self._deadline is not None and self._deadline <= now:
            self._step(self._deadline)
            if self._owner == "action":
                self._task = replace(self._task, elapsed_s=self._task.duration_s)
                self._end_action("succeeded", "duration completed")
                self._halt("duration completed")
            else:
                self._halt("watchdog timeout")
        self._step(now)

    def _step(self, end):
        dt = end - self._now
        if dt <= 0:
            return
        self._now = end
        vx, vy, wz = self._velocity
        turn = wz * dt
        half_turn = turn / 2
        scale = dt if abs(half_turn) < 1e-8 else dt * math.sin(half_turn) / half_turn
        heading = self._yaw + half_turn
        self._x += scale * (vx * math.cos(heading) - vy * math.sin(heading))
        self._y += scale * (vx * math.sin(heading) + vy * math.cos(heading))
        self._yaw = math.remainder(self._yaw + turn, math.tau)
        if self._task_start is not None:
            self._task = replace(self._task, elapsed_s=min(end - self._task_start, self._task.duration_s))
        self._animate(dt)

    def _animate(self, dt):
        target = (
            self._velocity[0] / self.limits.max_linear_mps,
            self._velocity[1] / self.limits.max_linear_mps,
            self._velocity[2] / self.limits.max_yaw_rad_s,
        )
        alpha = -math.expm1(-dt / 0.11)
        drive = tuple(old + alpha * (new - old) for old, new in zip(self._gait_drive, target))
        self._gait_drive = drive if target != ZERO or max(map(abs, drive)) > 1e-4 else ZERO
        forward, lateral, yaw = self._gait_drive
        intensity = min(1.0, math.hypot(forward, lateral) + 0.6 * abs(yaw))
        self._phase = (self._phase + math.tau * (1.15 + 1.8 * intensity) * dt) % math.tau
        positions = []
        for index, offset in enumerate((0.0, math.pi, math.pi, 0.0)):
            phase = self._phase + offset
            side = -1 if index % 2 == 0 else 1
            end = 1 if index < 2 else -1
            swing = math.sin(phase)
            lift = intensity * max(0.0, math.cos(phase)) ** 2
            positions.extend((
                0.18 * (lateral + end * 0.45 * yaw) * swing,
                0.75 + 0.34 * (forward - side * 0.55 * yaw) * swing + 0.12 * lift,
                -1.50 - 0.34 * lift,
            ))
        positions = tuple(min(hi, max(lo, q)) for q, (lo, hi) in zip(positions, self.model.joint_limits))
        self._joint_velocity = tuple((q - old) / dt for q, old in zip(positions, self._joint_position))
        self._joint_position = positions

    def get_state(self):
        with self._lock:
            self._advance()
            age = None if self._last_command is None else self._now - self._last_command
            return Snapshot(
                self._connected, self._x, self._y, self._yaw, self.model.standing_height,
                self._velocity, self._joint_position, self._joint_velocity, self._owner,
                self._stop_reason, age, self._task,
            )

    def get_joint_states(self):
        state = self.get_state()
        return JOINT_NAMES, state.joint_position, state.joint_velocity

    def get_imu(self):
        state = self.get_state()
        return {
            "simulated": True, "orientation_xyzw": state.orientation_xyzw,
            "angular_velocity_rad_s": (0.0, 0.0, state.velocity[2]), "linear_acceleration_m_s2": None,
        }

    def get_battery(self):
        return {"simulated": True, "percentage": self.get_state().battery_percentage}
