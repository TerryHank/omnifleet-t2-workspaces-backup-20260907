"""ROS-independent task allocation and conflict state machine.

The ROS node is intentionally thin.  Keeping the state machine independent
allows all allocation, queue, handover and conflict rules to be tested without
starting a ROS graph or touching a vehicle.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot, isfinite
from typing import Iterable, Optional
import uuid


ACTIVE_STATES = {"DISPATCHING", "ACTIVE", "CANCEL_REQUESTED"}
TERMINAL_STATES = {"SUCCEEDED", "FAILED", "CANCELED"}


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = ""


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    waypoints: tuple[Waypoint, ...]
    candidate_robot_ids: tuple[str, ...] = ()
    pass_radius: float = 0.25
    priority: float = 0.0
    region_id: str = ""
    allow_reassignment: bool = True
    max_attempts: int = 3


@dataclass
class RobotRecord:
    robot_id: str
    online: bool = False
    ready: bool = False
    fleet_enabled: bool = False
    navigation_active: bool = False
    estop: bool = False
    last_heartbeat: Optional[float] = None
    pose_x: Optional[float] = None
    pose_y: Optional[float] = None
    pose_frame: str = ""
    current_task_id: str = ""
    status: str = ""
    message: str = ""
    retry_after: float = 0.0

    def heartbeat_age(self, now: float) -> float:
        return max(0.0, now - self.last_heartbeat) if self.last_heartbeat is not None else float("inf")

    def available(self, now: float, heartbeat_timeout: float, auto_enable_fleet: bool) -> bool:
        return (
            self.online
            and self.heartbeat_age(now) <= heartbeat_timeout
            and self.ready
            and not self.navigation_active
            and not self.estop
            and now >= self.retry_after
            and (self.fleet_enabled or auto_enable_fleet)
        )


@dataclass
class TaskRecord:
    spec: TaskSpec
    state: str = "QUEUED"
    assigned_robot_id: str = ""
    attempts: int = 0
    created_at: float = 0.0
    updated_at: float = 0.0
    message: str = "等待分配"
    code: str = "QUEUED"
    dispatch_token: str = ""
    retry_at: float = 0.0
    failed_robots: set[str] = field(default_factory=set)
    feedback: dict = field(default_factory=dict)

    @property
    def active(self) -> bool:
        return self.state in ACTIVE_STATES


class CoordinatorCore:
    """Deterministic task scheduler used by the ROS node and unit tests."""

    def __init__(
        self,
        robot_ids: Iterable[str],
        heartbeat_timeout: float = 2.5,
        conflict_radius: float = 1.0,
        auto_enable_fleet: bool = True,
    ) -> None:
        ids = tuple(dict.fromkeys(str(x).strip("/") for x in robot_ids if str(x).strip("/")))
        if not ids:
            raise ValueError("at least one robot is required")
        if heartbeat_timeout <= 0 or conflict_radius < 0:
            raise ValueError("invalid scheduler limits")
        self.robots = {robot_id: RobotRecord(robot_id) for robot_id in ids}
        self.tasks: dict[str, TaskRecord] = {}
        self.queue: list[str] = []
        self.heartbeat_timeout = float(heartbeat_timeout)
        self.conflict_radius = float(conflict_radius)
        self.auto_enable_fleet = bool(auto_enable_fleet)

    def register(self, robot_id: str, now: float) -> RobotRecord:
        robot_id = str(robot_id).strip("/")
        if not robot_id:
            raise ValueError("empty robot id")
        record = self.robots.setdefault(robot_id, RobotRecord(robot_id))
        record.online = True
        record.last_heartbeat = float(now)
        return record

    def heartbeat(
        self,
        robot_id: str,
        now: float,
        *,
        ready: bool,
        fleet_enabled: bool,
        navigation_active: bool,
        estop: bool = False,
        status: str = "",
        message: str = "",
    ) -> RobotRecord:
        record = self.register(robot_id, now)
        record.ready = bool(ready)
        record.fleet_enabled = bool(fleet_enabled)
        record.navigation_active = bool(navigation_active)
        record.estop = bool(estop)
        record.status = str(status)
        record.message = str(message)
        return record

    def pose(self, robot_id: str, x: float, y: float, frame_id: str) -> None:
        record = self.robots.setdefault(robot_id, RobotRecord(robot_id))
        if isfinite(x) and isfinite(y):
            record.pose_x = float(x)
            record.pose_y = float(y)
            record.pose_frame = str(frame_id)

    def submit(self, spec: TaskSpec, now: float) -> TaskRecord:
        self._validate_spec(spec)
        old = self.tasks.get(spec.task_id)
        if old and old.state not in TERMINAL_STATES:
            raise ValueError("DUPLICATE_TASK")
        record = TaskRecord(spec=spec, created_at=float(now), updated_at=float(now))
        self.tasks[spec.task_id] = record
        self.queue.append(spec.task_id)
        return record

    def cancel(self, task_id: str, now: float, reason: str = "调用方取消") -> TaskRecord:
        record = self._task(task_id)
        if record.state in TERMINAL_STATES:
            raise ValueError("TASK_ALREADY_FINISHED")
        record.updated_at = float(now)
        record.message = reason or "调用方取消"
        record.code = "CANCEL_REQUESTED"
        if record.active:
            record.state = "CANCEL_REQUESTED"
        else:
            record.state = "CANCELED"
            record.assigned_robot_id = ""
            self.queue = [item for item in self.queue if item != task_id]
        return record

    def assign_ready(self, now: float) -> list[TaskRecord]:
        assignments: list[TaskRecord] = []
        ordered = sorted(
            (self.tasks[task_id] for task_id in self.queue if task_id in self.tasks),
            key=lambda item: (-item.spec.priority, item.created_at, item.spec.task_id),
        )
        self.queue = []
        for record in ordered:
            if record.state != "QUEUED" or record.retry_at > now:
                self.queue.append(record.spec.task_id)
                continue
            robot_id = self.choose_robot(record.spec, now, record.failed_robots)
            if robot_id is None:
                self.queue.append(record.spec.task_id)
                continue
            self._assign(record, robot_id, now)
            assignments.append(record)
        return assignments

    def choose_robot(
        self,
        spec: TaskSpec,
        now: float,
        excluded: Iterable[str] = (),
    ) -> Optional[str]:
        excluded_set = set(excluded)
        candidates = spec.candidate_robot_ids or tuple(self.robots)
        scored: list[tuple[float, str]] = []
        for robot_id in candidates:
            record = self.robots.get(robot_id)
            if not record or robot_id in excluded_set:
                continue
            if not record.available(now, self.heartbeat_timeout, self.auto_enable_fleet):
                continue
            if self._conflicts(spec, robot_id):
                continue
            distance = 0.0
            first = spec.waypoints[0]
            if (
                record.pose_x is not None
                and record.pose_y is not None
                and (not first.frame_id or not record.pose_frame or first.frame_id == record.pose_frame)
            ):
                distance = hypot(first.x - record.pose_x, first.y - record.pose_y)
            load = sum(
                1
                for task in self.tasks.values()
                if task.assigned_robot_id == robot_id and task.active
            )
            scored.append((distance + load * 1000.0, robot_id))
        return min(scored)[1] if scored else None

    def mark_started(self, task_id: str, token: str, now: float) -> bool:
        record = self.tasks.get(task_id)
        if not record or record.dispatch_token != token or record.state != "DISPATCHING":
            return False
        record.state = "ACTIVE"
        record.updated_at = float(now)
        record.code = "ACTIVE"
        record.message = "车辆已接受任务"
        return True

    def feedback(self, task_id: str, token: str, now: float, **values) -> bool:
        record = self.tasks.get(task_id)
        if not record or record.dispatch_token != token or not record.active:
            return False
        record.feedback.update(values)
        record.updated_at = float(now)
        if values.get("message"):
            record.message = str(values["message"])
        return True

    def finish(
        self,
        task_id: str,
        token: str,
        now: float,
        *,
        success: bool,
        code: str,
        message: str,
        allow_reassign: bool = True,
    ) -> Optional[TaskRecord]:
        record = self.tasks.get(task_id)
        if not record or record.dispatch_token != token:
            return None
        record.updated_at = float(now)
        record.code = str(code)
        record.message = str(message)
        robot_id = record.assigned_robot_id
        if robot_id in self.robots and self.robots[robot_id].current_task_id == task_id:
            self.robots[robot_id].current_task_id = ""
        record.assigned_robot_id = ""
        if success:
            record.state = "SUCCEEDED"
            return record
        if record.state == "CANCEL_REQUESTED" or code in {"CANCELED", "CANCEL_REQUESTED"}:
            record.state = "CANCELED"
            return record
        record.failed_robots.add(robot_id)
        if (
            allow_reassign
            and record.spec.allow_reassignment
            and record.attempts < max(1, record.spec.max_attempts)
        ):
            record.state = "QUEUED"
            record.dispatch_token = uuid.uuid4().hex
            record.retry_at = float(now) + 0.2
            self.queue.append(task_id)
        else:
            record.state = "FAILED"
        return record

    def expire_heartbeats(self, now: float) -> list[tuple[str, str]]:
        """Mark lost robots offline and requeue their active tasks.

        Returns `(task_id, old_dispatch_token)` for the ROS node to cancel any
        still-live action handle. Late action results are ignored by token.
        """
        lost: list[tuple[str, str]] = []
        for robot in self.robots.values():
            if not robot.online or robot.heartbeat_age(now) <= self.heartbeat_timeout:
                continue
            robot.online = False
            robot.ready = False
            robot.fleet_enabled = False
            for record in self.tasks.values():
                if record.assigned_robot_id != robot.robot_id or not record.active:
                    continue
                token = record.dispatch_token
                lost.append((record.spec.task_id, token))
                record.updated_at = float(now)
                record.message = f"{robot.robot_id} 心跳超时，准备接管"
                record.code = "ROBOT_HEARTBEAT_TIMEOUT"
                record.failed_robots.add(robot.robot_id)
                if robot.current_task_id == record.spec.task_id:
                    robot.current_task_id = ""
                record.assigned_robot_id = ""
                if record.spec.allow_reassignment and record.attempts < record.spec.max_attempts:
                    record.state = "QUEUED"
                    record.dispatch_token = uuid.uuid4().hex
                    record.retry_at = float(now) + 0.2
                    self.queue.append(record.spec.task_id)
                else:
                    record.state = "FAILED"
        return lost

    def status(self, now: float) -> dict:
        return {
            "robots": [
                {
                    "robot_id": robot.robot_id,
                    "online": robot.online,
                    "ready": robot.ready,
                    "fleet_enabled": robot.fleet_enabled,
                    "navigation_active": robot.navigation_active,
                    "heartbeat_age_s": robot.heartbeat_age(now),
                    "task_id": robot.current_task_id,
                    "message": robot.message,
                }
                for robot in sorted(self.robots.values(), key=lambda item: item.robot_id)
            ],
            "tasks": [
                {
                    "task_id": task.spec.task_id,
                    "state": task.state,
                    "assigned_robot_id": task.assigned_robot_id,
                    "attempts": task.attempts,
                    "region_id": task.spec.region_id,
                    "code": task.code,
                    "message": task.message,
                    "feedback": dict(task.feedback),
                }
                for task in sorted(self.tasks.values(), key=lambda item: item.spec.task_id)
            ],
        }

    def _assign(self, record: TaskRecord, robot_id: str, now: float) -> None:
        record.state = "DISPATCHING"
        record.assigned_robot_id = robot_id
        record.attempts += 1
        record.dispatch_token = uuid.uuid4().hex
        record.updated_at = float(now)
        record.code = "DISPATCHING"
        record.message = f"已分配给 {robot_id}"
        self.robots[robot_id].current_task_id = record.spec.task_id

    def _task(self, task_id: str) -> TaskRecord:
        record = self.tasks.get(task_id)
        if not record:
            raise ValueError("TASK_NOT_FOUND")
        return record

    @staticmethod
    def _validate_spec(spec: TaskSpec) -> None:
        if not spec.task_id or len(spec.task_id) > 160:
            raise ValueError("INVALID_TASK_ID")
        if not spec.waypoints:
            raise ValueError("EMPTY_WAYPOINTS")
        if not 0.05 <= spec.pass_radius <= 0.5 or not isfinite(spec.pass_radius):
            raise ValueError("INVALID_PASS_RADIUS")
        if spec.max_attempts < 1:
            raise ValueError("INVALID_MAX_ATTEMPTS")
        for waypoint in spec.waypoints:
            if not all(isfinite(value) for value in (waypoint.x, waypoint.y, waypoint.yaw)):
                raise ValueError("INVALID_WAYPOINT")

    def _conflicts(self, spec: TaskSpec, robot_id: str) -> bool:
        for other in self.tasks.values():
            if not other.active or other.spec.task_id == spec.task_id:
                continue
            if other.assigned_robot_id == robot_id:
                return True
            if spec.region_id and spec.region_id == other.spec.region_id:
                return True
            if self._routes_conflict(spec, other.spec):
                return True
        return False

    def _routes_conflict(self, first: TaskSpec, second: TaskSpec) -> bool:
        if not first.waypoints or not second.waypoints:
            return False
        frames_a = {point.frame_id for point in first.waypoints if point.frame_id}
        frames_b = {point.frame_id for point in second.waypoints if point.frame_id}
        if frames_a and frames_b and frames_a.isdisjoint(frames_b):
            return False
        threshold = self.conflict_radius + first.pass_radius + second.pass_radius
        for a0, a1 in _segments(first.waypoints):
            for b0, b1 in _segments(second.waypoints):
                if _segment_distance(a0, a1, b0, b1) <= threshold:
                    return True
        return False


def _segments(points: tuple[Waypoint, ...]):
    if len(points) == 1:
        return ((points[0], points[0]),)
    return tuple(zip(points, points[1:]))


def _segment_distance(a0: Waypoint, a1: Waypoint, b0: Waypoint, b1: Waypoint) -> float:
    if _segments_intersect(a0, a1, b0, b1):
        return 0.0
    return min(
        _point_segment_distance(a0, b0, b1),
        _point_segment_distance(a1, b0, b1),
        _point_segment_distance(b0, a0, a1),
        _point_segment_distance(b1, a0, a1),
    )


def _point_segment_distance(point: Waypoint, start: Waypoint, end: Waypoint) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return hypot(point.x - start.x, point.y - start.y)
    t = max(0.0, min(1.0, ((point.x - start.x) * dx + (point.y - start.y) * dy) / length2))
    return hypot(point.x - (start.x + t * dx), point.y - (start.y + t * dy))


def _orientation(a: Waypoint, b: Waypoint, c: Waypoint) -> float:
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)


def _on_segment(a: Waypoint, b: Waypoint, c: Waypoint) -> bool:
    return (
        min(a.x, c.x) <= b.x <= max(a.x, c.x)
        and min(a.y, c.y) <= b.y <= max(a.y, c.y)
    )


def _segments_intersect(a0: Waypoint, a1: Waypoint, b0: Waypoint, b1: Waypoint) -> bool:
    eps = 1e-9
    o1 = _orientation(a0, a1, b0)
    o2 = _orientation(a0, a1, b1)
    o3 = _orientation(b0, b1, a0)
    o4 = _orientation(b0, b1, a1)
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)):
        return True
    return (
        abs(o1) <= eps and _on_segment(a0, b0, a1)
        or abs(o2) <= eps and _on_segment(a0, b1, a1)
        or abs(o3) <= eps and _on_segment(b0, a0, b1)
        or abs(o4) <= eps and _on_segment(b0, a1, b1)
    )
