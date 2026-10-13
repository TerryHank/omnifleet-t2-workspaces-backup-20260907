"""ROS 2 Fleet Coordinator node.

The node owns only task-level coordination.  Vehicle safety and motion remain
inside each robot's existing local navigation and MSC safety chain.
"""

from __future__ import annotations

import copy
import json
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener

from omnifleet_fleet_interfaces.srv import CancelTask, SubmitTask
from omnifleet_navigation_interfaces.action import ExecuteNavigation
from omnifleet_navigation_interfaces.msg import NavigationPoint

from .core import CoordinatorCore, TaskSpec, Waypoint


class FleetCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("fleet_coordinator", namespace="/fleet")
        robot_ids = list(self.declare_parameter("robot_ids", ["robot_104", "robot_113"]).value)
        self.heartbeat_timeout = float(self.declare_parameter("heartbeat_timeout_sec", 2.5).value)
        self.conflict_radius = float(self.declare_parameter("conflict_radius_m", 1.0).value)
        self.dispatch_period = float(self.declare_parameter("dispatch_period_sec", 0.2).value)
        self.auto_enable_fleet = bool(self.declare_parameter("auto_enable_fleet", True).value)
        self.core = CoordinatorCore(
            robot_ids,
            heartbeat_timeout=self.heartbeat_timeout,
            conflict_radius=self.conflict_radius,
            auto_enable_fleet=self.auto_enable_fleet,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_clients = {
            robot_id: ActionClient(self, ExecuteNavigation, f"/{robot_id}/navigation/execute")
            for robot_id in robot_ids
        }
        self.fleet_enable_clients = {
            robot_id: self.create_client(SetBool, f"/{robot_id}/navigation/allow_fleet")
            for robot_id in robot_ids
        }
        for robot_id in robot_ids:
            self.create_subscription(
                String,
                f"/{robot_id}/navigation/status",
                lambda message, rid=robot_id: self._on_robot_status(rid, message),
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.create_subscription(
                Odometry,
                f"/{robot_id}/lidar_odometry/pose",
                lambda message, rid=robot_id: self._on_robot_pose(rid, message),
                qos_profile_sensor_data,
            )
        self.task_status_pub = self.create_publisher(String, "/fleet/task_status", 10)
        self.fleet_status_pub = self.create_publisher(String, "/fleet/status", 10)
        self.submit_srv = self.create_service(SubmitTask, "/fleet/submit_task", self._submit_task)
        self.cancel_srv = self.create_service(CancelTask, "/fleet/cancel_task", self._cancel_task)
        self.timer = self.create_timer(self.dispatch_period, self._tick)
        self.handles = {}
        self.pending_enable = set()
        self.get_logger().info(
            f"Fleet Coordinator ready for {', '.join(sorted(self.core.robots))}"
        )

    def _on_robot_status(self, robot_id: str, message: String) -> None:
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError):
            self.get_logger().warning(f"{robot_id} status is not valid JSON")
            return
        now = time.monotonic()
        safety_reason = str(data.get("safety_reason", ""))
        estop = bool(data.get("estop", False)) or "急停" in safety_reason
        self.core.heartbeat(
            robot_id,
            now,
            ready=bool(data.get("local_ready", False)),
            fleet_enabled=bool(data.get("fleet_enabled", False)),
            navigation_active=bool(data.get("navigation_active", False)),
            estop=estop,
            status=str(data.get("phase", data.get("control_mode", ""))),
            message=str(data.get("readiness_reason", data.get("message", ""))),
        )

    def _on_robot_pose(self, robot_id: str, message: Odometry) -> None:
        pose = message.pose.pose.position
        self.core.pose(robot_id, float(pose.x), float(pose.y), message.header.frame_id)

    def _submit_task(self, request: SubmitTask.Request, response: SubmitTask.Response):
        try:
            waypoints = tuple(
                Waypoint(
                    float(item.pose.position.x),
                    float(item.pose.position.y),
                    _yaw(item),
                    str(item.header.frame_id),
                )
                for item in request.waypoints
            )
            spec = TaskSpec(
                task_id=str(request.task_id),
                waypoints=waypoints,
                candidate_robot_ids=tuple(str(item).strip("/") for item in request.candidate_robot_ids if str(item).strip("/")),
                pass_radius=float(request.pass_radius or 0.25),
                priority=float(request.priority),
                region_id=str(request.region_id),
                allow_reassignment=bool(request.allow_reassignment),
            )
            record = self.core.submit(spec, time.monotonic())
            response.accepted = True
            response.code = "QUEUED"
            response.message = "任务已进入队列"
            response.assigned_robot_id = ""
            self.get_logger().info(f"accepted task {record.spec.task_id}")
        except ValueError as error:
            response.accepted = False
            response.code = str(error)
            response.message = str(error)
            response.assigned_robot_id = ""
        return response

    def _cancel_task(self, request: CancelTask.Request, response: CancelTask.Response):
        try:
            record = self.core.cancel(str(request.task_id), time.monotonic(), str(request.reason))
            response.accepted = True
            response.code = record.code
            response.message = record.message
            if record.spec.task_id in self.handles:
                self.handles[record.spec.task_id][0].cancel_goal_async()
        except ValueError as error:
            response.accepted = False
            response.code = str(error)
            response.message = str(error)
        return response

    def _tick(self) -> None:
        now = time.monotonic()
        for task_id, token in self.core.expire_heartbeats(now):
            handle_info = self.handles.get(task_id)
            if handle_info and handle_info[1] == token:
                handle_info[0].cancel_goal_async()
                self.handles.pop(task_id, None)
        for record in self.core.assign_ready(now):
            self._dispatch(record)
        self._publish_status(now)

    def _dispatch(self, record) -> None:
        robot_id = record.assigned_robot_id
        client = self.action_clients.get(robot_id)
        if not client:
            self.core.finish(record.spec.task_id, record.dispatch_token, time.monotonic(), success=False,
                              code="NO_ACTION_CLIENT", message=f"未配置 {robot_id} Action")
            return
        if not client.server_is_ready():
            self.core.finish(record.spec.task_id, record.dispatch_token, time.monotonic(), success=False,
                              code="NAV_ACTION_UNAVAILABLE", message=f"{robot_id} 导航 Action 不可用")
            return
        enable_client = self.fleet_enable_clients[robot_id]
        if self.auto_enable_fleet and not self.core.robots[robot_id].fleet_enabled:
            if robot_id in self.pending_enable or not enable_client.service_is_ready():
                self.core.finish(record.spec.task_id, record.dispatch_token, time.monotonic(), success=False,
                                  code="FLEET_ENABLE_UNAVAILABLE", message=f"{robot_id} 协同权限服务不可用")
                return
            self.pending_enable.add(robot_id)
            request = SetBool.Request()
            request.data = True
            future = enable_client.call_async(request)
            token = record.dispatch_token
            future.add_done_callback(lambda done, rec=record, rid=robot_id, t=token: self._fleet_enabled(done, rec, rid, t))
            return
        self._send_goal(record, robot_id)

    def _fleet_enabled(self, future, record, robot_id: str, token: str) -> None:
        self.pending_enable.discard(robot_id)
        if record.dispatch_token != token or record.state != "DISPATCHING":
            return
        try:
            reply = future.result()
        except Exception as error:
            self.core.finish(record.spec.task_id, token, time.monotonic(), success=False,
                              code="FLEET_ENABLE_ERROR", message=str(error))
            return
        if not reply.success:
            self.core.finish(record.spec.task_id, token, time.monotonic(), success=False,
                              code="FLEET_ENABLE_REJECTED", message=reply.message)
            return
        self.core.robots[robot_id].fleet_enabled = True
        self._send_goal(record, robot_id)

    def _send_goal(self, record, robot_id: str) -> None:
        token = record.dispatch_token
        try:
            waypoints = [self._to_robot_pose(item, robot_id) for item in record.spec.waypoints]
        except TransformException as error:
            self.core.finish(record.spec.task_id, token, time.monotonic(), success=False,
                              code="TF_UNAVAILABLE", message=f"无法转换到 {robot_id} map: {error}")
            return
        goal = ExecuteNavigation.Goal()
        goal.task_id = record.spec.task_id
        goal.command_epoch = uuid.uuid4().hex
        goal.revision = int(record.attempts)
        goal.source = ExecuteNavigation.Goal.FLEET
        goal.single_point = len(waypoints) == 1
        goal.pass_radius = float(record.spec.pass_radius)
        for index, pose in enumerate(waypoints):
            point = NavigationPoint()
            point.name = f"{record.spec.task_id}-{index + 1}"
            point.pose = pose
            point.kind = NavigationPoint.STOP if index == len(waypoints) - 1 else NavigationPoint.PASS
            point.dwell_seconds = 0.0
            goal.waypoints.append(point)
        client = self.action_clients[robot_id]
        future = client.send_goal_async(
            goal,
            feedback_callback=lambda feedback, rec=record, rid=robot_id, t=token: self._feedback(rec, rid, t, feedback),
        )
        future.add_done_callback(lambda done, rec=record, rid=robot_id, t=token: self._goal_response(done, rec, rid, t))

    def _goal_response(self, future, record, robot_id: str, token: str) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self.core.finish(record.spec.task_id, token, time.monotonic(), success=False,
                              code="ACTION_SEND_ERROR", message=str(error))
            return
        if not handle.accepted:
            self.core.finish(record.spec.task_id, token, time.monotonic(), success=False,
                              code="ACTION_REJECTED", message=f"{robot_id} 拒绝任务")
            return
        self.handles[record.spec.task_id] = (handle, token)
        self.core.mark_started(record.spec.task_id, token, time.monotonic())
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda done, rec=record, rid=robot_id, t=token: self._result(done, rec, rid, t))

    def _feedback(self, record, robot_id: str, token: str, message) -> None:
        feedback = message.feedback
        self.core.feedback(
            record.spec.task_id,
            token,
            time.monotonic(),
            phase=str(feedback.phase),
            message=str(feedback.message),
            tf_age_ms=float(feedback.tf_age_ms),
            robot_id=robot_id,
            passed_points=int(feedback.passed_points),
        )

    def _result(self, future, record, robot_id: str, token: str) -> None:
        self.handles.pop(record.spec.task_id, None)
        try:
            wrapped = future.result()
            result = wrapped.result
            success = bool(result.success)
            code = str(result.code or ("SUCCEEDED" if success else "NAVIGATION_FAILED"))
            message = str(result.message)
            if wrapped.status == GoalStatus.STATUS_CANCELED:
                success = False
                code = "CANCELED"
        except Exception as error:
            success = False
            code = "ACTION_RESULT_ERROR"
            message = str(error)
        self.core.finish(
            record.spec.task_id,
            token,
            time.monotonic(),
            success=success,
            code=code,
            message=message,
        )

    def _to_robot_pose(self, waypoint: Waypoint, robot_id: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = waypoint.frame_id or "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = __import__("math").sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = __import__("math").cos(waypoint.yaw / 2.0)
        target = f"{robot_id}/map"
        if pose.header.frame_id in ("map", target):
            if pose.header.frame_id == "map":
                pose.header.frame_id = target
            return pose
        transform = self.tf_buffer.lookup_transform(
            target, pose.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.2)
        )
        return do_transform_pose(pose, transform)

    def _publish_status(self, now: float) -> None:
        data = self.core.status(now)
        payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
        self.fleet_status_pub.publish(String(data=payload))
        for task in data["tasks"]:
            self.task_status_pub.publish(String(data=json.dumps(task, ensure_ascii=False, separators=(",", ":"))))

    def destroy_node(self):
        for handle, _token in self.handles.values():
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        self.handles.clear()
        return super().destroy_node()


def _yaw(pose: PoseStamped) -> float:
    import math

    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
