import pytest

from omnifleet_fleet_coordinator.core import CoordinatorCore, TaskSpec, Waypoint


def point(x, y, frame="fleet_map"):
    return Waypoint(x=x, y=y, frame_id=frame)


def ready(core, robot_id, now, x=0.0, y=0.0):
    core.heartbeat(
        robot_id,
        now,
        ready=True,
        fleet_enabled=True,
        navigation_active=False,
    )
    core.pose(robot_id, x, y, "fleet_map")


def test_auto_selects_closest_available_robot():
    core = CoordinatorCore(["robot_104", "robot_113"])
    ready(core, "robot_104", 0.0, x=10.0)
    ready(core, "robot_113", 0.0, x=1.0)
    core.submit(TaskSpec("t1", (point(0.0, 0.0),)), 0.0)
    assigned = core.assign_ready(0.1)
    assert [item.assigned_robot_id for item in assigned] == ["robot_113"]


def test_same_region_is_queued_until_first_task_finishes():
    core = CoordinatorCore(["robot_104", "robot_113"])
    ready(core, "robot_104", 0.0)
    ready(core, "robot_113", 0.0, x=20.0)
    core.submit(TaskSpec("a", (point(0, 0),), region_id="zone-a"), 0.0)
    first = core.assign_ready(0.1)[0]
    core.submit(TaskSpec("b", (point(20, 0),), region_id="zone-a"), 0.2)
    assert core.assign_ready(0.3) == []
    core.finish("a", first.dispatch_token, 1.0, success=True, code="SUCCEEDED", message="done")
    assigned = core.assign_ready(1.1)
    assert [item.spec.task_id for item in assigned] == ["b"]


def test_failure_reassigns_to_other_robot():
    core = CoordinatorCore(["robot_104", "robot_113"])
    ready(core, "robot_104", 0.0)
    ready(core, "robot_113", 0.0, x=10.0)
    core.submit(TaskSpec("t", (point(1, 0),), max_attempts=2), 0.0)
    first = core.assign_ready(0.1)[0]
    old_robot = first.assigned_robot_id
    core.finish("t", first.dispatch_token, 0.2, success=False, code="NAVIGATION_FAILED", message="failed")
    assigned = core.assign_ready(0.5)
    assert len(assigned) == 1
    assert assigned[0].assigned_robot_id != old_robot
    assert assigned[0].attempts == 2


def test_heartbeat_loss_requeues_active_task_and_late_result_is_ignored():
    core = CoordinatorCore(["robot_104", "robot_113"], heartbeat_timeout=1.0)
    ready(core, "robot_104", 0.0)
    ready(core, "robot_113", 0.0, x=10.0)
    core.submit(TaskSpec("t", (point(1, 0),), max_attempts=2), 0.0)
    first = core.assign_ready(0.1)[0]
    first_token = first.dispatch_token
    core.mark_started("t", first_token, 0.2)
    lost = core.expire_heartbeats(1.2)
    assert lost == [("t", first_token)]
    assert core.tasks["t"].state == "QUEUED"
    assert core.finish("t", first_token, 1.3, success=True, code="SUCCEEDED", message="late") is None
    ready(core, "robot_113", 1.3, x=10.0)
    assigned = core.assign_ready(1.5)
    assert assigned[0].assigned_robot_id == "robot_113"


def test_cancel_removes_queued_task():
    core = CoordinatorCore(["robot_104"])
    ready(core, "robot_104", 0.0)
    core.submit(TaskSpec("t", (point(1, 0),)), 0.0)
    record = core.cancel("t", 0.1, "user")
    assert record.state == "CANCELED"
    assert core.assign_ready(0.2) == []


def test_same_frame_route_conflict_is_detected():
    core = CoordinatorCore(["robot_104", "robot_113"], conflict_radius=0.5)
    ready(core, "robot_104", 0.0)
    ready(core, "robot_113", 0.0, x=10.0)
    core.submit(TaskSpec("a", (point(0, 0), point(10, 0))), 0.0)
    first = core.assign_ready(0.1)[0]
    core.submit(TaskSpec("b", (point(5, -0.1), point(5, 0.1))), 0.2)
    assert core.assign_ready(0.3) == []
    assert first.assigned_robot_id == "robot_104"


def test_invalid_task_is_rejected():
    core = CoordinatorCore(["robot_104"])
    with pytest.raises(ValueError, match="EMPTY_WAYPOINTS"):
        core.submit(TaskSpec("empty", ()), 0.0)
