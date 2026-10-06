"""Validation for the coordinator agent's high-level navigation lease."""


def status_is_fresh(status, age, timeout=0.6):
    if not isinstance(status, dict) or age < 0 or age > timeout:
        return False
    try:
        coordinator_age = float(status.get("coordinator_age", timeout + 1))
    except (TypeError, ValueError):
        return False
    return (
        0 <= coordinator_age <= timeout
        and status.get("control_gate_ready") is True
        and status.get("local_ready") is True
        and status.get("allow_fleet_motion") is True
        and status.get("fleet_enabled") is True
        and status.get("fleet_hold") is False
        and not status.get("estop")
        and not status.get("manual_active")
        and not status.get("local_override")
    )


def command_is_current(status, task_id, command_epoch, revision, allow_newer=False):
    if not isinstance(status, dict):
        return False
    if status.get("control_mode") != "FLEET" or status.get("command_kind") != "navigate":
        return False
    if status.get("command_task_id") != task_id or status.get("control_epoch") != command_epoch:
        return False
    try:
        applied = int(status.get("applied_command_seq", -1))
    except (TypeError, ValueError):
        return False
    return applied >= revision if allow_newer else applied == revision
