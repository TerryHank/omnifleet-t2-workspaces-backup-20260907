from omnifleet_local_navigation.fleet_lease import command_is_current, status_is_fresh


def status(**overrides):
    result = {
        "coordinator_age": 0.2,
        "control_gate_ready": True,
        "local_ready": True,
        "allow_fleet_motion": True,
        "fleet_enabled": True,
        "fleet_hold": False,
        "estop": False,
        "manual_active": False,
        "local_override": False,
        "control_mode": "FLEET",
        "command_kind": "navigate",
        "command_task_id": "task-1",
        "control_epoch": "epoch-1",
        "applied_command_seq": 7,
    }
    result.update(overrides)
    return result


def test_lease_requires_fresh_agent_and_coordinator_status():
    assert status_is_fresh(status(), 0.5)
    assert status_is_fresh(status(coordinator_age=0.6), 0.6)
    assert not status_is_fresh(status(), 0.601)
    assert not status_is_fresh(status(coordinator_age=0.601), 0.1)


def test_lease_fails_closed_for_permissions_and_safety_gates():
    for change in (
        {"control_gate_ready": False},
        {"local_ready": False},
        {"allow_fleet_motion": False},
        {"fleet_enabled": False},
        {"fleet_hold": True},
        {"estop": True},
        {"manual_active": True},
        {"local_override": True},
    ):
        assert not status_is_fresh(status(**change), 0.1), change


def test_fleet_action_must_match_task_epoch_and_revision():
    current = status()
    assert command_is_current(current, "task-1", "epoch-1", 7)
    assert not command_is_current(current, "task-2", "epoch-1", 7)
    assert not command_is_current(current, "task-1", "epoch-2", 7)
    assert not command_is_current(current, "task-1", "epoch-1", 8)
    assert command_is_current(current, "task-1", "epoch-1", 6, allow_newer=True)
    assert not command_is_current(current, "task-2", "epoch-1", 6, allow_newer=True)
    assert not command_is_current(status(command_kind="hold"), "task-1", "epoch-1", 7)
