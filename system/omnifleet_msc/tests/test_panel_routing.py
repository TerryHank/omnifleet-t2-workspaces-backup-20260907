import math

import pytest
from pathlib import Path

from omnifleet_msc.panel_routing import (
    covariance_in_robot_map,
    local_goal_rejection,
    navigation_mode,
    pose_in_robot_map,
)
from omnifleet_msc.urdf_visuals import combine, fleet_root, load_visuals


def robot(robot_id, *, online=True, registered=True):
    return {'id': robot_id, 'online': online, 'registered': registered}


def local_row(robot_id='robot_113', **overrides):
    row = {'id': robot_id, 'online': True, 'registered': True, 'pose': [1., 2., 0.],
           'pose_age': .1, 'local_nav_ready': True, 'local_busy': False,
           'alignment': None, 'reason': ''}
    row.update(overrides)
    return row


def local_core(**overrides):
    state = {'online': True, 'nav_ready': True, 'control_gate_ready': True,
             'estop': False, 'manual_active': False, 'local_override': False,
             'goal_error': '', 'nav_active': False, 'pending_goal': False}
    state.update(overrides)
    return state


def test_navigation_mode_is_single_for_one_fresh_registered_vehicle():
    assert navigation_mode([robot('robot_113')]) == {
        'mode': 'SINGLE', 'robot_id': 'robot_113', 'online_count': 1,
        'reason': '单车 Nav2',
    }


def test_navigation_mode_is_fleet_for_two_fresh_registered_vehicles():
    result = navigation_mode([robot('robot_104'), robot('robot_113')])
    assert result['mode'] == 'FLEET'
    assert result['robot_id'] is None
    assert result['online_count'] == 2


@pytest.mark.parametrize('robots,fresh', [([], True), ([robot('robot_113')], False)])
def test_navigation_mode_waits_for_empty_or_stale_vehicle_state(robots, fresh):
    assert navigation_mode(robots, fresh)['mode'] == 'WAITING'


def test_unregistered_online_vehicle_blocks_automatic_control_mode():
    result = navigation_mode([robot('robot_113'), robot('robot_unknown', registered=False)])
    assert result['mode'] == 'WAITING'
    assert '未登记' in result['reason']


def test_reference_robot_pose_is_identity_in_fleet_map():
    assert pose_in_robot_map([1.2, -0.4, 0.7], None, 'robot_113', 'robot_113') == pytest.approx(
        [1.2, -0.4, 0.7]
    )


def test_peer_pose_is_transformed_from_fleet_map_to_robot_map():
    result = pose_in_robot_map([10.0, 0.0, math.pi / 2], [10.0, 0.0, math.pi / 2],
                               'robot_104', 'robot_113')
    assert result == pytest.approx([0.0, 0.0, 0.0])


def test_peer_pose_requires_verified_alignment():
    with pytest.raises(ValueError, match='共享地图对齐'):
        pose_in_robot_map([1.0, 2.0, 0.0], None, 'robot_104', 'robot_113')


def test_single_goal_requires_online_pose_local_gate_and_action_server():
    robot_state = local_row()
    core_state = local_core()
    assert local_goal_rejection(robot_state, core_state, True, 'robot_113') == ''
    assert 'ExecuteNavigation' in local_goal_rejection(robot_state, core_state, False, 'robot_113')
    assert '安全门' in local_goal_rejection(robot_state, local_core(control_gate_ready=False), True, 'robot_113')
    assert '安全门' in local_goal_rejection(robot_state, local_core(estop=True), True, 'robot_113')
    assert '任务' in local_goal_rejection(robot_state, core_state, True, 'robot_113', fleet_busy=True)


def test_single_goal_rejects_stale_pose_busy_nav_and_missing_peer_tf():
    assert '过期' in local_goal_rejection(local_row(pose_age=1.), local_core(), True, 'robot_113')
    assert '正在执行' in local_goal_rejection(local_row(local_busy=True), local_core(), True, 'robot_113')
    assert '正在执行' in local_goal_rejection(local_row(), local_core(nav_active=True), True, 'robot_113')
    assert '共享地图对齐' in local_goal_rejection(
        local_row('robot_104'), local_core(), True, 'robot_113')


def test_pose_covariance_rotates_xy_uncertainty_with_map_alignment():
    covariance = [0.0] * 36
    covariance[0] = 4.0
    covariance[7] = 1.0
    covariance[35] = 0.25
    result = covariance_in_robot_map(covariance, math.pi / 2)
    assert result[0] == pytest.approx(1.0)
    assert result[7] == pytest.approx(4.0)
    assert result[35] == pytest.approx(0.25)


def test_one_authoritative_t2_urdf_renders_independent_fleet_poses():
    root = Path(__file__).resolve().parents[3]
    visuals = load_visuals(root / 'vehicle/omnifleet_description/urdf/omnifleet_t2.urdf')
    assert visuals
    first = visuals[0]['transform']
    robot_113 = fleet_root({'pose': [1.0, 2.0, 0.0], 'local_pose_3d': None, 'alignment': [0, 0, 0]})
    robot_104 = fleet_root({'pose': [4.0, 5.0, 1.0], 'local_pose_3d': None, 'alignment': [0, 0, 0]})
    assert combine(robot_113, first)[0] != pytest.approx(combine(robot_104, first)[0])
