"""Pure routing decisions for the unified Foxglove map interaction."""
import math

from .geometry import compose, inverse, pose


def navigation_mode(robots, fresh=True):
    """Return the view/dispatch mode from the current online robot snapshot."""
    if not fresh:
        return {'mode': 'WAITING', 'robot_id': None, 'online_count': 0,
                'reason': '车队状态已过期，禁止发送新目标'}
    online = [r for r in robots if r.get('online')]
    if not online:
        return {'mode': 'WAITING', 'robot_id': None, 'online_count': 0,
                'reason': '当前没有在线车辆'}
    if any(not r.get('registered', True) for r in online):
        return {'mode': 'WAITING', 'robot_id': None, 'online_count': len(online),
                'reason': '发现未登记车辆，暂不发送导航目标'}
    if len(online) == 1:
        return {'mode': 'SINGLE', 'robot_id': online[0]['id'], 'online_count': 1,
                'reason': '单车 Nav2'}
    return {'mode': 'FLEET', 'robot_id': None, 'online_count': len(online),
            'reason': f'车队协同：{len(online)} 台在线'}


def pose_in_robot_map(value, alignment, robot_id, reference_robot):
    """Convert a fleet_map [x,y,yaw] goal into one robot's local map frame."""
    value = pose(value)
    if alignment is None:
        if robot_id != reference_robot:
            raise ValueError(robot_id + ' 缺少共享地图对齐')
        alignment = [0.0, 0.0, 0.0]
    return compose(inverse(pose(alignment)), value)


def local_goal_rejection(robot, core_state, action_ready, reference_robot, fleet_busy=False):
    """Explain why a unified-map LOCAL goal cannot be admitted right now."""
    robot_id = robot['id']
    if not robot.get('registered', True):
        return '在线车辆尚未登记到车队'
    if not robot.get('online') or not core_state.get('online'):
        return robot_id + ' 不在线'
    if robot.get('pose') is None or robot.get('pose_age', 99) > .8:
        return robot_id + ' 位姿或地图对齐已过期'
    if not robot.get('local_nav_ready'):
        return robot.get('reason') or robot_id + ' 本地导航尚未就绪'
    if (not core_state.get('nav_ready') or not core_state.get('control_gate_ready') or
            core_state.get('estop') or core_state.get('manual_active') or core_state.get('local_override') or
            core_state.get('goal_error')):
        return robot_id + ' 本地导航安全门未就绪'
    if robot.get('local_busy') or core_state.get('nav_active') or core_state.get('pending_goal'):
        return robot_id + ' 正在执行本地导航目标'
    if fleet_busy:
        return '车队任务仍在执行或取消中，不能切换成单车目标'
    try:
        pose_in_robot_map([0., 0., 0.], robot.get('alignment'), robot_id, reference_robot)
    except ValueError as error:
        return str(error)
    if not action_ready:
        return robot_id + ' 的 ExecuteNavigation 接口不可用'
    return ''


def covariance_in_robot_map(covariance, alignment_yaw):
    """Rotate a ROS 6x6 pose covariance between two planar map frames."""
    if len(covariance) != 36 or not all(math.isfinite(float(v)) for v in covariance):
        raise ValueError('初始位姿协方差无效')
    c, s = math.cos(alignment_yaw), math.sin(alignment_yaw)
    transform = [[float(i == j) for j in range(6)] for i in range(6)]
    transform[0][0], transform[0][1] = c, s
    transform[1][0], transform[1][1] = -s, c
    matrix = [[float(covariance[6 * i + j]) for j in range(6)] for i in range(6)]
    left = [[sum(transform[i][k] * matrix[k][j] for k in range(6))
             for j in range(6)] for i in range(6)]
    result = [[sum(left[i][k] * transform[j][k] for k in range(6))
               for j in range(6)] for i in range(6)]
    return [result[i][j] for i in range(6) for j in range(6)]
