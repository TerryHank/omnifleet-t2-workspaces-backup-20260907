import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
MOLA_LAUNCH = PACKAGE / "launch" / "mola_no_wheel.launch.py"
NAV_LAUNCH = PACKAGE / "launch" / "nav2_direct.launch.py"
CALIBRATION = PACKAGE / "config" / "airy_imu_calibration.yaml"
SMOOTHER = PACKAGE / "config" / "smoother_lio_only.yaml"


def test_launch_files_are_valid_python():
    for path in (MOLA_LAUNCH, NAV_LAUNCH):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def test_profiles_support_pure_lio_and_rep105_modes():
    source = MOLA_LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    profiles = next(
        ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "PROFILES" for target in node.targets)
    )
    assert profiles == {"simple_direct", "simple_direct_observation", "simple_rep105", "smoother_direct"}
    assert '"publish_localization_following_rep105": "True"' in source
    assert '"publish_localization_following_rep105": "False"' in source
    assert '"mola_bridge_odometry_frame": frame("odom")' in source
    assert '"forward_ros_tf_odom_to_mola": "False"' in source
    assert '"odom_topic_name": "/odom"' in source
    assert "pure_lio_chassis_tf_lease" not in source
    assert "mola_initial_map_mm_file" in source
    assert "map_path" in source
    assert 'executable="t2_driver"' not in source
    assert 'chassis_core.launch.py' not in source
    assert 'DeclareLaunchArgument("start_chassis_core"' not in source
    assert 'SetRemap(src="/lidar_odometry/nav_grid_gridmap", dst="/map")' in source
    assert 'SetEnvironmentVariable("LIDAR_POSE_X", "-0.041")' in source
    assert 'SetEnvironmentVariable("LIDAR_POSE_Z", "0.150")' in source
    assert '"ignore_lidar_pose_from_tf": "True"' in source
    assert '"ignore_imu_pose_from_tf": "True"' in source
    assert "/home/iecme/workspace/src" not in source


def test_smoother_profile_has_official_anchor_and_planar_constraints():
    source = MOLA_LAUNCH.read_text(encoding="utf-8")
    assert 'SetEnvironmentVariable("MOLA_LINK_FIRST_POSE_SIGMA", "1e-6")' in source
    assert 'SetEnvironmentVariable("MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION", "true")' in source
    assert 'SetEnvironmentVariable("MOLA_STATE_ESTIMATOR_PUBLISH_RATE", "10")' in source
    assert 'SetEnvironmentVariable("MOLA_LOCALIZATION_PUBLISH_TF", "true")' in source
    assert 'SetEnvironmentVariable("MOLA_PUBLISH_MAP_TO_ODOM_TF", "true")' in source
    assert 'SetEnvironmentVariable("MOLA_STATE_ESTIMATOR_PUBLISH_TWIST", "true")' in source
    assert 'SetEnvironmentVariable("MOLA_MAP_TO_ODOM_FRAME", "wheel_odom")' in source
    assert 'SetEnvironmentVariable("MOLA_MAP_TO_ODOM_CHILD_FRAME", "odom")' in source
    assert '"use_state_estimator": "True"' not in source
    assert 'SetEnvironmentVariable("ODOM1_TOPIC", "/lidar_odometry/pose")' in source
    assert 'SetEnvironmentVariable("ODOM2_TOPIC", "/odom")' in source
    assert '"state_estimation/map_odom"' in source
    assert 'namespace="mola_smoother"' in source
    assert 'arguments=[str(smoother_system)]' in source
    smoother = yaml.safe_load(SMOOTHER.read_text(encoding="utf-8"))["params"]
    assert smoother["do_process_imu_labels_re"] == "^$"
    assert smoother["additional_isam2_update_steps"] == 1
    assert smoother["publish_map_to_odom_tf"] == "${MOLA_PUBLISH_MAP_TO_ODOM_TF|false}"
    assert smoother["map_to_odom_frame_name"] == "${MOLA_MAP_TO_ODOM_FRAME|}"
    assert smoother["map_to_odom_child_frame"] == "${MOLA_MAP_TO_ODOM_CHILD_FRAME|}"


def test_imu_calibration_is_explicit_and_finite():
    config = yaml.safe_load(CALIBRATION.read_text(encoding="utf-8"))
    params = config["robot_113"]["omnifleet_t2_experiment_imu_adapter"]["ros__parameters"]
    assert params["drop_non_increasing_timestamps"] is True
    assert len(params["angular_velocity_bias"]) == 3
    assert len(params["rotation_xyzw"]) == 4
    assert 9.7 < params["linear_acceleration_scale"] < 10.0


def test_nav2_reuses_the_production_official_stack_read_only():
    source = NAV_LAUNCH.read_text(encoding="utf-8")
    assert 'get_package_share_directory("omnifleet_planner")' in source
    assert 'DeclareLaunchArgument("transform_tolerance", default_value="0.8")' in source
    assert 'DeclareLaunchArgument("odom_topic", default_value="/lidar_odometry/pose")' in source
    assert '"odom_topic": LaunchConfiguration("odom_topic")' in source
    assert 'DeclareLaunchArgument("obstacle_topic", default_value="/lidar_odometry/nav_voxelmap_points")' in source
    for costmap in ("local_costmap", "global_costmap"):
        assert f'"{costmap}.{costmap}.ros__parameters.obstacle_layer.lidar.topic": LaunchConfiguration("obstacle_topic")' in source
        assert f'"{costmap}.{costmap}.ros__parameters.obstacle_layer.lidar_clearing.clearing": LaunchConfiguration("obstacle_clearing")' in source
    assert 'SetParameter(name="odom_topic", value=LaunchConfiguration("odom_topic"))' in source
    assert '"params_file": configured_params' in source
    assert "RewrittenYaml" in source
