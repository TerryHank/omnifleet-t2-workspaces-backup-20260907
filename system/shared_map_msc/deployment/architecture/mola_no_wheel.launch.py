"""Cold-switch MOLA no-wheel experiments without modifying production files."""

from pathlib import Path
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


PROFILES = {"simple_direct", "simple_direct_observation", "simple_rep105", "smoother_direct"}


def _as_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _launch(context):
    for name in ("start_chassis", "start_chassis_core"):
        value = context.launch_configurations.get(name)
        if value is not None and _as_bool(value):
            raise RuntimeError(
                f"{name} is removed: the chassis is managed only by "
                "omnifleet-t2-chassis.service via systemctl"
            )
    profile = LaunchConfiguration("profile").perform(context).strip()
    if profile not in PROFILES:
        raise RuntimeError(
            f"unknown profile {profile!r}; expected one of {sorted(PROFILES)}"
        )
    mode_file = Path('/etc/omnifleet_t2/architecture.env')
    configured_mode = 'pure_mola'
    if mode_file.is_file():
        values={}
        for line in mode_file.read_text().splitlines():
            if '=' in line:
                key,value=line.split('=',1);values[key.strip()]=value.strip()
        configured_mode=values.get('OMNIFLEET_ARCHITECTURE',configured_mode)
    required_mode='rep105' if profile in {'simple_rep105','smoother_direct'} else 'pure_mola'
    if configured_mode != required_mode:
        raise RuntimeError(
            f"architecture mismatch: profile {profile!r} requires {required_mode!r}, "
            f"but /etc/omnifleet_t2/architecture.env selects {configured_mode!r}; "
            "run switch_architecture.sh before launching"
        )
    experiment_share = Path(
        get_package_share_directory("omnifleet_t2_mola_experiments")
    )
    localization_share = Path(get_package_share_directory("omnifleet_localization"))
    mola_share = Path(get_package_share_directory("mola_lidar_odometry"))
    smoother_share = Path(
        get_package_share_directory("mola_state_estimation_smoother")
    )
    upstream_launch = mola_share / "ros2-launchs" / "ros2-lidar-odometry.launch.py"
    smoother_system = (
        smoother_share / "mola-cli-launchs" / "state_estimator_ros2.yaml"
    )
    pipeline = localization_share / "config" / "mola" / "lidar3d-icp-nav2.yaml"
    calibration = experiment_share / "config" / "airy_imu_calibration.yaml"
    smoother_config = experiment_share / "config" / "smoother_lio_only.yaml"
    for path in (
        upstream_launch,
        smoother_system,
        pipeline,
        calibration,
        smoother_config,
    ):
        if not path.is_file():
            raise RuntimeError(f"missing experiment dependency: {path}")

    output_root = Path(
        LaunchConfiguration("map_dir").perform(context)
    ).expanduser().resolve() / "experiments" / profile
    output_root.mkdir(parents=True, exist_ok=True)

    map_path = LaunchConfiguration("map_path").perform(context).strip()
    if map_path and not Path(map_path).expanduser().is_file():
        raise RuntimeError(f"initial MOLA map does not exist: {map_path}")
    initial_pose = LaunchConfiguration('initial_pose').perform(context).strip()

    height_file = Path('/home/iecme/.local/share/omnifleet_t2/map-height.json')
    heights = json.loads(height_file.read_text()) if height_file.exists() else {'slice_z_min': -0.03, 'slice_z_max': 2.0}
    use_smoother = profile == "smoother_direct"
    actions = [
        SetEnvironmentVariable("MOLA_LOCALIZATION_PUBLISH_TF", "true"),
        SetEnvironmentVariable("MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS", "true"),
        SetEnvironmentVariable("MOLA_SAVE_MM", str(output_root / "local_map.mm")),
        SetEnvironmentVariable("MOLA_VERBOSITY_MOLA_LO", "WARN"),
        SetEnvironmentVariable("MOLA_LO_DEBUG_ICP_QUALITY", "false"),
        SetEnvironmentVariable("MOLA_ROS2_PUBLISH_IN_SIM_TIME", "true" if profile in {"simple_rep105", "simple_direct_observation"} else "false"),
        SetEnvironmentVariable("MOLA_ROS2_TRANSFORM_TOLERANCE", "0.0"),
        SetEnvironmentVariable("MOLA_OPTIMIZE_TWIST", "false"),
        SetEnvironmentVariable("MOLA_ICP_CLOUD_DECIMATION", "0.60"),
        SetEnvironmentVariable("MOLA_MAP_CLOUD_DECIMATION", "0.30"),
        SetEnvironmentVariable("MOLA_LOCAL_VOXELMAP_RESOLUTION", "0.60"),
        SetEnvironmentVariable("MOLA_TF_FOOTPRINT_LINK", ""),
        SetEnvironmentVariable("MOLA_NAV_GRID_RESOLUTION", "0.10"),
        SetEnvironmentVariable("MOLA_NAV_GRID_Z_MIN", str(heights['slice_z_min'])),
        SetEnvironmentVariable("MOLA_NAV_GRID_Z_MAX", str(heights['slice_z_max'])),
        SetEnvironmentVariable("MOLA_PUBLISH_LOCAL_MAP_UPDATES_EVERY_N", "2"),
        SetEnvironmentVariable("MOLA_ROS2_PUBLISH_MAPS_PERIOD", "0.5"),
        SetEnvironmentVariable("LIDAR_POSE_X", "-0.041"),
        SetEnvironmentVariable("LIDAR_POSE_Y", "0.0"),
        SetEnvironmentVariable("LIDAR_POSE_Z", "0.150"),
        SetEnvironmentVariable("LIDAR_POSE_YAW", "0.0"),
        SetEnvironmentVariable("LIDAR_POSE_PITCH", "0.0"),
        SetEnvironmentVariable("LIDAR_POSE_ROLL", "0.0"),
        SetEnvironmentVariable("IMU_POSE_X", "-0.041"),
        SetEnvironmentVariable("IMU_POSE_Y", "0.0"),
        SetEnvironmentVariable("IMU_POSE_Z", "0.150"),
        SetEnvironmentVariable("IMU_POSE_YAW", "0.0"),
        SetEnvironmentVariable("IMU_POSE_PITCH", "0.0"),
        SetEnvironmentVariable("IMU_POSE_ROLL", "0.0"),
    ]
    if map_path and not initial_pose:
        actions.append(LogInfo(msg='Saved map loaded in localization-only mode. Set initial pose via /initialpose or /relocalize_near_pose, then set MOLA active=true; Nav2 waits for localization.'))
    if use_smoother:
        actions.extend([
            SetEnvironmentVariable("MOLA_LINK_FIRST_POSE_SIGMA", "1e-6"),
            SetEnvironmentVariable("MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION", "true"),
            SetEnvironmentVariable("MOLA_INITIAL_TWIST_SIGMA_LIN", "0.05"),
            SetEnvironmentVariable("MOLA_INITIAL_TWIST_SIGMA_ANG", "0.05"),
            SetEnvironmentVariable("MOLA_STATE_ESTIMATOR_PUBLISH_RATE", "10"),
            SetEnvironmentVariable("MOLA_PUBLISH_MAP_TO_ODOM_TF", "true"),
            SetEnvironmentVariable("MOLA_MAP_TO_ODOM_FRAME", "wheel_odom"),
            SetEnvironmentVariable("MOLA_MAP_TO_ODOM_CHILD_FRAME", "odom"),
        ])

    imu_adapter = Node(
        package="omnifleet_localization",
        executable="airy_imu_adapter_node",
        name="omnifleet_t2_experiment_imu_adapter",
        output="screen",
        parameters=[str(calibration)],
    )
    actions.append(imu_adapter)

    mola_arguments = {
        "lidar_topic_name": "/rslidar_points",
        "lidar_topic_type": "PointCloud2",
        "imu_topic_name": "/omnifleet_t2/experiments/rslidar_imu_corrected",
        "lidar_qos_reliability": "best_effort",
        "lidar_qos_depth": "20",
        "imu_qos_reliability": "best_effort",
        "imu_qos_depth": "1000",
        "use_imu_for_lio": "True",
        "ignore_lidar_pose_from_tf": "True",
        "ignore_imu_pose_from_tf": "True",
        "mola_deskew_method": "MotionCompensationMethod::IMU",
        "mola_tf_base_link": "base_link",
        "mola_lo_reference_frame": "map",
        "mola_state_estimator_reference_frame": "map",
        "mola_bridge_odometry_frame": "odom",
        "mola_lo_pipeline": str(pipeline),
        "mola_initial_map_mm_file": map_path,
        # REP-105: MOLA emits map -> odom; the chassis owns odom -> base_link.
        "publish_localization_following_rep105": "True",
        "forward_ros_tf_odom_to_mola": "False",
        "use_state_estimator": "False",
        "enforce_planar_motion": "False",
        "localization_publish_tf_source": "lidar_odometry",
        "localization_publish_odom_source": "lidar_odometry",
        "start_active": "True" if initial_pose or not map_path else "False",
        "start_mapping_enabled": "False" if map_path else "True",
        "use_mola_gui": "False",
        "use_rviz": "False",
        "use_diagnostic_aggregator": "False",
        "use_sim_time": "False",
    }
    if initial_pose:
        mola_arguments['initial_pose']=initial_pose
        mola_arguments['initial_localization_method']='InitLocalization::FixedPose'
    if use_smoother:
        # The LIO process publishes pose messages only. A separate Smoother
        # fuses those poses with /odom and owns the sole map -> odom TF.
        mola_arguments.update({
            "publish_localization_following_rep105": "False",
        })
    elif profile in {"simple_direct", "simple_direct_observation"}:
        # Official MOLA direct mode: publish map -> base_link and do not
        # query an external wheel odom TF for REP-105 composition.
        mola_arguments.update({
            "publish_localization_following_rep105": "False",
            "forward_ros_tf_odom_to_mola": "False",
            "enforce_planar_motion": "True",
        })
    elif profile == "simple_rep105":
        # Official Simple Estimator REP-105 path: feed wheel odometry as a
        # direct nav_msgs/Odometry source while the chassis owns odom->base_link.
        mola_arguments.update({
            "forward_ros_tf_odom_to_mola": "False",
            "odom_topic_name": "/odom",
            "odom_sensor_label": "wheel_odom",
            "enforce_planar_motion": "True",
        })
    lio_group_actions = [
        # LIO is the only MOLA TF publisher and follows REP-105.
        SetEnvironmentVariable(
            "MOLA_LOCALIZATION_PUBLISH_TF",
            "false" if use_smoother else "true",
        ),
        SetRemap(src="/lidar_odometry/nav_grid_gridmap", dst="/map"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(upstream_launch)),
            launch_arguments=mola_arguments.items(),
        ),
    ]
    actions.append(
        GroupAction(scoped=True, actions=lio_group_actions)
    )
    if use_smoother:
        actions.append(
            GroupAction(scoped=True, actions=[
                SetEnvironmentVariable(
                    "MOLA_STATE_ESTIMATOR_YAML", str(smoother_config)
                ),
                SetEnvironmentVariable("ODOM1_TOPIC", "/lidar_odometry/pose"),
                SetEnvironmentVariable("ODOM1_LABEL", "lidar_odom"),
                SetEnvironmentVariable("ODOM2_TOPIC", "/odom"),
                SetEnvironmentVariable("ODOM2_LABEL", "wheel_odom"),
                SetEnvironmentVariable("ODOM3_TOPIC", ""),
                SetEnvironmentVariable("ODOM4_TOPIC", ""),
                SetEnvironmentVariable("IMU_TOPIC", ""),
                SetEnvironmentVariable("GNSS_TOPIC", ""),
                SetEnvironmentVariable("MOLA_WITH_GUI", "false"),
                SetEnvironmentVariable("MOLA_TF_MAP", "map"),
                SetEnvironmentVariable("MOLA_TF_BASE_LINK", "base_link"),
                SetEnvironmentVariable("MOLA_TF_FOOTPRINT_LINK", ""),
                SetEnvironmentVariable(
                    "MOLA_FORWARD_ROS_TF_ODOM_TO_MOLA", "false"
                ),
                SetEnvironmentVariable("MOLA_LOCALIZATION_PUBLISH_TF", "true"),
                SetEnvironmentVariable("MOLA_PUBLISH_TWIST_FROM_LATEST_ODOM", "true"),
                SetEnvironmentVariable("MOLA_LATEST_ODOM_TWIST_SOURCE", "wheel_odom"),
                SetEnvironmentVariable("MOLA_LATEST_ODOM_TWIST_MAX_AGE", "0.5"),
                SetEnvironmentVariable(
                    "MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS", "true"
                ),
                SetEnvironmentVariable(
                    "MOLA_LOCALIZATION_PUBLISH_TF_SOURCE",
                    "state_estimation/map_odom",
                ),
                SetEnvironmentVariable(
                    "MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE",
                    "state_estimation",
                ),
                Node(
                    package="mola_launcher",
                    executable="mola-cli",
                    namespace="mola_smoother",
                    name="mola_smoother_cli",
                    output="screen",
                    arguments=[str(smoother_system)],
                    remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static")],
                ),
            ])
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="simple_direct"),
        DeclareLaunchArgument('initial_pose',default_value='',description='[x,y,z,yaw_deg,pitch_deg,roll_deg] in the saved map; otherwise wait for initial localization'),
        DeclareLaunchArgument(
            "map_dir", default_value="/var/lib/omnifleet_t2/maps"
        ),
        DeclareLaunchArgument(
            "map_path",
            # Fresh mapping is the safe default. Pass an explicit .mm path
            # only when intentionally running localization from a saved map.
            default_value="",
        ),
        OpaqueFunction(function=_launch),
    ])
