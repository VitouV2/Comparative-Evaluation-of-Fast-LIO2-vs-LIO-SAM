"""Bring up the SLAM module for one robot on the frozen team interface.

    ros2 launch agc_slam slam.launch.py robot_name:=ugv1 use_sim_time:=true

Publishes, under the robot's namespace:
    <robot>/odom       nav_msgs/Odometry
    <robot>/map_cloud  sensor_msgs/PointCloud2
    TF: map -> <robot>/odom -> <robot>/base_link

The LIO backend stays in its own private frames; only this module's output is
on the frozen interface. Pass backend:=point_lio or backend:=lio_sam to swap
the odometry source without changing anything a consumer sees.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# Per-backend wiring: where the odometry and cloud come from, and which
# calibrated sensor the backend's reported pose actually belongs to.
# Getting `body_sensor` wrong offsets every pose by a sensor mount height,
# which is the kind of error that silently poisons a merged map.
BACKENDS = {
    'fast_lio2': {
        'odom_topic': '/Odometry',
        'cloud_topic': '/cloud_registered',
        'body_sensor': 'imu',
    },
    'point_lio': {
        'odom_topic': '/aft_mapped_to_init',
        'cloud_topic': '/cloud_registered',
        'body_sensor': 'imu',
    },
    'lio_sam': {
        'odom_topic': '/lio_sam/mapping/odometry',
        'cloud_topic': '/lio_sam/mapping/cloud_registered',
        'body_sensor': 'lidar',
    },
    'kiss_icp': {
        'odom_topic': '/kiss/odometry',
        'cloud_topic': '/kiss/frame',
        'body_sensor': 'lidar',
    },
}


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot_name').perform(context)
    backend = LaunchConfiguration('backend').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    calibration_file = LaunchConfiguration('calibration_file').perform(context)
    config_file = LaunchConfiguration('backend_config').perform(context)
    share = get_package_share_directory('agc_slam')

    if backend not in BACKENDS:
        raise RuntimeError(
            f"unknown backend '{backend}'; choose one of {sorted(BACKENDS)}")
    wiring = BACKENDS[backend]

    if not calibration_file:
        calibration_file = os.path.join(
            share, 'config', 'calibration', f'{robot}_husky_a200.yaml')
    if not os.path.isfile(calibration_file):
        raise RuntimeError(
            f'calibration file not found: {calibration_file}\n'
            'Pass calibration_file:=<path>, or add one under '
            'config/calibration/ for this robot. Running without extrinsics '
            'would publish poses offset by the IMU mount height.')

    if not config_file:
        config_file = os.path.join(share, 'config', f'{robot}_{backend}.yaml')

    sim = use_sim_time.lower() in ('true', '1', 'yes')

    actions = [
        LogInfo(msg=[
            f'agc_slam: robot={robot} backend={backend} ',
            f'use_sim_time={sim}\n',
            f'  calibration: {calibration_file}\n',
            f'  backend odom:  {wiring["odom_topic"]}\n',
            f'  backend cloud: {wiring["cloud_topic"]}\n',
            f'  publishing:    /{robot}/odom, /{robot}/map_cloud',
        ]),
    ]

    # The backend node. Its own topics/frames stay private and un-namespaced,
    # exactly as upstream publishes them, so upstream needs no patch.
    if backend == 'fast_lio2':
        if not os.path.isfile(config_file):
            raise RuntimeError(f'backend config not found: {config_file}')
        actions.append(Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio_backend',
            parameters=[config_file, {'use_sim_time': sim}],
            output='screen',
        ))
    else:
        actions.append(LogInfo(msg=(
            f'agc_slam: backend "{backend}" is not launched by this file yet; '
            'start it yourself and this interface will adapt its output. '
            'Backends are wired in for the P2 benchmark.')))

    # The interface node, namespaced so its relative topics resolve to
    # /<robot>/odom and /<robot>/map_cloud.
    actions.append(GroupAction([
        PushRosNamespace(robot),
        Node(
            package='agc_slam',
            executable='slam_interface',
            name='slam_interface',
            output='screen',
            parameters=[{
                'use_sim_time': sim,
                'robot_name': robot,
                'backend': backend,
                'backend_odom_topic': wiring['odom_topic'],
                'backend_cloud_topic': wiring['cloud_topic'],
                'backend_body_sensor': wiring['body_sensor'],
                'calibration_file': calibration_file,
                'map_frame': LaunchConfiguration('map_frame'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'map_publish_period': LaunchConfiguration('map_publish_period'),
                'publish_identity_map_to_odom':
                    LaunchConfiguration('publish_identity_map_to_odom'),
            }],
        ),
    ]))

    return actions


def generate_launch_description():
    share = get_package_share_directory('agc_slam')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='ugv1',
            description='Robot namespace, e.g. ugv1 or uav1'),
        DeclareLaunchArgument(
            'backend', default_value='fast_lio2',
            description=f'LIO backend: one of {sorted(BACKENDS)}'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true in Gazebo and when replaying bags with --clock'),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description='Extrinsics YAML; defaults to the one for this robot'),
        DeclareLaunchArgument(
            'backend_config', default_value='',
            description='Backend params YAML; defaults to <robot>_<backend>.yaml'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Global frame the coordinator aligns robots into'),
        DeclareLaunchArgument(
            'voxel_size', default_value='0.2',
            description='map_cloud leaf size (m); raise it if Wi-Fi saturates'),
        DeclareLaunchArgument(
            'map_publish_period', default_value='1.0',
            description='Seconds between map_cloud publications'),
        DeclareLaunchArgument(
            'publish_identity_map_to_odom', default_value='true',
            description='Placeholder map->odom; false once the coordinator owns it'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz with the SLAM module view'),
        OpaqueFunction(function=_launch_setup),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(share, 'config', 'agc_slam.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
