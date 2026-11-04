import re
import subprocess
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    warthog_ns = 'w200_0001'
    husky_ns = 'a200_0001'

    # Warthog URDF
    warthog_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('warthog_description'),
                'urdf',
                'warthog.urdf.xacro'
            ]),
        ]),
        value_type=str
    )

    # Husky URDF - use Clearpath generated URDF with proper sensors
    husky_urdf_raw = subprocess.check_output([
        'xacro',
        '/home/v2/clearpath_a200/robot.urdf.xacro',
        'is_sim:=true'
    ]).decode('utf-8')
    # Strip ros2_control hardware plugin to avoid crash
    husky_urdf = re.sub(
        r'<ros2_control.*?</ros2_control>', '', husky_urdf_raw, flags=re.DOTALL)
    # ros2_control was the Husky's only drive, so replace it with Gazebo's
    # DiffDrive system (skid-steer: both wheels per side driven together).
    # Geometry from the Clearpath URDF: wheels at y=+-0.2775, radius 0.1651.
    husky_diff_drive_plugin = f'''
  <gazebo>
    <plugin filename="ignition-gazebo-diff-drive-system"
            name="ignition::gazebo::systems::DiffDrive">
      <left_joint>front_left_wheel_joint</left_joint>
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <wheel_separation>0.555</wheel_separation>
      <wheel_radius>0.1651</wheel_radius>
      <topic>/{husky_ns}/cmd_vel</topic>
      <odom_topic>/{husky_ns}/platform/odom</odom_topic>
      <tf_topic>/{husky_ns}/platform/odom/tf</tf_topic>
      <frame_id>{husky_ns}/odom</frame_id>
      <child_frame_id>{husky_ns}/base_link</child_frame_id>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>
  </gazebo>
</robot>'''
    husky_urdf = husky_urdf.replace('</robot>', husky_diff_drive_plugin, 1)

    # Gazebo warehouse world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r /opt/ros/humble/share/clearpath_gz/worlds/warehouse.sdf'
        }.items()
    )

    # Warthog RSP
    warthog_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=warthog_ns,
        parameters=[
            {'use_sim_time': True},
            {'robot_description': warthog_description},
            {'frame_prefix': warthog_ns + '/'}
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen'
    )

    # Spawn warthog
    spawn_warthog = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', warthog_ns,
            '-topic', '/' + warthog_ns + '/robot_description',
            '-x', '-5.0', '-y', '0.0', '-z', '0.1',
        ],
        output='screen'
    )

    # Husky RSP
    husky_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=husky_ns,
        parameters=[
            {'use_sim_time': True},
            {'robot_description': husky_urdf},
            {'frame_prefix': husky_ns + '/'}
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen'
    )

    # Spawn husky
    spawn_husky = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', husky_ns,
            '-topic', '/' + husky_ns + '/robot_description',
            '-x', '5.0', '-y', '0.0', '-z', '0.1',
        ],
        output='screen'
    )

    # Diff drive for husky
    husky_diff_drive = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='husky_diff_drive_bridge',
        arguments=[
            '/' + husky_ns + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/' + husky_ns + '/platform/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Warthog
            '/' + warthog_ns + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/' + warthog_ns + '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/' + warthog_ns + '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Husky sensors: the Clearpath URDF publishes them un-namespaced
            # in Gazebo, so bridge those names and remap onto the namespace.
            '/sensors/lidar3d_0/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/sensors/lidar3d_0/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/sensors/imu_0/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            ('/sensors/lidar3d_0/scan/points', '/' + husky_ns + '/sensors/lidar3d_0/points'),
            ('/sensors/lidar3d_0/scan', '/' + husky_ns + '/sensors/lidar3d_0/scan'),
            ('/sensors/imu_0/data', '/' + husky_ns + '/sensors/imu_0/data'),
        ],
        output='screen'
    )

    # Lidar time fixer relay
    lidar_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=[
            '/' + husky_ns + '/sensors/lidar3d_0/points',
            '/' + husky_ns + '/sensors/lidar3d_0/points_timed'
        ],
        output='screen'
    )


    # Global RSP for gz_ros2_control
    warthog_rsp_global = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': warthog_description},
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        warthog_rsp_global,
        warthog_rsp,
        spawn_warthog,
        husky_rsp,
        spawn_husky,
        bridge,
        husky_diff_drive,
        lidar_relay,
    ])
# This won't work - need to edit the file properly
