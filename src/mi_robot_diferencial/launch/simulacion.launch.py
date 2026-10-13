"""
Lanza la pista en Gazebo, el robot, el bridge ROS 2 <-> Gazebo y RViz.

    ros2 launch mi_robot_diferencial simulacion.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('mi_robot_diferencial')
    xacro_file = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'pista.sdf')
    rviz_file = os.path.join(pkg, 'rviz', 'control.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    rviz = LaunchConfiguration('rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'mi_robot', '-topic', 'robot_description',
                   '-x', '-0.5', '-y', '-2.5', '-z', '0.08'], # <--- X en -0.5
        output='screen',
    )

    # ]  ROS -> Gazebo     [  Gazebo -> ROS
    bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/world/pista/model/mi_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                # --- NUEVOS SENSORES ---
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            remappings=[('/world/pista/model/mi_robot/joint_state', '/joint_states')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}], condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo, robot_state_publisher, spawn, bridge, rviz_node,
    ])
