"""RViz teleop: camera (or fake hand) -> teleop_node -> /joint_states -> robot_state_publisher -> RViz.

  ros2 launch robot_bringup teleop.launch.py              # real webcam
  ros2 launch robot_bringup teleop.launch.py fake:=true   # synthetic hand, no camera
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    urdf_file = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'robot_hand.urdf')
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    fake = LaunchConfiguration('fake')

    return LaunchDescription([

        DeclareLaunchArgument('fake', default_value='false',
                              description='Use a synthetic hand instead of the webcam'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='robot_vision',
            executable='mediapipe_world_node',
            output='screen',
            condition=UnlessCondition(fake)
        ),

        Node(
            package='robot_vision',
            executable='fake_hand_node',
            output='screen',
            condition=IfCondition(fake)
        ),

        # robot_state_publisher listens on /joint_states, so publish the commands there
        Node(
            package='robot_teleop',
            executable='teleop_node',
            output='screen',
            parameters=[{'output_topic': '/joint_states'}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('robot_description'), 'rviz', 'robot_hand.rviz')]
        )

    ])
