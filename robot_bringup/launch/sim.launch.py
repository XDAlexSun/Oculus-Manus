"""MuJoCo teleop: camera (or fake hand) -> teleop_node -> /hand/joint_command -> MuJoCo.

  ros2 launch robot_bringup sim.launch.py              # real webcam
  ros2 launch robot_bringup sim.launch.py fake:=true   # synthetic hand, no camera

On macOS, the MuJoCo viewer needs `mjpython`, so launch mujoco_hand_node by hand instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    fake = LaunchConfiguration('fake')

    return LaunchDescription([

        DeclareLaunchArgument('fake', default_value='false',
                              description='Use a synthetic hand instead of the webcam'),

        DeclareLaunchArgument('model_path', default_value='',
                              description='MJCF file (empty = packaged robot_sim/models/hand.xml)'),

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

        # default output topic is /hand/joint_command, which mujoco_hand_node subscribes to
        Node(
            package='robot_teleop',
            executable='teleop_node',
            output='screen'
        ),

        Node(
            package='robot_sim',
            executable='mujoco_hand_node',
            output='screen',
            parameters=[{'model_path': LaunchConfiguration('model_path')}]
        ),

    ])
