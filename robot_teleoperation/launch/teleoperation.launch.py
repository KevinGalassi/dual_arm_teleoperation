from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
def generate_launch_description():


    config = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'teleoperation_config.yaml'
    )




    left_teleoperation_node = Node(
        package='robot_teleoperation',
        executable='teleoperation_fsm.py',
        name='left_teleoperation',
        parameters=[config],
        output="screen",
    )

    right_teleoperation_node = Node(
        package='robot_teleoperation',
        executable='teleoperation_fsm.py',
        name='right_teleoperation',
        parameters=[config],
        output="screen",
    )


    return LaunchDescription([
        left_teleoperation_node,
        right_teleoperation_node

    ])


