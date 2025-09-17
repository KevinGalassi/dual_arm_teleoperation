import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():

    # ros2 launch hand_tracker camera.launch.py 
    camera_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('hand_tracker'), 'launch'),
         '/camera.launch.py'])
      )


    # ros2 run hand_tracker hand_tracker_node.py 
    tracker_node = Node(
        package = 'hand_tracker',
        executable='hand_tracker_node.py',
        name='hand_tracker'
    )

    # ros2 run rqt_image_view rqt_image_view 
    rqt_image_view_node = Node(
        package = 'rqt_image_view',
        executable='rqt_image_view',
        name='rqt_view'
    )



    return LaunchDescription(
        [
            camera_launch,
            tracker_node,
            rqt_image_view_node
        ]
    )
