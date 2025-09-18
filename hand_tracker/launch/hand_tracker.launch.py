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
    camera_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 1920,
                'image_height': 1080,
                'pixel_format': 'mjpeg2rgb',
                'framerate': 30.0,
                'camera_frame_id': 'usb_cam'
            }]
        )


    # ros2 run hand_tracker hand_tracker_node.py 
    tracker_node = Node(
        package = 'hand_tracker',
        executable='hand_tracker_node.py',
        name='hand_tracker'
    )

    '''
    # ros2 run rqt_image_view rqt_image_view 
    rqt_image_view_node = Node(
        package = 'rqt_image_view',
        executable='rqt_image_view',
        name='rqt_view'
    )

    '''
    


    return LaunchDescription(
        [
            camera_node,
            tracker_node,
            #rqt_image_view_node
        ]
    )
