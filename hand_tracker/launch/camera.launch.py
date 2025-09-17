from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
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
    ])