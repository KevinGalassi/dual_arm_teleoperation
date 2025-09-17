import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time

from hand_tracker.camera import OAKCamera  # Assuming your class is saved as oak_camera.py

class OAKCameraNode(Node):

    def __init__(self):
        super().__init__('oak_camera_node')

        # Parameters similar to usb_cam
        self.declare_parameter('camera_name', 'oak_camera')
        self.declare_parameter('frame_id', 'oak_camera_frame')
        self.declare_parameter('fps', 30)

        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value

        self.camera = OAKCamera()
        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

        self.timer_period = 1.0 / self.fps
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(f'{self.camera_name} initialized at {self.fps} FPS')

    def timer_callback(self):
        img = self.camera.capture()

        if img is None:
            self.get_logger().warn('No image received from camera')
            return

        # Publish image
        msg = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

        # Publish dummy camera info (replicate usb_cam interface)
        cam_info = CameraInfo()
        cam_info.header = msg.header
        cam_info.width = img.shape[1]
        cam_info.height = img.shape[0]
        cam_info.k = [1,0,0,0,1,0,0,0,1]  # Dummy intrinsic matrix
        cam_info.d = [0.0,0.0,0.0,0.0,0.0]  # No distortion
        cam_info.r = [1,0,0,0,1,0,0,0,1]
        cam_info.p = [1,0,0,0,0,1,0,0,0,0,1,0]
        self.info_pub.publish(cam_info)


def main(args=None):
    rclpy.init(args=args)
    
    node = OAKCameraNode()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
