from collections import deque
from geometry_msgs.msg import TwistStamped, Vector3
import numpy as np

class TwistSmoother:
    """
    A class to smooth the input TwistStamped messages using a moving average filter.
    """

    def __init__(self, ee_frame: str, ma_window: int = 5):
        """
        Initialize the TwistSmoother.

        Args:
            ma_window (int, optional): The window size for the moving average. Defaults to 5.
        """
        self.ma_window = ma_window
        self.linear_x_buffer = deque(maxlen=ma_window)
        self.linear_y_buffer = deque(maxlen=ma_window)
        self.angular_z_buffer = deque(maxlen=ma_window)

    def update_buffers(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """
        Update the moving average buffers with new velocity values.

        Args:
            linear_x (float): Linear velocity in the x direction.
            linear_y (float): Linear velocity in the y direction.
            angular_z (float): Angular velocity around the z axis.
        """
        self.linear_x_buffer.append(linear_x)
        self.linear_y_buffer.append(linear_y)
        self.angular_z_buffer.append(angular_z)

    def get_smoothed_twist(self, raw_twist: TwistStamped) -> TwistStamped:
        """
        Compute and return a smoothed TwistStamped message using the moving average.

        Args:
            raw_twist (TwistStamped): The raw TwistStamped message to smooth.

        Returns:
            TwistStamped: The smoothed TwistStamped message.
        """
        linear_x = raw_twist.twist.linear.x
        linear_y = raw_twist.twist.linear.y
        angular_z = raw_twist.twist.angular.z

        self.update_buffers(linear_x, linear_y, angular_z)

        avg_linear_x = np.mean(self.linear_x_buffer)
        avg_linear_y = np.mean(self.linear_y_buffer)
        avg_angular_z = np.mean(self.angular_z_buffer)

        smoothed_twist = TwistStamped()
        smoothed_twist.header.stamp = raw_twist.header.stamp
        smoothed_twist.header.frame_id = raw_twist.header.frame_id
        smoothed_twist.twist.linear = Vector3(x=avg_linear_x, y=avg_linear_y, z=0.0)
        smoothed_twist.twist.angular = Vector3(x=0.0, y=0.0, z=avg_angular_z)

        return smoothed_twist

    def reset_buffer(self) -> None:
        """
        Reset all moving average buffers to empty.
        """
        self.linear_x_buffer = deque(maxlen=self.ma_window)
        self.linear_y_buffer = deque(maxlen=self.ma_window)
        self.angular_z_buffer = deque(maxlen=self.ma_window)
