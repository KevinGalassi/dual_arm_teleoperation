from collections import deque
from geometry_msgs.msg import TwistStamped, Vector3, Point
import numpy as np


class TwistSmoother:
    def __init__(self, ee_frame, vel_lin_gain=1.0, vel_twist_gain=1.0, ma_window=5):
        self.ee_frame = ee_frame
        self.vel_lin_gain = vel_lin_gain
        self.vel_twist_gain = vel_twist_gain
        self.ma_window = ma_window

        # Buffers for moving average
        self.linear_x_buffer = deque(maxlen=ma_window)
        self.linear_y_buffer = deque(maxlen=ma_window)
        self.angular_z_buffer = deque(maxlen=ma_window)


    def update_buffers(self, linear_x, linear_y, angular_z):
        self.linear_x_buffer.append(linear_x)
        self.linear_y_buffer.append(linear_y)
        self.angular_z_buffer.append(angular_z)

    def get_smoothed_twist(self, raw_twist: TwistStamped) -> TwistStamped:
        # Extract raw values
        linear_x = raw_twist.twist.linear.x
        linear_y = raw_twist.twist.linear.y
        angular_z = raw_twist.twist.angular.z

        # Update buffers
        self.update_buffers(linear_x, linear_y, angular_z)

        # Compute moving averages
        avg_linear_x = np.mean(self.linear_x_buffer)
        avg_linear_y = np.mean(self.linear_y_buffer)
        avg_angular_z = np.mean(self.angular_z_buffer)

        # Construct smoothed TwistStamped
        smoothed_twist = TwistStamped()
        smoothed_twist.header.stamp = raw_twist.header.stamp
        smoothed_twist.header.frame_id = raw_twist.header.frame_id
        smoothed_twist.twist.linear = Vector3(x=avg_linear_x, y=avg_linear_y, z=0.0)
        smoothed_twist.twist.angular = Vector3(x=0.0, y=0.0, z=avg_angular_z)

        return smoothed_twist
    

    def reset_buffer(self):
        self.linear_x_buffer = deque(maxlen=self.ma_window)
        self.linear_y_buffer = deque(maxlen=self.ma_window)
        self.angular_z_buffer = deque(maxlen=self.ma_window)

