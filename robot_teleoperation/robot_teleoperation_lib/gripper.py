from rclpy.node import Node
from rclpy.service import Service
from rclpy.action import ActionClient

from control_msgs.action import GripperCommand
from hand_tracker.msg import HandTrack
import rclpy


class GripperController:
    def __init__(self, node: Node, gripper_action_server: str):
        self.node = node
        self._action_client = ActionClient(node, GripperCommand, gripper_action_server)
        
        
        self._last_gripper_cmd = None
        self.max_width = 0.04
        

    def send_gripper_goal(self, position: float, max_effort: float = 10.0):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self._gripper_future_goal = self._action_client.send_goal_async(goal_msg)
        
        return self._gripper_future_goal
        
    ##### helper method
    def get_future(self):
        return self._gripper_future_goal

    def close_gripper(self):
        return self.send_gripper_goal(0.0)

    def open_gripper(self):
        return self.send_gripper_goal(self.max_width)


    def done(self):
        return self._gripper_future_goal.done()