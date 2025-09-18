from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from control_msgs.action import GripperCommand

class GripperController:
    """
    Wrapper class to interface with a robot gripper using ROS 2 action clients exposed by moveit.
    Provides methods to send open/close commands and manage gripper state.
    """
    def __init__(self, node: Node, gripper_action_server: str):
        """
        Initialize the GripperController.

        Args:
            node (Node): The ROS 2 node to use for communication.
            gripper_action_server (str): The name of the gripper action server.
        """
        self.node = node
        self._action_client = ActionClient(node, GripperCommand, gripper_action_server)
        self._last_gripper_cmd = None
        self.max_width = 0.04

    def send_gripper_goal(self, position: float, max_effort: float = 10.0)->Future:
        """
        Send a goal to the gripper action server to move the gripper to a specified position.

        Args:
            position (float): The target position for the gripper.
            max_effort (float, optional): The maximum effort to apply. Defaults to 10.0.

        Returns:
            Future: A future object for the goal result.
        """
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self._gripper_future_goal = self._action_client.send_goal_async(goal_msg)
        return self._gripper_future_goal

    def get_future(self)->Future:
        """
        Get the future object for the last sent gripper goal.

        Returns:
            Future: The future object for the last gripper goal.
        """
        return self._gripper_future_goal

    def close_gripper(self)->Future:
        """
        Send a command to close the gripper.

        Returns:
            Future: The future object for the close command.
        """
        return self.send_gripper_goal(0.0)

    def open_gripper(self)->Future:
        """
        Send a command to open the gripper to its maximum width.

        Returns:
            Future: The future object for the open command.
        """
        return self.send_gripper_goal(self.max_width)

    def done(self)->bool
        """
        Check if the last gripper goal has completed.

        Returns:
            bool: True if the last goal is done, False otherwise.
        """
        return self._gripper_future_goal.done()