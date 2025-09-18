from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

from moveit_msgs.action import MoveGroup

class MoveitClient():
    """
    A client class to interface with MoveIt! via ROS 2 ActionClient.
    Handles sending joint-space goals to a MoveGroup action server.
    """

    def __init__ (self, node : Node, group_name:str, action_name: str, joint_name:str, ready_joint_positions: list[float]):
        '''
        Initialize the MoveitClient.

        Args:
            node (Node): The ROS 2 node used for communication.
            group_name (str): The name of the MoveIt! planning group.
            action_name (str): The name of the MoveGroup action server.
            joint_name (str): The prefix for joint names.
            ready_joint_positions (list[float]): The target joint positions for the 'ready' pose.
        '''
        self.node = node
        self.group_name = group_name
        self.joint_name = joint_name
        self._ready_joint_positions = ready_joint_positions
        self.moveit_client = ActionClient(self.node, MoveGroup, action_name)

    def send_homing_goal(self)->Future:
        """
        Send a goal to MoveIt! to move the robot to the 'ready' joint configuration.

        Returns:
            future: The future object for the asynchronous goal request.
        """
        self._client_done = False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.planner_id = "RRTConnect"
        goal_msg.request.group_name = self.group_name
        goal_msg.request.goal_constraints.append(
            Constraints(
                name="ready",
                joint_constraints=[
                    JointConstraint(joint_name=f"{self.joint_name}{i+1}", position=self._ready_joint_positions[f"joint{i+1}"], weight=1.0)
                    for i in range(7)
                ]
            )
        )    
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        self.node.get_logger().info('sending')

        # Send goal asynchronously
        future = self.moveit_client.send_goal_async(goal_msg)
        return future


