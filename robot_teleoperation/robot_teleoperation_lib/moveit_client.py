

import numpy as np
from enum import Enum, auto
import os, sys
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import TwistStamped, Vector3, Point
from hand_tracker.msg import HandTrack

from std_srvs.srv import Trigger

import sys

from robot_teleoperation_lib.twist_smoother import TwistSmoother
from robot_teleoperation_lib.gripper import GripperController


from moveit_msgs.action import MoveGroup

class MoveitClient():


    def __init__ (self, node : Node, group_name:str, action_name: str, joint_name:str, ready_joint_positions: list[float]):
        '''
        node:
        group_name
        joint_name
        action_name
        ready_joint_positions
        '''
        
        
        self.node = node
        self.group_name = group_name
        self.joint_name = joint_name
        self._ready_joint_positions = ready_joint_positions
        self.moveit_client = ActionClient(self.node, MoveGroup, action_name)


    # Set the target to the named pose 'ready'
    def send_homing_goal(self):

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


