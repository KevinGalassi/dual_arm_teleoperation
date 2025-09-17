#!/usr/bin/env python3

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
import time
import sys

from robot_teleoperation_lib.twist_smoother import TwistSmoother
from robot_teleoperation_lib.gripper import GripperController
from robot_teleoperation_lib.moveit_client import MoveitClient



class State(Enum):
    INIT     = auto()
    IDLE     = auto()
    MOVING   = auto()
    RECOVERY = auto()
    ERROR    = auto() 


class TeleFSMNode(Node):

    def __init__(self):
        '''
        hand_tracking_topic 
        
        '''
        super().__init__('fsm_node')

        self.declare_parameter('group_name', 'left_panda_arm') 
        self.declare_parameter('joint_name', 'left_panda_joint') 
        self.declare_parameter('hand_tracking_topic', '/left/hand_tracking') 
        self.declare_parameter('robot_twist_topic', '/left_arm/left_servo_node/delta_twist_cmds') 
        self.declare_parameter('ee_frame', 'left_panda_link8') 
        self.declare_parameter('gripper_action_server', '/left_panda_hand_controller/gripper_cmd') 
        self.declare_parameter('servo_node_name', '/left_arm/left_servo_node') 
        self.declare_parameter('clip_radius', 0.1) 
        self.declare_parameter('vel_lin_gain', 0.1) 
        self.declare_parameter('vel_twist_gain', 0.087) 
        self.declare_parameter('control_loop_frequency', 30)


        self.group_name             = self.get_parameter('group_name').get_parameter_value().string_value
        self.joint_name             = self.get_parameter('joint_name').get_parameter_value().string_value
        self.hand_tracking_topic    = self.get_parameter('hand_tracking_topic').get_parameter_value().string_value
        self.robot_twist_topic      = self.get_parameter('robot_twist_topic').get_parameter_value().string_value
        self.ee_frame               = self.get_parameter('ee_frame').get_parameter_value().string_value
        self.gripper_action_server  = self.get_parameter('gripper_action_server').get_parameter_value().string_value
        self.servo_node_name        = self.get_parameter('servo_node_name').get_parameter_value().string_value
        self.clip_values            = self.get_parameter('clip_radius').get_parameter_value().double_value
        self.vel_lin_gain           = self.get_parameter('vel_lin_gain').get_parameter_value().double_value
        self.vel_twist_gain         = self.get_parameter('vel_twist_gain').get_parameter_value().double_value
        self.control_loop_frequency = self.get_parameter('control_loop_frequency').get_parameter_value().integer_value

        self._ready_joint_positions ={
            "joint1": 0.0,
            "joint2": -0.785,
            "joint3": 0.0,
            "joint4": -2.36,
            "joint5": 0.0,
            "joint6": 1.57,
            "joint7": 0.785
        }

        loop_freq = self.control_loop_frequency # Hz
        loop_time = 1.0/loop_freq
        self.timer = self.create_timer(loop_time, self.fsm_step)

        # FSM
        self.state = State.INIT
        self._empty_command_counter = 0
        self._empty_command_max = 20
        
        self.smoother = TwistSmoother(ee_frame=self.ee_frame, vel_lin_gain=self.vel_lin_gain, vel_twist_gain=self.vel_twist_gain, ma_window=5)

        # Publisher
        self.pub_twist_cmd = self.create_publisher(
            TwistStamped,
            self.robot_twist_topic,
            10
        )

        parts = self.robot_twist_topic.split('/')
        parts[-1] = 'raw_' + parts[-1]
        raw_twist_topic = '/'.join(parts)
        self.pub_raw_twist_cmd = self.create_publisher(
            TwistStamped,
            raw_twist_topic,
            10
        )
    

        # Subscribers
        self.sub_tracker = self.create_subscription(
            msg_type    = HandTrack,  # from std_msgs.msg
            topic       = self.hand_tracking_topic,
            callback    = self.tracking_callback,
            qos_profile = 10
        )
        self.msg = None


        self._gripper_future = None
        self._homing_future = None

        self._gripper_result = None
        self._homing_result = None

        self._servo_start_response = None
        self._servo_start_future = None

        # Start Servo Service
        self._start_servo        = self.create_service_client(Trigger, 'start_servo')
        self._pause_servo        = self.create_service_client(Trigger, 'pause_servo')
        self._stop_servo         = self.create_service_client(Trigger, 'stop_servo')
        self._unpause_servo      = self.create_service_client(Trigger, 'unpause_servo')


        # Control for MOVEIT
        self.moveit_client = MoveitClient(self, 
                                        group_name=self.group_name,
                                        action_name='move_action',
                                        joint_name=self.joint_name,
                                        ready_joint_positions=self._ready_joint_positions
                                    )

        # Gripper Action
        self.gripper_controller = GripperController(self, self.gripper_action_server)

        self.get_logger().info('Init completed')
        self.get_logger().warn('Proceding to opening gripper....')



    def create_service_client(self, srv_type, name):
        print(f'Create client to: {self.servo_node_name}/{name}')
        client = self.create_client(srv_type, f'{self.servo_node_name}/{name}')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{name} service not available, waiting...')
        return client


    def tracking_callback(self, msg):
        self.msg : HandTrack = msg

    def _get_homing_result_callback(self, future):
        self._homing_result = future.result().result


    def _homing_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_homing_result_future = goal_handle.get_result_async()
        self._get_homing_result_future.add_done_callback(self._get_homing_result_callback)



    def _get_gripper_result_callback(self, future):
        self._gripper_result = future.result().result

    def _gripper_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_gripper_result_future = goal_handle.get_result_async()
        self._get_gripper_result_future.add_done_callback(self._get_gripper_result_callback)

    def _servo_start_response_callback(self, future):
        self._servo_start_future = True
        self._servo_start_response = future.result()


    def init_action(self):    

        if not self._gripper_future:
            self.get_logger().info(f"Sending Gripper")
            self._gripper_future = self.gripper_controller.open_gripper()
            self._gripper_future.add_done_callback(self._gripper_response_callback)

        if not self._servo_start_future:
            self._servo_start_future = self._start_servo.call_async(Trigger.Request())
            self._servo_start_future.add_done_callback(self._servo_start_response_callback)
        
        if self._gripper_result and self._servo_start_response:
            if self._gripper_result.reached_goal and self._servo_start_response.success:
                self.state = State.IDLE
                self.get_logger().info("State transition from 'INIT' to 'IDLE'")
                self.get_logger().info('System in Idle action: available command:\nThumb up: Move to moving state\nThumb down: Move to recovery')
                
                # Clear futures
                self._homing_future = None
                self._homing_result = None
                self._servo_start_future = None
                self._servo_start_response = None
                self._last_gripper_cmd = HandTrack.OPEN_PALM
                
            else:
                self.state = State.RECOVERY
                self.get_logger().error("Error in init_action -> transition to 'RECOVERY'")
        else:
            pass




    def idle_action(self):
        if self.msg and self.msg.gesture_id == HandTrack.THUMB_UP:
            self.state = State.MOVING
            self.get_logger().info("State transition from 'IDLE' to 'MOVING'")
            self.get_logger().info('"Victory" when you have finished moving, [Open/Close] hand to command the gripper')
            return

        if self.msg and self.msg.gesture_id == HandTrack.THUMB_DOWN:
            self.state = State.RECOVERY
            self.get_logger().info("State transition from 'IDLE' to 'RECOVERY'")
            return
        


    def process_twist_command(self):
        # Managing of missing command, send 0 is not receiving any information
        if not self.msg: self._empty_command_counter += 1
        if self._empty_command_counter >= self._empty_command_max:
            self._empty_command_counter = 0
            new_target = TwistStamped()
            new_target.header.stamp = self.get_clock().now().to_msg()
            new_target.header.frame_id = self.ee_frame

            self.pub_twist_cmd.publish(new_target)
            self.pub_raw_twist_cmd.publish(new_target)
            
            self.smoother.reset_buffer()
            return
        else:
            self._empty_command_counter = 0

        # Execute when gesture== OPEN_PALM OR CLOSED_FIST
        target : Point = self.msg.point
        angle : float = self.msg.angle

        # TODO: CLIP VALUE TO AVOID obscillation if close to target
        if False:
            target.x = 0.0 if abs(target.x) < 0.1 else target.x
            target.y = 0.0 if abs(target.y) < 0.1 else target.y
        else:
            if (target.x**2 + target.y**2)**0.5 < self.clip_values:
                target.x = 0.0
                target.y = 0.0
        
        angle    = 0.0 if abs(angle) < 5.0/180*np.pi else angle


        # Create new twist target
        raw_twist = TwistStamped()
        raw_twist.header.stamp = self.get_clock().now().to_msg()
        raw_twist.header.frame_id = self.ee_frame

        # NOTE: twist command ins ervo node should be in range [-1,1] (Line 977)
        # https://moveit.picknik.ai/humble/api/html/servo__calcs_8cpp_source.html
        linear = Vector3()
        linear.x = np.clip(target.x * self.vel_lin_gain, -1, 1)
        linear.y = np.clip(target.y * self.vel_lin_gain, -1, 1)
        raw_twist.twist.linear = linear

        angular = Vector3()
        angular.z = np.clip(angle * self.vel_twist_gain, -1, 1)
        raw_twist.twist.angular = angular

        # Smoothing using a moving average of target
        smoothed_twist = self.smoother.get_smoothed_twist(raw_twist)
        self.pub_twist_cmd.publish(smoothed_twist)
        self.pub_raw_twist_cmd.publish(raw_twist)


    def process_gripper_command(self):
        # Gripper
        future = self.gripper_controller.get_future()
        
        if future and not future.done():
            return
        else:
            if self._last_gripper_cmd != HandTrack.CLOSED_FIST and self.msg.gesture_id == HandTrack.CLOSED_FIST:
                self._last_gripper_cmd = HandTrack.CLOSED_FIST
                self.gripper_controller.close_gripper()

            elif self._last_gripper_cmd != HandTrack.OPEN_PALM and self.msg.gesture_id == HandTrack.OPEN_PALM:
                self._last_gripper_cmd = HandTrack.OPEN_PALM
                self.gripper_controller.open_gripper()


    def moving_action(self):

        if isinstance(self.msg, type(None)):
            return

        if self.msg.gesture_id == HandTrack.VICTORY:
            self.state = State.IDLE
            self.msg = None
            self.get_logger().info("State transition from 'MOVING' to 'IDLE'")
            self.get_logger().info('System in Idle action: available command:\nThumb up: Move to moving state\nThumb down: Move to recovery')
            return

        self.process_twist_command()
        self.process_gripper_command()
        self.msg = None

        return 
    





    def recover_action(self):
        self.get_logger().info(f"Recovery action", throttle_duration_sec=5.0)
        if not self._homing_future:
            self.get_logger().info(f"Sending homing")
            self._homing_future = self.moveit_client.send_homing_goal()
            self._homing_future.add_done_callback(self._homing_response_callback)

        if not self._gripper_future:
            self.get_logger().info(f"Sending Gripper")
            self._gripper_future = self.gripper_controller.open_gripper()
            self._gripper_future.add_done_callback(self._gripper_response_callback)


        if self._gripper_result and self._homing_result:
            
            if self._homing_result.error_code.val == 1 and self._gripper_result.reached_goal:
                self.state = State.IDLE
                self.get_logger().info("State transition from 'RECOVERY' to 'IDLE'")
                self.get_logger().info('System in Idle action: available command:\nThumb up: Move to moving state\nThumb down: Move to recovery')
            
                # Clear futures
                self._homing_future = None
                self._gripper_future = None
                self._homing_result = None
                self._gripper_result = None
                self._last_gripper_cmd = HandTrack.OPEN_PALM
                
            else:
                self.state = State.IDLE
                self.get_logger().error("Error in init_action -> transition to 'ERROR'")

        else:
            pass
        

    
    # def _check_homing_done(self, fut):
    #     TERMINAL_STATUSES = {2, 3, 4, 5, 8, 9}  # PREEMPTED, SUCCEEDED, ABORTED, REJECTED, RECALLED, LOST
    #     # Ensure both futures exist
    #     if not self._homing_future or not self._gripper_future:
    #         self.get_logger().info(f"on of future is None")
    #         return

    #     # Check if both futures are done
    #     if not self._homing_future.done() or not self._gripper_future.done():
    #         self.get_logger().info(f"One of the two has done false")
    #         return

        
    #     # Get goal results
    #     homing_result = self._homing_future.result()
    #     gripper_result = self._gripper_future.result()
        
        
    #     self.get_logger().info(f"Homing_status: {homing_result.status}")
    #     self.get_logger().info(f"Gripperstatus: {homing_result.status}")
        
    #     # Check MoveIt goal status for "done"
    #     if homing_result.status==2:
    #         self.get_logger().info(f"Homing_status: {homing_result.status}")
    #         self.get_logger().info(f"Gripperstatus: {homing_result.status}")

    #         if homing_result.status == 3 and gripper_result.status == 3:
    #             self.state = State.IDLE
    #             self.get_logger().info("State transition from 'RECOVERY' to 'IDLE'")
    #             self.get_logger().info('System in Idle action: available command:\nThumb up: Move to moving state\nThumb down: Move to recovery')
            
    #             # Clear futures
    #             self._homing_future = None
    #             self._gripper_future = None
    #             self._last_gripper_cmd = HandTrack.OPEN_PALM
                
    #         else:
    #             #self.state = State.ERROR
    #             self.get_logger().error(f"Homing {homing_result.status}")
    #             self.get_logger().error(f"Gripper {gripper_result.status}")

    #             self.get_logger().error("Error in init_action -> transition to 'ERROR'")


    #     else:
    #         self.get_logger().error(f"Homing {homing_result.status}")
    #         self.get_logger().error(f"Gripper {gripper_result.status}")




    # def send_goal(self, order):
    #     goal_msg = Fibonacci.Goal()
    #     goal_msg.order = order

    #     self._action_client.wait_for_server()

    #     self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    #     self._send_goal_future.add_done_callback(self.goal_response_callback)




    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected :(')
    #         return

    #     self.get_logger().info('Goal accepted :)')

    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info('Result: {0}'.format(result.sequence))
    #     rclpy.shutdown()

    # def feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))






    def error_action(self):
        self.get_logger().error('System if in error, requested human intervention', throttle_duration_sec=5.0)


    def fsm_step(self):
        if self.state == State.INIT:
            self.init_action()
        elif self.state == State.IDLE:
            self.idle_action()
        elif self.state == State.MOVING:
            self.moving_action()
        elif self.state == State.RECOVERY:
            self.recover_action()
        elif self.state == State.ERROR:
            self.error_action()




def main(args=None):

    rclpy.init(args=args)
   
    teleoperatioN_fsm = TeleFSMNode()
    
    rclpy.spin(teleoperatioN_fsm)

    teleoperatioN_fsm.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()