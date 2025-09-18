#!/usr/bin/env python3
import copy

import numpy as np
from enum import Enum, auto

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Vector3, Point
from hand_tracker.msg import HandTrack

from std_srvs.srv import Trigger

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

        self.declare_parameter('group_name', 'left_panda_arm')                                      # Group name as defined in .srdf
        self.declare_parameter('joint_name', 'left_panda_joint')                                    # Prefix of robot joints
        self.declare_parameter('hand_tracking_topic', '/left/hand_tracking')                        # Hand-Tracking command topic
        self.declare_parameter('robot_twist_topic', '/left_arm/left_servo_node/delta_twist_cmds')   # Servo node twist command
        self.declare_parameter('ee_frame', 'left_panda_link8')                                      # Frame to move
        self.declare_parameter('gripper_action_server', '/left_panda_hand_controller/gripper_cmd')  # Action server of the gripper
        self.declare_parameter('servo_node_name', '/left_arm/left_servo_node')                      # Name of servo node (To start the servo)
        self.declare_parameter('clip_radius', 0.1)                                                  # Area in which command is not executed
        self.declare_parameter('vel_lin_gain', 0.1)                                                 # Linear velocity gain coefficient
        self.declare_parameter('vel_twist_gain', 0.087)                                             # Angular velocity gain coefficient
        self.declare_parameter('control_loop_frequency', 30)                                        # Desired frequecy loop in H<


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
        } # rad

        loop_freq = self.control_loop_frequency # Hz
        loop_time = 1.0/loop_freq
        self.timer = self.create_timer(loop_time, self.fsm_step)

        # FSM definition
        self.state = State.INIT
        self._empty_command_counter = 0
        self._empty_command_max = 20
        
        # State-transition synchronization helpers
        self._gripper_future = None
        self._homing_future = None

        self._gripper_result = None
        self._homing_result = None

        self._servo_start_response = None
        self._servo_start_future = None

        # Twist command smoother
        self.smoother = TwistSmoother(ee_frame=self.ee_frame, ma_window=5)

        self.transition_delay = None
        self.activation_delay = 1.5 #s

        # Publishers
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
            msg_type    = HandTrack,
            topic       = self.hand_tracking_topic,
            callback    = self.tracking_callback,
            qos_profile = 10
        )
        self.msg = None

        # Start Servo Service
        self._start_servo        = self.create_service_client(Trigger, 'start_servo')

        # Moveit controller client (Used for homing procedure)
        self.moveit_client = MoveitClient(self, 
                                        group_name=self.group_name,
                                        action_name='move_action',
                                        joint_name=self.joint_name,
                                        ready_joint_positions=self._ready_joint_positions
                                    )

        # Gripper Action client
        self.gripper_controller = GripperController(self, self.gripper_action_server)

        self.get_logger().info('Init completed')



    def create_service_client(self, srv_type, name):
        print(f'Create client to: {self.servo_node_name}/{name}')
        client = self.create_client(srv_type, f'{self.servo_node_name}/{name}')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{name} service not available, waiting...')
        return client


    def tracking_callback(self, msg):
        self.msg : HandTrack = msg


    # State-Transition helpers methods
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


    # State action definition
    def init_action(self):    
        self.get_logger().warn('Proceding to opening gripper and starting servo controller', once=True)

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

    def  moving_transition_delay(self):
        self.state = State.MOVING
        self.transition_delay.cancel()
        self.transition_delay = None    


    def idle_action(self):
        # THUMB_UP -> start moving
        if self.msg and self.msg.gesture_id == HandTrack.THUMB_UP:
            if not self.transition_delay:
                self.get_logger().info("State transition from 'IDLE' to 'MOVING'")
                self.get_logger().info('"Victory" when you have finished moving, [Open/Close] hand to command the gripper')
                self.transition_delay = self.create_timer(self.activation_delay, lambda: (self.moving_transition_delay()))
            return
        
        # THUMB_DOWN -> Recovery to start state
        if self.msg and self.msg.gesture_id == HandTrack.THUMB_DOWN:
            self.state = State.RECOVERY
            self.get_logger().info("State transition from 'IDLE' to 'RECOVERY'")
            return
        

    def process_twist_command(self, msg:HandTrack):
        # 1) Managing of missing command, send 0 is not receiving any information for a fixed number of steps
        if not msg: 
            return

        # Execute when gesture == OPEN_PALM OR CLOSED_FIST
        target : Point = msg.point
        angle : float = msg.angle

        # NOTE: Set target to 0 if lower than a threshold
        # IDEA: Prevent robot's movement when the hand is in a neutral position (center of the image)
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


    def process_gripper_command(self, msg:HandTrack):
        if not msg:
            return
        future = self.gripper_controller.get_future()
        if future and not future.done():
            return
        else:
            if self._last_gripper_cmd != HandTrack.CLOSED_FIST and msg.gesture_id == HandTrack.CLOSED_FIST:
                self._last_gripper_cmd = HandTrack.CLOSED_FIST
                self.gripper_controller.close_gripper()

            elif self._last_gripper_cmd != HandTrack.OPEN_PALM and msg.gesture_id == HandTrack.OPEN_PALM:
                self._last_gripper_cmd = HandTrack.OPEN_PALM
                self.gripper_controller.open_gripper()


    def moving_action(self):
        if self.msg and self.msg.gesture_id == HandTrack.VICTORY:
            self.state = State.IDLE
            self.msg = None
            self.get_logger().info("State transition from 'MOVING' to 'IDLE'")
            self.get_logger().info('System in Idle action: available command:\nThumb up: Move to moving state\nThumb down: Move to recovery')
            return

        msg_to_execute = copy.deepcopy(self.msg)
        self.process_twist_command(msg_to_execute)
        self.process_gripper_command(msg_to_execute)
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
                self.state = State.ERROR
                self.get_logger().error("Error in init_action -> transition to 'ERROR'")

        else:
            pass
        

    def error_action(self):
        self.get_logger().error('System is in error, requested human intervention', throttle_duration_sec=5.0)


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