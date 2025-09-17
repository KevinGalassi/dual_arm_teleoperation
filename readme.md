## TODO
- manage quality of service
- reliable/best_effort, deadline/liefespan.
- Add insta-rpocess comunication


## Installation


https://ai.google.dev/edge/mediapipe/solutions/guide?hl=en



The nodes for the teleoperation can be found here:
https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

Follow the installation. Paritculary points "Download Source Code of MoveIt and the Tutorials" and "Build your Colcon Workspace"



## Changes made
### dual-arm_panda_moveit_config
- Edit xacro to change robot configuration
- Add " >- " to omp_planning.yaml->request_adapters to parse the field as a string rather than string_array


### Camera
The camera node used is a Luxonis OAK-1 camera which can be installed from source.


The binaries for the ROS can be installed using:
```
$ sudo apt install ros-<distro>-depthai-ros
```
More information: https://docs.luxonis.com/software/ros/depthai-ros/

To run the camera use:
```
ros2 launch depthai_ros_driver camera.launch.py
```
To visualize
```
ros2 run rqt_image_view rqt_image_view
```

## Run



##### Debug camera
ros2 launch hand_tracker camera.launch.py 
ros2 run hand_tracker hand_tracker_node.py 
ros2 run rqt_image_view rqt_image_view 

ros2 topic echo /left/hand_tracking 
ros2 topic echo /right/hand_tracking 


or

ros2 launch hand_tracker hand_tracker.launch.py

#### ROBOT
```
ros2 launch robot_teleoperation dual_servo_arm.launch.py 
```

```
ros2 launch robot_teleoperation teleoperation.launch.py
```


For the use with keyboard:
```
ros2 run robot_teleoperation left_keyboard_input 
ros2 service call /left_arm/left_servo_node/start_servo std_srvs/srv/Trigger {}
ros2 service call /right_arm/right_servo_node/start_servo std_srvs/srv/Trigger {}
```

#### Gripper
ros2 action send_goal /left_panda_hand_controller/gripper_cmd control_msgs/action/GripperCommand "{ command: { position: 0.0, max_effort: 10.0 } }"
ros2 action send_goal /right_panda_hand_controller/gripper_cmd control_msgs/action/GripperCommand "{ command: { position: 0.8, max_effort: 10.0 } }"

## Quick Comments