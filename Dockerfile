FROM osrf/ros:humble-desktop

# Install packages
RUN apt-get update 

RUN apt-get install -y ros-humble-moveit \
    ros-humble-moveit-msgs \
    ros-humble-usb-cam \
    ros-humble-ros2-controllers \
    ros-humble-moveit-servo \
    ros-humble-ros2-control \
    ros-humble-moveit-resources

RUN apt-get install -y python3-pip
RUN pip install mediapipe



WORKDIR /workspace


# Copy local src folder into the workspace
COPY . /workspace/src

# Set working directory
WORKDIR /workspace

# Build ROS 2 packages
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"


# Source workspace on container start
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

