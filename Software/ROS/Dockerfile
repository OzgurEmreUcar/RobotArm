FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all,graphics,utility,compute

RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-gazebo-* \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-moveit \
    ros-humble-moveit-ros-visualization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit-setup-assistant \
    nano \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pyserial
WORKDIR /home
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN echo "source /home/robot_ws/install/setup.bash" >> /etc/bash.bashrc
CMD ["/bin/bash", "-c", "cd /home/robot_ws && exec /bin/bash"]
