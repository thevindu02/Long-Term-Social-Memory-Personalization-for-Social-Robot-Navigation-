FROM osrf/ros:humble-desktop

# Install Gazebo Fortress and ROS 2 integrations, Nav2, and basic utilities
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-gazebo \
    python3-pip \
    git \
    wget \
    nano \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Add a non-root user for safe X11 forwarding
RUN useradd -ms /bin/bash rosdev && \
    usermod -aG sudo rosdev && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER rosdev
WORKDIR /home/rosdev/gazebo_workspace

# Setup bashrc to auto-source ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
