# Use the official ROS base image
FROM osrf/ros:humble-desktop

# Ensure that the Ubuntu Universe repository is enabled
RUN apt-get update && apt install -y \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe 

# Development tools: Compilers and other tools to build ROS packages
RUN apt-get update && apt install -y \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro\
    ros-humble-joint-state-publisher\
    ros-humble-tf-transformations \
    ros-humble-usb-cam \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration\
    ros-humble-image-proc \
    python3-pip \
    unzip \
    libsecret-1-0 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install transforms3d
RUN pip install opencv-python
RUN pip install opencv-contrib-python 
RUN pip install pydantic==1.10.9

# Source ROS Humble
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Uninstall ros-humble-turtlesim: since it will collide with the manaul turtlesim package
RUN apt-get update && apt remove -y \
    ros-humble-turtlesim \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt install -y \
    jq \
    && rm -rf /var/lib/apt/lists/*
# Install and Setup Arduino
WORKDIR /root
RUN wget -O arduino-ide_2.2.1_Linux_64bit.zip https://github.com/arduino/arduino-ide/releases/download/2.2.1/arduino-ide_2.2.1_Linux_64bit.zip
RUN unzip -d . arduino-ide_2.2.1_Linux_64bit.zip && rm -rf arduino-ide_2.2.1_Linux_64bit.zip
RUN echo "alias arduino-ide='/root/arduino-ide_2.2.1_Linux_64bit/arduino-ide  --no-sandbox'" >>~/.bashrc

# Set up workspace
RUN mkdir -p /root/hologlyph_bots/src
WORKDIR /root/hologlyph_bots/src

# Copy your ROS package to the workspace
COPY . /root/hologlyph_bots/src

# Build the workspace
WORKDIR /root/hologlyph_bots
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source Worksapce
RUN echo "source /root/hologlyph_bots/install/setup.bash" >> ~/.bashrc