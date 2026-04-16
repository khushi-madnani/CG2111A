FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-rplidar-ros \
    ros-humble-robot-localization \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ./src /ros2_ws/src

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["sleep", "infinity"]