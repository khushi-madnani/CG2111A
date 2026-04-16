FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-rplidar-ros \
    ros-humble-robot-localization \
    python3-serial \
    libwebsocketpp-dev \
    libasio-dev \
    ros-humble-foxglove-msgs \
    ros-humble-nlohmann-json-schema-validator-vendor \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
# COPY ./src /ros2_ws/src

# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
#     echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc 
RUN git config --global --add safe.directory /ros2_ws/src/foxglove-sdk && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "alias start_bridge='ros2 run foxglove_bridge foxglove_bridge'" >> ~/.bashrc && \
    echo "alias start_lidar='ros2 launch rplidar_ros rplidar_a1_launch.py'" >> ~/.bashrc && \
    echo "alias start_tf='ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser'" >> ~/.bashrc && \
    echo "alias start_slam='ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p odom_frame:=base_link -p base_frame:=base_link -p scan_topic:=/scan -p mode:=mapping -p use_scan_matching:=true -p sensor_qos_reliability:=reliable -p transform_timeout:=1.0 -p minimum_travel_distance:=0.0 -p minimum_travel_heading:=0.0 -p map_update_interval:=0.2 -p lookup_transform_timeout_multiplier:=4.0'" >> ~/.bashrc
CMD ["sleep", "infinity"]