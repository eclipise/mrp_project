# uses official ROS Humble image as a base
FROM ros:humble

RUN apt-get update

# copies the workspace into the container
COPY src/jetson/dev_ws /ros_ws

# moves into the workspace
WORKDIR /ros_ws

# installs dependencies, explicitly skipping two that are only for simulation
RUN rosdep install --from-paths src --ignore-src -y --skip-keys gazebo_ros2_control --skip-keys gazebo_ros_pkgs

# builds the workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# sources ROS and the workspace, then starts the software
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch mrp launch_robot_nav.launch.py"]