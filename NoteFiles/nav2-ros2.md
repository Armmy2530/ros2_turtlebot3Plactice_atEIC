# Installation
- sudo apt install ros-humble-navigation2
- sudo apt install ros-humble-nav2-bringup
- sudo apt install ros-humble-turtlebot3-gazebo

# Run Example
- source /opt/ros/humble/setup.bash
- export TURTLEBOT3_MODEL=waffle
- export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
- ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False