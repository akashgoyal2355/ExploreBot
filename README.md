# ExploreBot

This repo holds a ROS2 package named `explore_bot`.

# Overview
This package provides a simulation based control of a mobile robot that is capable of detecting colored balls in the environment and find their centroid and radius in pixels.

The robot is a custom designed chassis with 2 defferential drive wheels, 1 castor wheel, and a RGB camera. The robot can be controlled by sending velocity commands of type [geometry_msgs/msg/Twist](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html) over the `/cmd_vel` topic.

# Entities

1. URDF files:

    - `robot.urdf.xacro`: An XML file defining the robot's joint and links.
    - `robot.gazebo`: An XML file defining the gazebo materials of the robot's links.
    - `inertial_macros.xacro`: An XML file defining the inertial properties of the robor's links.
    - `gazebo_control.xacro`: An XML file defining the differential drive gazebo plugin for the robot's wheels.
    - `camera.xacro`: An XML file defning the robot's RGB camera link and the gazebo camera controller plugin.

2. `spawn.launch.py`: This is a ROS2 launch file written in Python that spawns the robot inside the custom `house.world` in Gazebo. It also launches the `robot_state_publisher` that uses the robot's URDF to publish the state of the robot. 

3. `color_detector_node.py`: This is a ROS2 node written in Python that subscribes to the Camera images and detects colored balls in those image frames. The node then overlays the detection information, such as the color of the ball, and the centroid and radius pixels on the image frames and outputs an OpenCV window with the overlayed image. The executable of this node is named `detector`.

# How to run

1. Clone the repository into your ROS workspace's `src` folder

    `git clone https://github.com/akashgoyal2355/ExploreBot`

2. Resolve the package's dependencies using [rosdep](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Rosdep.html)

3. Build the ROS workspace using `colcon`

    ```
    cd <your_ros_workspace>
    colcon build --symlink-install
    ```
4. Source the executables in your ROS workspace

    `source install/setup.bash`

5. Launch the `spawn.launch.py`

    `ros2 launch explore_bot spawn.launch.py`

6. Launch the `detector` node in another terminal

    `ros2 run explore_bot detector`

# Robot

# World

# RQT Graph

# Compatibility
This package is tested on

- ROS2 Galactic
- Ubuntu 20.04 Focal

# Dependencies

- OpenCV
- URDF
- imutils
- sensor_msgs
- cv_bridge