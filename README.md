# ExploreBot

This repo holds a ROS2 package named `explore_bot`.

# Overview
`explore_bot` provides a simulation based control of a mobile robot that is capable of detecting colored balls in the environment, and find their centroid and radius in pixels.

The robot is a custom designed chassis with 2 defferential drive wheels, 1 castor wheel, and a RGB camera on-board. The robot can be controlled by sending velocity commands of type [geometry_msgs/msg/Twist](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html) over the `/cmd_vel` topic.

# Entities

1. URDF files:

    - `robot.urdf.xacro`: An XML file defining the robot's joint and links.
    - `robot.gazebo`: An XML file defining the gazebo materials of the robot's links.
    - `inertial_macros.xacro`: An XML file defining the inertial properties of the robot's links.
    - `gazebo_control.xacro`: An XML file defining the differential drive gazebo plugin for the robot's wheels.
    - `camera.xacro`: An XML file defning the robot's RGB camera link and the gazebo camera controller plugin.

2. `spawn.launch.py`: This is a ROS2 launch file written in Python that spawns the robot inside the custom `house.world` in Gazebo. It also launches the `robot_state_publisher` that uses the robot's URDF to publish the state of the robot on the `/robot_description` and the `/tf` and `/tf_static` ROS topics.

3. `color_detector_node.py`: This is a ROS2 node written in Python that subscribes to the Camera images on the `/camera/image_raw` ROS topic, and detects colored balls in those image frames. The node then overlays the detection information such as the color of the ball, and the centroid and radius pixels, on the corresponding image frame, and outputs an OpenCV window with the overlayed image. The executable of this node is named `detector`.

# How to run

1. Clone the repository into your ROS workspace's `src` folder

    `git clone https://github.com/akashgoyal2355/ExploreBot`

2. Resolve the package's dependencies using [rosdep](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Rosdep.html)

    ```
    cd <your_ros_workspace>

    rosdep install --from-paths src -y --ignore-src
    ```

3. Build the ROS workspace using `colcon`

    ```
    cd <your_ros_workspace>
    colcon build --symlink-install
    ```
4. Source the executables in your ROS workspace

    ```
    cd <your_ros_workspace>
    source install/setup.bash
    ```

5. Launch the `spawn.launch.py`

    `ros2 launch explore_bot spawn.launch.py`

    This will launch a Gazebo server with the robot spawned inside the `house.world`, as can be seen below.
    
    This robot can then be controlled by publishing velocity commands onto the `/cmd_vel` topic. The [teleop_twist_keyboard](https://index.ros.org/p/teleop_twist_keyboard/github-ros2-teleop_twist_keyboard/) can be used to publish the velocity commands and control the robot.

    `ros2 run teleop_twist_keyboard teleop_twist_keyboard `

6. Launch the `detector` node in another terminal after sourcing your ROS workspace.

    `ros2 run explore_bot detector`

    The `detector` node will open a OpenCV window with the camera frames published by the robot's camera. Additionally, as soon as there is a colored ball detected by the robot, the detection information can be observed on the window, as shown below.

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