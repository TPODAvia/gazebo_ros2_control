# gazebo_ros2_control

This is a ROS 2 package for integrating the `ros2_control` controller architecture with the [Gazebo Classic](http://gazebosim.org/) simulator.

This package provides a Gazebo plugin which instantiates a `ros2_control` controller manager and connects it to a Gazebo model.

## Documentation
See the [documentation file](doc/index.rst) or [control.ros.org](https://control.ros.org/master/doc/simulators/gazebo_ros2_control/doc/index.html)

## Build status

ROS 2 Distro | Branch | Build status | Documentation
:----------: | :----: | :----------: | :-----------:
**Rolling** | [`master`](https://github.com/ros-controls/gazebo_ros2_control/tree/master) | [![Gazebo ros2 control CI](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci.yaml/badge.svg?branch=master)](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci.yaml) | [Documentation](https://control.ros.org/master/index.html) <br /> [API Reference](https://control.ros.org/master/doc/api/index.html)
**Iron** | [`master`](https://github.com/ros-controls/gazebo_ros2_control/tree/master) | [![Gazebo ros2 control CI](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci.yaml/badge.svg?branch=master)](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci.yaml) | [Documentation](https://control.ros.org/master/index.html) <br /> [API Reference](https://control.ros.org/master/doc/api/index.html)
**Humble** | [`humble`](https://github.com/ros-controls/gazebo_ros2_control/tree/humble) | [![Gazebo ros2 control CI](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci.yaml/badge.svg?branch=humble)](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci.yaml) | [Documentation](https://control.ros.org/humble/index.html) <br /> [API Reference](https://control.ros.org/humble/doc/api/index.html)



## Code examples

These commands are used to launch and run different nodes and packages in a ROS2 environment. Here's a breakdown of what each command does:

1. `ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py`
   This file contains instructions to start certain nodes and services in the `gazebo_ros2_control_demos` package.
   
2. `ros2 run gazebo_ros2_control_demos example_position`
   The `example_position` node is responsible for controlling the position of a simulated object in the Gazebo environment.
   
3. `ros2 launch gazebo_ros2_control_demos cart_example_velocity.launch.py`
   This file contains instructions to start nodes and services that control the velocity of the simulated object.
   
4. `ros2 run gazebo_ros2_control_demos example_velocity`
   The `example_velocity` node controls the velocity of the simulated object in the Gazebo environment.
   
5. `ros2 run plot_pkg plot.py`
   This script generates plots based on the data produced by the nodes in the `gazebo_ros2_control_demos` package.