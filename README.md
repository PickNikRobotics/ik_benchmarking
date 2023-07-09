# IK Benchmarking with MoveIt 2

This ROS 2 package provides utilities for Inverse Kinematics (IK) solvers benchmarking for performance evaluation and comparison.

## Contents

1. **Robot Models**: Contains MoveIt config packages for various robot models
    - Kuka iiwa (7 DOF) arm

    - Universal Robot UR5 (6DOF) arm

    - NASA Valkyrie (44 DOF) humanoid

2. **IK Benchmarking Programs [WIP]**: Programs to measure and compare the performance of the IK solvers.
    - This [node](src/ik_benchmarks//src//run_ik_benchmarks.cpp) calculates average success rate and solving time.
    

## Getting Started

Clone the repo:

```
git clone https://github.com/Robotawi/ik_benchmarking_ws.git
```


Install dependencies and build the workspace:

```
cd ik_benchmarking_ws

rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

colcon build
```

Source the workspace:

```
source install/setup.bash
```

## Running the benchmarks (WIP)

The benchmarking program is designed to adapt to different robots and their corresponding moveit config packages. The main launch file `start_ik_benchmarks.launch.py` accepts two arguments, `moveit_config_pkg`, and `move_group`. 

The `moveit_config_pkg` argument specifies the name of the robot's moveit_config package. This package contains the necessary configuration files and parameters for the robot's motion planning using MoveIt. By convention, the name of this package is like `<robot_name>_moveit_config`. The `move_group` argument provides the name of the robot's move group to use in the evaluation. The move group represents a subset of the robot's joints that are used for planning robot motion. If these arguments are not provided, the benchmarking program defaults to using the `iiwa` robot.

Run the following command to start the benchmarking program with the `UR5` 6 DOF robot arm:
```
ros2 launch ik_benchmarks start_ik_benchmarks.launch.py moveit_config_pkg:=ur5_moveit_config move_group:=ur5_arm
```

To switch to the `iiwa` 7 DOF arm, modify the two arguments accordingly to match the move group and moveit config package of the `iiwa` robot:

```
ros2 launch ik_benchmarks start_ik_benchmarks.launch.py moveit_config_pkg:=iiwa_moveit_config move_group:=iiwa_arm
```