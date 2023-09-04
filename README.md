# IK Solvers Benchmarking using ROS2 and MoveIt2

## Introduction

Inverse Kinematics (IK) serves as a foundational element in robotic systems, facilitating purposeful interactions with the surrounding environment. It empowers robots to achieve specific poses and reach target locations with precision. Despite the importance of IK solvers in robotic planning and control, choosing the right one can be a complex decision. Different IK solvers offer unique strengths and weaknesses, raising the need to conduct a performance evaluation for specific use-cases.

This `ik_benchmarking` package utilizes ROS 2 and MoveIt 2 to offer a suite of benchmarking utilities designed to aid the evaluation of IK solvers. This tutorial is crafted to walk you through the installing the package, configuring IK solvers for benchmarking, running the necessary scripts for data collection and visualization of the results for easier analysis. 

In addition, the architectural components of the package are outlined with key classes that enable its functionality. Towards the end, we discuss potential future improvements, ensuring that the package remains aligned with emerging needs and technologies.


## Installation 

In the following steps, the `ik_benchmarking` is assumed to be installed in the `ws_moveit2` workspace as it is closely connect with MoveIt 2, but feel free to use your own workspace.


1. **Clone the repository**
    ```bash
    cd ~/ws_moveit2/src
    git clone https://github.com/Robotawi/ik_benchmarking.git
    ```

2. **Build the package as follows**
    ```bash
    cd ~/ws_moveit2
    colcon build --packages-select ik_benchmarking --symlink-install
    ```
    If you made a fresh workspace, build the whole workspace
    ```bash
    colcon build --symlink-install
    ```
Note: Including the `--symlink-install` flag is advantageous as it allows you to make changes to the package files without requiring a complete rebuild of the workspace. This applies only to files that are interpreted at run time, like YAML, Python scripts, etc. 


3. **Source the Workspace**
    ```bash
    source install/setup.bash
    ```

With these steps completed, we are now set to dive into the configuration of the IK solvers for benchmarking purposes.


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