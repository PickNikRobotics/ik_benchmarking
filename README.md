# IK Solvers Benchmarking using ROS2 and MoveIt2

## Introduction

Inverse Kinematics (IK) serves as a foundational element in robotic systems, 
facilitating purposeful interactions with the surrounding environment. 
It empowers robots to achieve specific poses and reach target locations with precision. 
Despite the importance of IK solvers in robotic planning and control, 
choosing the right one can be a complex decision. 
Different IK solvers offer unique strengths and weaknesses, 
raising the need to conduct a performance evaluation for specific use-cases.

This `ik_benchmarking` package utilizes ROS 2 and MoveIt 2 to offer a suite of 
benchmarking utilities designed to aid the evaluation of IK solvers. 
This tutorial is crafted to walk you through the installing the package, 
configuring IK solvers for benchmarking, running the necessary scripts for 
data collection and visualization of the results for easier analysis. 

In addition, the architectural components of the package are outlined with 
key classes that enable its functionality. Towards the end, we discuss potential future improvements, ensuring that the package remains aligned with emerging needs and technologies.


## Installation 

In the following steps, the `ik_benchmarking` is assumed to be installed in 
the `ws_moveit2` workspace as it is closely connect with MoveIt 2, 
but feel free to use your own workspace.


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
Note: Including the `--symlink-install` flag is advantageous as it allows you 
to make changes to the package files without requiring a complete rebuild of the workspace. 
This applies only to files that are interpreted at run time, like YAML, Python scripts, etc. 


3. **Source the Workspace**
    ```bash
    source install/setup.bash
    ```

With these steps completed, we are now set to dive into the configuration of the 
IK solvers for benchmarking purposes.


## Usage 

### Configuration via `ik_benchmarking.yaml`

Before running any benchmarking, it is crucial to set up the `ik_benchmarking.yaml` configuration file according to your needs. 
This file allows you to specify various settings like the MoveIt configuration package to load the robot model from, planning group of the robot that is pre-defined inside 
the MoveIt configuration package, sample size, and the IK solvers you wish to test. 
Below is an explanation of each key-value pair in the configuration file:

```yaml
moveit_config_pkg: moveit_resources_panda_moveit_config
planning_group: panda_arm
sample_size: 10000

ik_solvers:
  1:
    name: KDL
    kinematics_file: kdl_kinematics.yaml
  2:
    name: TRAC_IK
    kinematics_file: trac_ik_kinematics.yaml
  3:
    name: pick_ik
    kinematics_file: pick_ik_kinematics.yaml
```

#### Key Components

- `moveit_config_pkg`: Specifies the MoveIt configuration package for the robot arm you are benchmarking. 
For example, `moveit_resources_panda_moveit_config` is used for the Panda robot arm. 
By convention, the MoveIt configuration packages are named `robot_moveit_config` or 
`moveit_resources_robot_moveit_config`. Your robot's MoveIt config package should follow this convention. 

- `planning_group`: Indicates the name of the planning group that you wish to use for benchmarking IK solvers. In the example, the planning group is `panda_arm`.

- `sample_size`: Specifies the number of samples the benchmarking will run. 
For instance, setting it to `10000` means that each IK solver will be tested 10,000 times.

#### IK Solvers

The `ik_solvers` section is an indexed list of solvers to benchmark:

- The numeric keys (e.g., `1`, `2`, `3`) are used to identify the IK solvers. 
The actual numerical value has no specific significance but should be unique for each solver. 
The numbering starts from `1` and there is no limit on the number of solvers to be used to 
collect IK solving data in a single benchmarking run. 
Make sure each solver has a key in the list and the following `name` and `kinematics_file`.

- `name`: The name of the IK solver you are benchmarking, such as `KDL`, `TRAC_IK`, or `pick_ik`.

- `kinematics_file`: The YAML file that holds the solver's kinematic configuration. 
This YAML file, by convention, exists in the `config` directory of the robot's MoveIt configuration package.

By configuring the `ik_benchmarking.yaml` file appropriately, you can control which IK solvers to benchmark, allowing for a comprehensive evaluation.

### Running the benchmarking 

#### The `ik_benchmarking_data_generator.py` script

After configuring the `ik_benchmarking.yaml` file, 
the next step is to execute the benchmarking script, `ik_benchmarking_data_generator.py` which is
included as a node inside the `ik_benchmarking` package. 
This Python script serves as a convenient way to automate the process of launching the IK solver benchmarking tests.

#### Script Overview

Here is a link to the [ik_benchmarking_data_generator.py](./scripts/ik_benchmarking_data_generator.py) script. 
The script is designed to dynamically read the `ik_benchmarking.yaml` configuration file, 
and construct the launch commands for the different IK solvers.

#### Key Components

- `load_benchmarking_config()` function reads the `ik_benchmarking.yaml` configuration file for 
the `ik_benchmarking` package. It retrieves the names of all the IK solvers specified in the 
configuration file and returns them as a list of strings.

- `ik_solver_names` is a list contains the names of the IK solvers as specified in the configuration file.

- `launch_commands` is a dynamically generated list of ROS 2 launch commands, tailored based on the 
names of the available IK solvers in the `ik_solver_names`. Each command in this list 
includes a solver name as an argument. This argument directs the benchmarking process to utilize 
a specific IK solver for generating performance data.

- `subprocess` module is used to execute each entry in the `launch_commands`, 
launching the benchmarking process for the respective IK solvers.

#### How to Run

To execute the script, make sure you completed the installation step and **sourced** your workspace. 
The package generates output files in the current working directory in which it is executed. 
Run the following command to generate IK solving data for benchmarking process:

```bash
cd ~/ws_moveit2 #cd into your workspace
source install/setup.bash
ros2 run ik_benchmarking ik_benchmarking_data_generator.py
```

This will automatically start the benchmarking tests for the IK solvers specified in your `ik_benchmarking.yaml` configuration file. 

Note: Ensure you have the necessary permissions to run the script. You might need to run `chmod +x ik_benchmarking_data_generator.py` to make the script executable.

#### Generated Data 

While the Inverse Kinematics is being solved, several types of data are collected to 
understand the performance and accuracy of the IK solver being evaluated. 

**Solve Time:** is the time taken to find an IK solution, measured in microseconds.

**Solve Rate:** is the percentage of the successful trials to the total IK solution trials. 

**Position Error:** After finding the IK solution, the function calculates the position error by
comparing the robot tip link position from both the forward and inverse kinematics processes.

**Orientation Error:** Similarly, an orientation error is calculated by determining the angle between
robot tip link orientation from forward and inverse kinematics.

**Joint Error:** This is calculated as the Euclidean distance between the sampled joint values used to
calculated forward kinematics and the joint values resulting from the inverse kinematics solution.

The data generator script saves these details in a CSV file for each solver. 
The files are named by `<solver_name>_ik_benchmarking_data.csv`, 
while the `solver_name` is loaded from the `ik_benchmarking.yaml` config file.

Note: The command at the start of this section generates the files in the directory `~/ws_moveit2`.
The setting of the desirable output directory is to be implemented soon.

#### Visualization 
![](figures/box_plot_solve_time.png)

This point is expecting improvements for more convenience setting of the output directory 
that hosts the generated CSV files. However, for the purpose of testing, please copy the script
responsible for plotting the generated data from inside the `ik_benchmarking` package and run it as follows.

```bash
# Copy the script to the workspace
cd ~/ws_moveit2
cp src/ik_benchmarking/scripts/ik_benchmarking_data_visualizer.py .

# Run the script on the generated data 
python3 ik_benchmarking_data_visualizer.py 

```

The script loads files that end with the suffix `_ik_benchmarking_data.csv` and plots the data from them. 
Solve times are visualized with box plots, solve rates with bar charts, 
and the different types of errors are illustrated with scatter plots. 

### Using Cyclone DDS


**Note:** To ensure smooth operation of the IK Benchmarking, configure ROS 2 to use Cyclone DDS
as the default DDS middleware. If it's not already configured, follow these steps:

- Install Cyclone DDS for your ROS 2 distribution. 

```bash
sudo apt install ros-${ROS_DISTRO}-cyclonedds
```

- Set up the necessary environment variables

```bash
echo "export RMW_IMPLEMENTATION=cyclonedds" >> ~/.bashrc
```

- Start new terminal session. 


