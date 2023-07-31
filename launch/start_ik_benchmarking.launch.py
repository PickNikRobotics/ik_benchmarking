import os
import sys
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def load_benchmarking_config(ik_benchmarking_pkg, ik_benchmarking_config):
    # Construct the configuration file path
    file_path = os.path.join(get_package_share_directory(ik_benchmarking_pkg),
                             "config",
                             ik_benchmarking_config
                             )
    # Open file and parse content
    with open(file_path, "r") as config_file:
        config_data = yaml.safe_load(config_file)

    # Extract content and handle missing keys
    def get_config_data(key):
        value = config_data.get(key)
        if value is None:
            raise ValueError(f"Missing required configuration key {key}")
        return value

    moveit_config_pkg = get_config_data('moveit_config_pkg')
    move_group = get_config_data('move_group')
    kinematics_file = get_config_data('kinematics_file')
    sample_size = get_config_data('sample_size')
    ik_solver_1 = get_config_data('ik_solver_1')
    ik_solver_2 = get_config_data('ik_solver_2')
    ik_solver_3 = get_config_data('ik_solver_3')
    ik_solver_1_kinematics_file = get_config_data(
        'ik_solver_1_kinematics_file')
    ik_solver_2_kinematics_file = get_config_data(
        'ik_solver_2_kinematics_file')
    ik_solver_3_kinematics_file = get_config_data(
        'ik_solver_3_kinematics_file')

    # Return a dictionary to avoid errors due to return order
    return {
        'moveit_config_pkg': moveit_config_pkg,
        'move_group': move_group,
        'kinematics_file': kinematics_file,
        'sample_size': sample_size,
        'ik_solver_1': ik_solver_1,
        'ik_solver_2': ik_solver_2,
        'ik_solver_3': ik_solver_3,
        'ik_solver_1_kinematics_file': ik_solver_1_kinematics_file,
        'ik_solver_2_kinematics_file': ik_solver_2_kinematics_file,
        'ik_solver_3_kinematics_file': ik_solver_3_kinematics_file
    }


def get_robot_name(moveit_config_pkg):
    parts = moveit_config_pkg.split("_")

    if len(parts) < 2:
        print(
            "Error: The package name {moveit_config_pkg} is not standard. Please use 'robot_moveit_config'.")
        exit(1)

    robot_name = parts[0]
    return robot_name


def generate_launch_description():

    ik_benchmarking_pkg = "ik_benchmarking"
    ik_benchmarking_config = "ik_benchmarking.yaml"
    benchmarking_config = load_benchmarking_config(
        ik_benchmarking_pkg, ik_benchmarking_config)

    robot_name = get_robot_name(benchmarking_config['moveit_config_pkg'])

    # Build moveit_config using the robot name and kinematic file
    moveit_config = (MoveItConfigsBuilder(robot_name)
                     .robot_description_kinematics(
        file_path=os.path.join(
            get_package_share_directory(
                benchmarking_config['moveit_config_pkg']),
            "config",
            benchmarking_config['kinematics_file'],
        )
    )
        .to_moveit_configs()
    )

    # Start benchmarking server node with required robot description and move_group parameters
    benchmarking_server_node = Node(
        package="ik_benchmarking",
        executable="ik_benchmarking_server",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "move_group": benchmarking_config['move_group'],
                "sample_size": benchmarking_config['sample_size']
            },
        ],
    )

    # Start benchmarking client node with the same parameters as the server
    benchmarking_client_node = Node(
        package="ik_benchmarking",
        executable="ik_benchmarking_client",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "move_group": benchmarking_config['move_group'],
                "sample_size": benchmarking_config['sample_size']
            },
        ],
    )

    return LaunchDescription([benchmarking_server_node, benchmarking_client_node])
