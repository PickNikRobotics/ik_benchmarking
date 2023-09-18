#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory

import subprocess

def load_benchmarking_config(ik_benchmarking_pkg, ik_benchmarking_config):
    # Construct the configuration file path
    file_path = os.path.join(get_package_share_directory(ik_benchmarking_pkg),
                             "config",
                             ik_benchmarking_config
                             )
    
    # Open config file and parse content related to only ik_solvers
    with open(file_path, "r") as config_file:
        config_data = yaml.safe_load(config_file)

    ik_solvers_data = config_data.get("ik_solvers")

    if ik_solvers_data is None:
        raise ValueError("Missing required configuration key 'ik_solvers'")
    
    ik_solver_names = [ik_value.get("name") for ik_value in ik_solvers_data]

    return ik_solver_names



def main():
    # Assume the ik_benchmarking package is inside ws_moveit2 and source the workspace
    source_command = "source /home/$USER/ws_moveit2/install/setup.bash"

    # Load IK solvers data from ik_benchmarking.yaml file
    ik_benchmarking_pkg = "ik_benchmarking"
    ik_benchmarking_config = "ik_benchmarking.yaml"
    ik_solver_names = load_benchmarking_config(ik_benchmarking_pkg, ik_benchmarking_config)

    # Commands to run ik benchmarking with different IK solvers
    launch_commands = [
        f"ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_name:={ik_solver_name}"
        for ik_solver_name in ik_solver_names
    ]

    for command in launch_commands:
        full_command = f"{source_command} && {command}"
        process = subprocess.Popen(
            full_command, shell=True, executable="/bin/bash")

        # Wait for completion or timeout after 30 seconds to run next command
        try:
            process.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == "__main__":
    main()
