#!/usr/bin/env python3

import os
import glob
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
    # Print the path where the resulting files will be saved 
    directory_path = os.getcwd()

    print(f"\n{'=' * 60}")
    print(f"\nThe benchmarking CSV files will be saved in the directory:\n\n{directory_path}")
    print(f"\n{'=' * 60}")

    # Load IK solvers data from ik_benchmarking.yaml file
    ik_benchmarking_pkg = "ik_benchmarking"
    ik_benchmarking_config = "ik_benchmarking.yaml"
    ik_solver_names = load_benchmarking_config(ik_benchmarking_pkg, ik_benchmarking_config)

    # Check if previous resulting CSV files already exist in the current directory
    current_csv_filenames = glob.glob("*.csv")
    result_csv_filenames = [ik_solver_name + "_ik_benchmarking_data.csv" for ik_solver_name in ik_solver_names]
    conflict_csv_filenames = []

    if current_csv_filenames:
        for filename in current_csv_filenames:
            if filename in result_csv_filenames:
                conflict_csv_filenames.append(filename)
    
    if conflict_csv_filenames:
        print("Warning: The current directory contains IK benchmarking files from previous runs: ")
        print(", ".join(conflict_csv_filenames)) 
        user_input = input("\nDo you want to permantently delete them and continue the benchmarking? (y/n): ")

        if user_input.lower() == 'y':
            for filename in conflict_csv_filenames:
                os.remove(filename)
            
            print("Conflicting CSV files deleted. Continuing with benchmarking...\n")

        elif user_input.lower() == 'n':
            print("Benchmarking aborted.")
            exit(0)
        
        else:
            print("Invalid input. Benchmarking aborted.")
            exit(1)

    # Commands to run ik benchmarking with different IK solvers
    launch_commands = [
        f"ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_name:={ik_solver_name}"
        for ik_solver_name in ik_solver_names
    ]

    for command in launch_commands:
        process = subprocess.Popen(command, shell=True, executable="/bin/bash")

        # Wait for completion or timeout after 30 seconds to run next command
        try:
            process.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == "__main__":
    main()
