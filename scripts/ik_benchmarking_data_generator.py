#!/usr/bin/env python3

import subprocess


def main():
    # Assume the ik_benchmarking package is inside ws_moveit2 and source the workspace
    source_command = "source /home/$USER/ws_moveit2/install/setup.bash"

    # Commands to run ik benchmarking with different IK solvers
    launch_commands = [
        "ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_number:=1",
        "ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_number:=2",
        "ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_number:=3"
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
