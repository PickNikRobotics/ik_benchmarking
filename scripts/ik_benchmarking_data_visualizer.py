#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

import rclpy
from rclpy.node import Node


class DataVisualizerNode(Node):
    def __init__(self):
        super().__init__("data_visualizer_node")

        # Allow loading the IK benchmarking data from non-current directories
        # using a 'data_directory' parameter that can be passed when running the script
        # the 'data_directory' parameter defaults to current directory, and if no data exist, a warning is printed
        self.declare_parameter("data_directory", os.getcwd())
        self.data_directory = (
            self.get_parameter("data_directory").get_parameter_value().string_value
        )
        print(f"{'=' * 60}")
        print(
            f"\nThe benchmarking CSV files will be loaded from the directory:\n\n{self.data_directory}"
        )
        print(f"{'=' * 60}")

        self.run_visualization()

    def run_visualization(self):
        data_list = self.read_ik_benchmarking_files()

        # Check if the data files really exist
        if not data_list:
            self.get_logger().warn(
                f"No IK benchmarking CSV data files found in the directory: {self.data_directory}"
            )
            rclpy.shutdown()
            return

        self.plot_data(data_list)

    def read_ik_benchmarking_files(self):
        file_pattern = os.path.join(self.data_directory, "*ik_benchmarking_data.csv")
        files = glob.glob(file_pattern)
        data_list = []

        for file in files:
            data = pd.read_csv(file)

            # Convert solve_time and errors to numeric
            for col in ["solve_time", "position_error", "orientation_error"]:
                data[col] = pd.to_numeric(data[col], errors="coerce")

            data["found_ik"] = data["found_ik"] == "yes"

            # Process the filename and remove the common suffix
            file_label = os.path.basename(file).replace("_ik_benchmarking_data.csv", "")
            data_list.append((file_label, data))

        return data_list

    def plot_data(self, data_list):
        # Common light blue color for all plots
        common_color = sns.color_palette("pastel")[0]

        # Box and whisker plot for solve times of successful trials
        plt.figure(figsize=(15, 10))

        all_data = []
        labels = []

        for file, data in data_list:
            success_data = data[data["found_ik"]]
            all_data.extend(success_data.dropna(subset=["solve_time"])["solve_time"])
            labels.extend([file] * len(success_data))

        df_to_plot = pd.DataFrame({"Solve Times": all_data, "Dataset": labels})

        # Box plot for solve times
        sns.boxplot(
            x="Dataset",
            y="Solve Times",
            data=df_to_plot,
            showfliers=False,
            color=common_color,
            boxprops=dict(edgecolor="black"),
        )
        plt.title("Solve Times for Successful Trials")
        plt.ylabel("Microseconds")
        plt.xlabel("IK Solvers")

        # Bar chart for success rates
        plt.figure(figsize=(15, 10))
        success_rates = [(file, data["found_ik"].mean()) for file, data in data_list]
        labels, rates = zip(*success_rates)
        plt.bar(labels, rates, color=common_color, edgecolor="black")
        plt.ylim(0, 1)
        plt.title("Success Rate for Each Dataset")
        plt.ylabel("Rate")
        plt.xlabel("IK Solvers")

        # Box plot for position_error, and orientation_error
        error_types = [("position_error", "Meters"), ("orientation_error", "Radians")]

        for error_type, unit in error_types:
            plt.figure(figsize=(15, 10))
            all_error_data = []
            error_labels = []

            for file, data in data_list:
                success_data = data[data["found_ik"]]
                all_error_data.extend(
                    success_data.dropna(subset=[error_type])[error_type]
                )
                error_labels.extend([file] * len(success_data))

            df_error_to_plot = pd.DataFrame(
                {error_type: all_error_data, "Dataset": error_labels}
            )
            sns.boxplot(
                x="Dataset",
                y=error_type,
                data=df_error_to_plot,
                showfliers=False,
                color=common_color,
                boxprops=dict(edgecolor="black"),
            )
            plt.title(f'{error_type.replace("_", " ").title()} for Successful Trials')
            plt.ylabel(f'{error_type.replace("_", " ").title()} ({unit})')
            plt.xlabel("IK Solvers")

        plt.show()


if __name__ == "__main__":
    rclpy.init(args=None)
    node = DataVisualizerNode()
    if rclpy.ok():
        rclpy.shutdown()
