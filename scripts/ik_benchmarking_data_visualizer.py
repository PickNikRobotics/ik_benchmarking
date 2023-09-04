import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


def read_ik_benchmarking_files():
    file_pattern = '*ik_benchmarking_data.csv'
    files = glob.glob(file_pattern)
    data_list = []

    for file in files:
        data = pd.read_csv(file)

        # Convert to numeric and drop non-numeric rows
        data['solve_time'] = pd.to_numeric(data['solve_time'], errors='coerce')
        data.dropna(subset=['solve_time'], inplace=True)

        data['found_ik'] = data['found_ik'] == 'yes'

        # Process the filename and remove the common suffix
        file_label = os.path.basename(file).replace(
            '_ik_benchmarking_data.csv', '')
        data_list.append((file_label, data))

    return data_list


def plot_data(data_list):
    # Box and whisker plot for solve times of successful trials
    plt.figure(figsize=(15, 10))

    all_data = []
    labels = []

    for file, data in data_list:
        success_data = data[data['found_ik']]
        all_data.extend(success_data['solve_time'])
        labels.extend([file] * len(success_data))

    df_to_plot = pd.DataFrame({
        'Solve Times': all_data,
        'Dataset': labels
    })

    # Plot the boxplot using seaborn
    sns.boxplot(x='Dataset', y='Solve Times',
                data=df_to_plot, showfliers=False)
    plt.title('Solve Times for Successful Trials')
    plt.ylabel('Microseconds')
    plt.xlabel('IK Solvers')
    plt.show()

    # Bar chart for success rates
    plt.figure(figsize=(15, 10))
    success_rates = [(file, data['found_ik'].mean())
                     for file, data in data_list]
    labels, rates = zip(*success_rates)
    plt.bar(labels, rates)
    plt.ylim(0, 1)
    plt.title('Success Rate for Each Dataset')
    plt.ylabel('Rate')
    plt.xlabel('IK Solvers')
    plt.show()

    # Scatter plots for position_error, orientation_error, and joints_error
    error_types = ['position_error', 'orientation_error', 'joints_error']

    for error_type in error_types:
        plt.figure(figsize=(15, 10))

        for file, data in data_list:
            success_data = data[data['found_ik']]
            plt.scatter([file] * len(success_data),
                        success_data[error_type], label=file)

        plt.title(
            f'{error_type.replace("_", " ").title()} for Successful Trials')
        plt.ylabel(error_type.replace("_", " ").title())
        plt.xlabel('IK Solvers')
        plt.legend()
        plt.show()


if __name__ == "__main__":
    data_list = read_ik_benchmarking_files()
    plot_data(data_list)
