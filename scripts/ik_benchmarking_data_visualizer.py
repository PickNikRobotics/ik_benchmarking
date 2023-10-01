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

        # Convert solve_time and errors to numeric
        for col in ['solve_time', 'position_error', 'orientation_error','joints_error']:
            data[col] = pd.to_numeric(data[col], errors='coerce')

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
        all_data.extend(success_data.dropna(subset=['solve_time'])['solve_time'])
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

    # Box plots for position_error, orientation_error, and joints_error
    error_types = ['position_error', 'orientation_error', 'joints_error']

    for error_type in error_types:
        plt.figure(figsize=(15, 10))
        all_error_data = []
        error_labels = []

        for file, data in data_list:
            success_data = data[data['found_ik']]
            all_error_data.extend(success_data.dropna(subset=[error_type])[error_type])
            error_labels.extend([file] * len(success_data))

        df_error_to_plot = pd.DataFrame({error_type: all_error_data, 'Dataset': error_labels})
        sns.boxplot(x='Dataset', y=error_type, data=df_error_to_plot, showfliers=False)
        plt.title(f'{error_type.replace("_", " ").title()} for Successful Trials')
        plt.ylabel(error_type.replace("_", " ").title())
        plt.xlabel('IK Solvers')
        plt.show()


if __name__ == "__main__":
    data_list = read_ik_benchmarking_files()
    plot_data(data_list)
