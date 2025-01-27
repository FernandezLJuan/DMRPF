import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#function to load data from .dat files
def load_data(directory):
    #initialize empty list for the data
    data = []

    #iterate through all the directories where results are expected to be stored
    # 5 - has a wait probability of 5%
    # 10 - has a wait probability of 10%
    # 20 - has a wait probability of 20%
    for prob_dir in ['5', '10', '20']:
        prob_path = os.path.join(directory, prob_dir)

        #for each .dat file in the directory
        for file_name in os.listdir(prob_path):
            if file_name.endswith('.dat'):
                file_path = os.path.join(prob_path, file_name) 
                with open(file_path, 'r') as file:
                    lines = file.readlines()
                    total_robots = 0 #counter for the total number of robots in the file
                    valid_robots = [] #list to store robtos which have reached their goals
                    for line in lines:

                        #skip comments and read lines with 4 columns (robot data related lines have 4 columns)
                        if not line.startswith('#') and len(line.split()) == 4:
                            robot_id, total_time, p_size, n_requests = line.split() #get the robot data
                            total_robots += 1 #add 1 to the total robots
                            if total_time != 'null': #if the robot has reached it's goal, count the metrics
                                valid_robots.append([int(prob_dir), int(robot_id), float(total_time), float(p_size), int(n_requests), total_robots])
                    
                    for robot in valid_robots:
                        robot[-1] = total_robots

                    #add the valid robots from this file to the overall data list
                    data.extend(valid_robots)

    #convert the final list into a pandas dataframe
    return pd.DataFrame(data, columns=['wait_probability', 'robot_id', 'tiempo_total', 'tamaño_relativo', 'n_peticiones', 'total_robots'])

def calculate_metrics(df, metric):
    #function to calculate mean and std deviation for valid robots
    grouped = df.groupby(['wait_probability', 'total_robots'])[metric].agg(
        mean='mean', #calculate mean of metric for each group
        std=lambda x: np.std(x, ddof=0)  #use ddof=0 for poblational standard deviation
    ).reset_index()

    return grouped

def print_metrics(grouped, metric):
    #prints the metrics grouped by delay and for each number of robots
    
    print(f"Averages and Standard Deviations for {metric}:\n")
    for density in grouped['wait_probability'].unique():
        print(f"Wait probability = {density}%")
        subset = grouped[grouped['wait_probability'] == density]
        for _, row in subset.iterrows():
            print(f"Total number of robots: {row['total_robots']}, Mean: {row['mean']:.2f}, Std: {row['std']}")
        print("\n")

#plot the metrics grouped by delay and number of robots
def plot_metrics(grouped, metric, map_name):
    colors = {5: 'red', 10: 'blue', 20: 'green'} #different colors for different way probability
    offsets = {5: -1.0, 10:0.0, 20: 1.0} #add offsets for better readability
    plt.figure(figsize=(10, 6))

    #plot by delay probability and number of robots in the simulation
    for density in grouped['wait_probability'].unique():
        subset = grouped[grouped['wait_probability'] == density]
        plt.errorbar(subset['total_robots'] + offsets[density], subset['mean'], yerr=subset['std'], fmt='o', label=f'Probabilidad de espera = {density}%', color=colors[density])

    plt.xlabel('Número de robots')
    plt.ylabel(f'{metric}')
    plt.legend()
    plt.title(f'{metric} en {map_name}')
    plt.grid(True)
    plt.show()


def main():

    #maps where tests with delays have been run
    maps = ['AR0015SR', 'warehouse-10-20-10-2-2']

    #metrics to be calcualted form the .dat files
    metrics = ['tiempo_total', 'tamaño_relativo', 'n_peticiones']
    print("1-total_time")
    print("2-p_size")
    print("3-n_requests\n")
    option = int(input("What metric do you want to see: "))

    #for each map, compute averages for the metrics and plot them in a graph
    for map_name in maps:
        directory = '../../results/'+map_name+'/delay_probs/'  #path to read results from
        df = load_data(directory)

        selected_metric = metrics[option-1]

        #filter out rows where the robot did not finish successfully
        df = df[df[selected_metric].notnull()]
        
        metric = selected_metric
        grouped = calculate_metrics(df, metric) #compute grouped metric averages
        print_metrics(grouped, metric)  #print the metrics for debugging
        plot_metrics(grouped, metric, map_name) #show the metrics in a graph

if __name__ == "__main__":
    main()

