import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#function to load data from .dat files and saving metrics 
#grouping them by number of robots in the simulation and density of dynamic obstacles
def load_data(directory):
    data = []

    #iterate each directory where results are saved
    # 5 - 5% density
    # 10 - 10% density
    # 15 - 15% density
    for prob_dir in ['5','10','15']:
        prob_path = os.path.join(directory, prob_dir)
        for file_name in os.listdir(prob_path):

            #read each results file and extract metrics
            if file_name.endswith('.dat'):
                file_path = os.path.join(prob_path, file_name)
                with open(file_path, 'r') as file:
                    lines = file.readlines()
                    total_robots = 0 #total number of robots
                    valid_robots = [] #robots that have reached their goal
                    for line in lines:
                        #parse lines that aren't comments and correspond to robot data (4 columns)
                        if not line.startswith('#') and len(line.split()) == 4:
                            robot_id, total_time, p_size, n_requests = line.split()
                            total_robots += 1 #count total robots

                            if total_time != 'null': #save metrics if the robot has reached it's goal
                                valid_robots.append([int(prob_dir), int(robot_id), float(total_time), float(p_size), int(n_requests), total_robots])

                    #append total_robots to each valid robot entry
                    for robot in valid_robots:
                        robot[-1] = total_robots

                    data.extend(valid_robots)

    #convert data to a pandas dataframe to plot it
    return pd.DataFrame(data, columns=['obstacle_density', 'robot_id', 'tiempo_total', 'tamaño_relativo', 'n_peticiones', 'total_robots'])

#function to calculate mean and std deviation for valid robots
def calculate_metrics(df, metric):
    grouped = df.groupby(['obstacle_density', 'total_robots'])[metric].agg(
        mean='mean', #compute mean of valid robot metrics
        std=lambda x: np.std(x, ddof=0)  #use ddof = 0 to compute poblational standard deviation
    ).reset_index()
    return grouped

#function to print the  grouped by density and number of robots
def print_metrics(grouped, metric):
    print(f"Averages and Standard Deviations for {metric}:\n")
    for density in grouped['obstacle_density'].unique():
        print(f"Obstacle density = {density}%")
        subset = grouped[grouped['obstacle_density'] == density]
        for _, row in subset.iterrows():
            print(f"Total number of robots: {row['total_robots']}, Mean: {row['mean']:.2f}, Std: {row['std']}")
        print("\n")

#function to plot the data
def plot_metrics(grouped, metric, map_name):
    colors = {5: 'red', 10: 'blue', 15: 'green'} #assign a different color to each osbtacle density value
    plt.figure(figsize=(10, 6))

    #plot data for each dynamic obstacle density grouping by size of fleet
    for density in grouped['obstacle_density'].unique():
        subset = grouped[grouped['obstacle_density'] == density]
        plt.errorbar(subset['total_robots'], subset['mean'], yerr=subset['std'], fmt='o', label=f'Densidad de obstáculos = {density}%', color=colors[density])

    plt.xlabel('Número de robots')
    plt.ylabel(f'{metric}')
    plt.legend()
    plt.title(f'{metric} en {map_name}')
    plt.grid(True)
    plt.show()

def main():

    #maps for which the simulations with dynamic obstacles have been ran
    maps = ['AR0015SR', 'warehouse-10-20-10-2-2']

    #metrics to be computed and plotted
    metrics = ['tiempo_total', 'tamaño_relativo', 'n_peticiones']
    print("1-total_time")
    print("2-p_size")
    print("3-n_requests\n")
    option = int(input("What metric do you want to see: "))

    #for each map, compute mean and std to then print and plot them
    for map_name in maps:
        directory = '../../results/'+map_name+'/dynamic_probs/'  #directory where the results are stored
        df = load_data(directory)
        
        #filter out rows where the robot did not finish successfully
        df = df[df[metrics[option-1]].notnull()]
        
        metric = metrics[option-1]
        grouped = calculate_metrics(df, metric)
        print_metrics(grouped, metric)  #print the metrics for debugging
        plot_metrics(grouped, metric, map_name) #plot metrics grouped by obstacle density and number of robots

if __name__ == "__main__":
    main()
