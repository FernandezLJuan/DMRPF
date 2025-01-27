import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#function to load result files from experiments and store data
def load_data(directory):
    data = []

    #iterate each directory where results are saved
    # 5 - 5% density
    # 10 - 10% density
    # 15 - 15% density
    for prob_dir in ['5','10','15']:
        prob_path = os.path.join(directory, prob_dir)

        #read results files
        for file_name in os.listdir(prob_path):
            if file_name.endswith('.dat'):
                file_path = os.path.join(prob_path, file_name)
                with open(file_path, 'r') as file:
                    lines = file.readlines()
                    total_robots = 0 #total robots in the fleet
                    valid_robots = 0 #number of robots that have reached the goal
                    for line in lines:

                        #if reading the robot data section
                        if not line.startswith('#') and len(line.split()) == 4:
                            robot_id, total_time, p_size, n_requests = line.split()
                            total_robots += 1 #robots in the fleet
                            if total_time != 'null':
                                valid_robots += 1 #if the robot has reached the goal, count it as a valid robot
                    
                    if total_robots > 0:
                        #compute precision and store alongside dynamic obstacle probability
                        precision = (valid_robots / total_robots)*100
                        data.append([int(prob_dir), total_robots, precision])

    return pd.DataFrame(data, columns=['obstacle_density', 'total_robots', 'precision'])

#function to calculate mean precision and std deviation for each density and fleet size
def calculate_precision_metrics(df):
    grouped = df.groupby(['obstacle_density', 'total_robots'])['precision'].agg(['mean', 'std']).reset_index()
    return grouped

#function to print the precision data
def print_precision_metrics(grouped):
    print(f"Averages and Standard Deviations for Precision:\n")
    for density in grouped['obstacle_density'].unique():
        print(f"Obstacle density = {density}%")
        subset = grouped[grouped['obstacle_density'] == density]
        for _, row in subset.iterrows():
            print(f"Total number of robots: {row['total_robots']}, Mean Precision: {row['mean']:.2f}, Std Precision: {row['std']:.2f}")
        print("\n")

#function to plot the precision data
def plot_precision_metrics(grouped, map_name):
    colors = {5: 'red', 10: 'blue', 15: 'green'} #bind a color to each density
    offsets = {5: -0.2,10:0.0, 15: 0.2} #offsets for better readability
    plt.figure(figsize=(10, 6))

    #for each density and fleet size, plot mean of success rate and standard deviation
    for density in grouped['obstacle_density'].unique():
        subset = grouped[grouped['obstacle_density'] == density]
        plt.errorbar(subset['total_robots'] + offsets[density], subset['mean'], yerr=subset['std'], fmt='o', label=f'Dens. obst = {density}%', color=colors[density])

    plt.xlabel('Número de robots')
    plt.ylabel('Tasa de éxito (mean ± std)')
    plt.legend()
    plt.title('Tasa de éxito por número de robots y densidad de obstáculos dinámicos en '+map_name)
    plt.grid(True)
    plt.show()

def main():

    #each map with dynamic obstacle experiments
    maps = ['AR0015SR', 'warehouse-10-20-10-2-2']

    #for each map, compute precision and plot based on fleet size and density
    for map_name in maps:
        directory = '../../results/'+map_name+'/dynamic_probs/'
        df = load_data(directory)
        
        precision_grouped = calculate_precision_metrics(df)
        print_precision_metrics(precision_grouped)  #print the precision metrics for debugging
        plot_precision_metrics(precision_grouped, map_name)   #plot the precision metrics

if __name__ == "__main__":
    main()
