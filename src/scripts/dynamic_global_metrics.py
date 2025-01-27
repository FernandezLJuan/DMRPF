import os
import pandas as pd
import matplotlib.pyplot as plt

#function to load data from .dat files
def load_data(directory):
    global_data = []

    #iterate each directory where results are saved
    # 5 - 5% density
    # 10 - 10% density
    # 15 - 15% density
    for prob_dir in ['5', '10', '15']:
        prob_path = os.path.join(directory, prob_dir)

        for file_name in os.listdir(prob_path):

            #read only results files
            if file_name.endswith('.dat'):
                file_path = os.path.join(prob_path, file_name)

                #open the file
                with open(file_path, 'r') as file:
                    lines = file.readlines()
                    total_robots = 0 #number of robots in the fleet

                    for line in lines:
                        #if the line is not a comment and has 4 columns, the robot data section is being read
                        if not line.startswith('#') and len(line.split()) == 4:
                            total_robots += 1 #count the number of robots

                        #when reading the global section, store data related to conflicts and rule activations
                        if line.startswith('#Global Data'):
                            global_line = lines[lines.index(line) + 2]
                            if not global_line.startswith('#'): #if line is not a comment
                                try:
                                    #store conflicts, rules, fleet size and obstacle density to create groups later
                                    op_conflicts, in_conflicts, *rules = map(int, global_line.split())
                                    global_data.append([int(prob_dir), op_conflicts, in_conflicts, *rules, total_robots])

                                except ValueError:
                                    continue

    #transform the data into a pandas dataframe to compute stats and later
    global_df = pd.DataFrame(global_data, columns=['obstacle_density', 'op_conflicts', 'in_conflicts', 'rule_1', 'rule_2', 'rule_3', 'rule_4', 'rule_5', 'rule_6', 'total_robots'])
    return global_df

#fucntion to calculate mean and std for the stored metrics
def calculate_global_metrics(df, metrics):
    grouped = df.groupby(['obstacle_density', 'total_robots'])[metrics].agg(['mean', 'std']).reset_index()
    return grouped

#function to plot both op_conflicts and in_conflicts together with offsets and error bars
def plot_conflicts_together(grouped, map_name):
    colors = {5: 'red', 10: 'blue', 15: 'green'} #bind each probability with a color
    offsets = {5: -1.0, 10: 0.0, 15: 1.0} #offset for better readability
    plt.figure(figsize=(12, 7))

    #plot conflicts based on obstacle density and robot fleet size
    for density in grouped['obstacle_density'].unique():
        subset = grouped[grouped['obstacle_density'] == density]
        offset = offsets[density]
        
        #plot opposite conflicts with circles
        plt.errorbar(subset['total_robots'] + offset, subset['op_conflicts']['mean'],
                     fmt='o', color=colors[density], label=f'Op Conflicts (Dens. {density}%)', capsize=5)
        
        #plot intersection conflicts with crosses
        plt.errorbar(subset['total_robots'] + offset, subset['in_conflicts']['mean'],
                     fmt='x', color=colors[density], label=f'In Conflicts (Dens. {density}%)', capsize=5, linestyle='')

    plt.xlabel('Número de robots')
    plt.ylabel('Conflictos')
    plt.legend()
    plt.title(f'Conflictos de oposición e intersección en {map_name}')
    plt.grid(True)
    plt.show()

def main():

    #maps for which experiments with dynamic obstacles have been run
    maps = ['AR0015SR', 'warehouse-10-20-10-2-2']

    #for each map
    for map_name in maps:
        directory = '../../results/' + map_name + '/dynamic_probs/'
        global_df = load_data(directory) #store data grouped by obstacle densities and fleet sizes

        #compute metrics
        grouped = calculate_global_metrics(global_df, ['op_conflicts', 'in_conflicts'])

        #plot metrics
        plot_conflicts_together(grouped, map_name)

if __name__ == "__main__":
    main()
