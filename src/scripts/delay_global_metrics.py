import os
import pandas as pd
import matplotlib.pyplot as plt

#function to load data from result files files
def load_data(directory):
    global_data = [] #list to store global data

    #iterate through all the directories where results are expected to be stored
    # 5  - has a wait probability of 5%
    # 10 - has a wait probability of 10%
    # 20 - has a wait probability of 20%
    for prob_dir in ['5', '10', '20']:
        prob_path = os.path.join(directory, prob_dir) #path to the current probability folder

        for file_name in os.listdir(prob_path):
            #only read .dat files
            if file_name.endswith('.dat'):
                file_path = os.path.join(prob_path, file_name)

                #read the results file
                with open(file_path, 'r') as file:
                    lines = file.readlines()
                    total_robots = 0
                    for line in lines:
                        #count number of robots in the fleet
                        if not line.startswith('#') and len(line.split()) == 4:
                            total_robots += 1

                        #read global data in the file and store it
                        if line.startswith('#Global Data'):
                            global_line = lines[lines.index(line) + 2] #skip the "#Global data" and the tag lines to read data
                            if not global_line.startswith('#'):
                                try:
                                    #store opposite conflicts, intersection conflcits and rule counts
                                    op_conflicts, in_conflicts, *rules = map(int, global_line.split())

                                    #store data with wait probability and number of robots
                                    global_data.append([int(prob_dir), op_conflicts, in_conflicts, *rules, total_robots])
                                except ValueError:
                                    continue

    #construct a dataframe with the global data including wait probability and number of robots
    global_df = pd.DataFrame(global_data, columns=['wait_probability', 'op_conflicts', 'in_conflicts', 'rule_1', 'rule_2', 'rule_3', 'rule_4', 'rule_5', 'rule_6', 'total_robots'])
    return global_df

#function to calculate mean and std deviation for selected metrics
def calculate_global_metrics(df, metrics):
    grouped = df.groupby(['wait_probability', 'total_robots'])[metrics].agg(['mean', 'std']).reset_index()
    return grouped

#function to print values grouped by delay probability
def print_grouped_values(grouped):
    densities = grouped['wait_probability'].unique()
    for delay in densities:
        print(f"Densidad - {delay}%:")
        subset = grouped[grouped['wait_probability'] == delay]
        for robots in subset['total_robots'].unique():
            mean_op = subset[subset['total_robots'] == robots]['op_conflicts']['mean'].values[0]
            std_op = subset[subset['total_robots'] == robots]['op_conflicts']['std'].values[0]
            mean_in = subset[subset['total_robots'] == robots]['in_conflicts']['mean'].values[0]
            std_in = subset[subset['total_robots'] == robots]['in_conflicts']['std'].values[0]
            print(f"  {robots} robots: op_conflicts = {mean_op:.2f} ± {std_op:.2f}, in_conflicts = {mean_in:.2f} ± {std_in:.2f}")
        print()

#function to plot both opposite and intersection conflicts together with offsets
def plot_conflicts_together(grouped, map_name):
    colors = {5: 'red', 10: 'blue', 20: 'green'} #bind a color to each delay probability
    offsets = {5: -1.0, 10: 0.0, 20: 1.0} #offsets for better readability
    plt.figure(figsize=(12, 7))

    #plot conflicts based on wait probabilities and number of robots
    for delay in grouped['wait_probability'].unique():
        subset = grouped[grouped['wait_probability'] == delay]
        offset = offsets[delay]

        #plot opposite conflicts with circles and error bars
        plt.errorbar(subset['total_robots'] + offset, subset[('op_conflicts', 'mean')],
                     fmt='o', color=colors[delay],
                     label=f'Conflictos de intersección (Prob. Espera. {delay}%)')

        #plot intersection conflicts with crosses
        plt.errorbar(subset['total_robots'] + offset, subset[('in_conflicts', 'mean')],
                     fmt='x', color=colors[delay],
                     label=f'Conflictos de oposición (Prob. Espera. {delay}%)', linestyle='')

    plt.xlabel('Número de robots')
    plt.ylabel('Conflictos')
    plt.legend()
    plt.title(f'Conflictos de oposición e intersección en {map_name}')
    plt.grid(True)
    plt.show()

def main():

    #maps for which the tests have been run
    maps = ['AR0015SR', 'warehouse-10-20-10-2-2']

    #for each map, read the results and plot them grouped by number of robots and delay probability
    for map_name in maps:
        directory = '../../results/' + map_name + '/delay_probs/' #path where results are stored
        global_df = load_data(directory)
        grouped = calculate_global_metrics(global_df, ['op_conflicts', 'in_conflicts'])

        print(f"Resultados para {map_name}:")
        print_grouped_values(grouped)
        plot_conflicts_together(grouped, map_name)

if __name__ == "__main__":
    main()
