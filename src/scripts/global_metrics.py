import os
import numpy as np
import matplotlib.pyplot as plt

#get metric data from the file
def process_file(file_path):
    num_robots = 0 #number of robots, used to group metrics
    op_conflicts, in_conflicts = None, None #count of both types of conflicts
    rule_activations = [0] * 6 #number of times each rule has been activated
    
    #parse the file
    with open(file_path, 'r') as file:
        lines = file.readlines()
        robot_section = False
        global_section = False

        for line in lines:
            #deterine which section is being read
            if line.startswith('#Robot Data'):
                robot_section = True
                continue
            if line.startswith('#Global Data'):
                robot_section = False
                global_section = True
                continue

            #if reading the robot section, count the number of robots
            if robot_section and line.strip() and not line.startswith('#'):
                num_robots += 1

            #if reading the global section, get the values of the global metrics
            if global_section and not line.startswith('#'):
                parts = line.split()
                op_conflicts = float(parts[0])
                in_conflicts = float(parts[1])
                rule_activations = [float(parts[i]) for i in range(2, 8)]
                global_section = False  #only need to read this once per file

    #return the number of robots and the values of the global metrics
    return num_robots, op_conflicts, in_conflicts, rule_activations

#plot total number of conflicts grouped by number of robots and map
def plot_conflicts(map_averages, group_name):
    plt.figure(figsize=(10, 6))

    #for each map, plot the metric averages
    for map_name, averages in map_averages.items():
        num_robots_list = list(averages.keys())
        op_conflicts_list = [averages[num]['op_conflicts'] for num in num_robots_list]
        in_conflicts_list = [averages[num]['in_conflicts'] for num in num_robots_list]
        
        #plot intersection conflicts as 'x' and swapping conflicts as circles
        plt.errorbar(num_robots_list, op_conflicts_list, label=f'{map_name} - op_conflicts', fmt='o', capsize=5)
        plt.errorbar(num_robots_list, in_conflicts_list, label=f'{map_name} - in_conflicts', fmt='x', capsize=5)
    
    #show the graph
    plt.xlabel('Número de Robots')
    plt.ylabel('Promedio de Conflictos')
    plt.title(f'Promedio de Conflictos para {group_name}')
    plt.legend()
    plt.grid(True)
    plt.show()

#plot the average number of activations of each rule grouped by robots
def plot_rule_activations(map_averages, group_name):
    for map_name, averages in map_averages.items():
        plt.figure(figsize=(10, 6))
        num_robots_list = list(averages.keys())
        for i in range(6):
            #plot the average activations of each rule
            rule_activations_list = [averages[num]['rule_activations'][i] for num in num_robots_list]
            plt.errorbar(num_robots_list, rule_activations_list, label=f'Rule {i + 1}', fmt='o', capsize=5)
        
        plt.xlabel('Número de Robots')
        plt.ylabel('Promedio de Activaciones de Reglas')
        plt.title(f'Promedio de Activaciones de Reglas para {map_name}')
        plt.legend()
        plt.grid(True)
        plt.show()

#function to process map data and compute averages and totals for different metrics
def process_map(data_dir, map_name):
    print(f"Processing map: {map_name}")
    
    #set the directory path for the current map
    map_dir = data_dir + map_name + '/deterministic/'
    
    #dictionary to store averages for different metrics by number of robots
    averages = {}
    
    #check if the map directory exists and is a valid directory
    if os.path.exists(map_dir) and os.path.isdir(map_dir):
        
        #iterate through all files in the map directory
        for file_name in os.listdir(map_dir):
            
            #only process .dat files
            if file_name.endswith('.dat'):
                file_path = os.path.join(map_dir, file_name)
                
                #process the file and extract relevant metrics (number of robots, conflicts, rule activations)
                num_robots, op_conflicts, in_conflicts, rule_activations = process_file(file_path)
                
                #if this is the first time encountering a certain number of robots, initialize the corresponding entry in averages
                if num_robots not in averages:
                    averages[num_robots] = {
                        'valid_counts': 0, #tracks the count of valid data entries for this number of robots
                        'op_conflicts': [], #list to store opposition conflicts
                        'in_conflicts': [], #list to store intersection conflicts
                        'rule_activations': [[] for _ in range(6)] #list of lists for rule activations (one list per rule)
                    }
                
                #increment the valid count for the current number of robots
                averages[num_robots]['valid_counts'] += 1
                #add current op_conflicts and in_conflicts to the respective lists
                averages[num_robots]['op_conflicts'].append(op_conflicts)
                averages[num_robots]['in_conflicts'].append(in_conflicts)
                #add current rule activations for each rule
                for i in range(6):
                    averages[num_robots]['rule_activations'][i].append(rule_activations[i])
    
        #if no valid data was found for the map, print a message and return an empty dictionary
        if not averages:
            print(f"No data found for map: {map_name}")
            return {}

        #calculate the overall averages for each metric (opposite conflicts, intersection conflicts, and rule activations)
        overall_averages = {
            num_robots: {
                'op_conflicts': np.mean(values['op_conflicts']),
                'in_conflicts': np.mean(values['in_conflicts']),
                'rule_activations': [np.mean(values['rule_activations'][i]) for i in range(6)]
            }
            for num_robots, values in averages.items()
        }

        #print summary of overall averages for the current map
        print(f"\nSummary for map: {map_name}")
        for num_robots, values in overall_averages.items():
            print(f"Robots: {num_robots} | Average op_conflicts: {values['op_conflicts']:.2f} | "
                  f"Average in_conflicts: {values['in_conflicts']:.2f}")

        #return the calculated averages
        return overall_averages
    
    #if the map directory doesn't exist, print an error message and return an empty dictionary
    else:
        print(f"Directory does not exist for map: {map_name}")
        return {}

def main(data_dir, map_groups):

    #compute averages for maps and fleet sizes and plot them
    for group_name, maps in map_groups.items():
        map_averages = {}
        
        for map_name in maps:
            map_averages[map_name] = process_map(data_dir, map_name)
        
        plot_conflicts(map_averages, group_name)
        plot_rule_activations(map_averages, group_name)

if __name__ == '__main__':
    mapas_agrupados = {
        "mapas reales": ['Paris_1_256', 'warehouse-10-20-10-2-2', 'warehouse-20-40-10-2-1'],
        "mapas conflictivos": ['AR0015SR', 'den520d']
    }

    data_dir = '../../results/'
    main(data_dir, mapas_agrupados)
