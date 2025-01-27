import os
import re
import matplotlib.pyplot as plt
import numpy as np

# Function to find files
def find_files(path, map_name):
    return [file for file in os.listdir(path) if map_name in file]


def plot_group(map_group, group_name):
    #define colors for different maps
    color_map = {
        'Paris_1_256': 'b', 
        'warehouse-10-20-10-2-2': 'g', 
        'warehouse-20-40-10-2-1': 'r', 
        'AR0015SR': 'm', 
        'den520d': 'c'
    }

    #stylized markers for each map
    markers = {
        'Paris_1_256': 'o', 
        'warehouse-10-20-10-2-2': 's', 
        'warehouse-20-40-10-2-1': 'D', 
        'AR0015SR': 'x', 
        'den520d': '^'
    }

    #offset for better readability
    offset = {
        'Paris_1_256': -2.0,
        'warehouse-10-20-10-2-2': 0.0,
        'warehouse-20-40-10-2-1': 2.0,
        'AR0015SR':-2.0,
        'den520d': 0.0
    }

    plt.figure(figsize=(12, 8))

    # Loop through maps and plot the precision with error bars for each map
    for map_name in map_group:
        if map_name in avg_precisions:
            # Collect the x and y values for plotting
            x_values = list(avg_precisions[map_name].keys())
            y_values = list(avg_precisions[map_name].values())
            yerr_values = list(std_precisions[map_name].values())

            x_offset = x_offset = [x + offset.get(map_name, 0.0) for x in x_values]

            # Plot with the label set for the map
            plt.errorbar(x_offset, y_values, yerr=yerr_values, fmt=markers[map_name], linestyle='none', color=color_map.get(map_name, 'k'), capsize = 5, label=map_name)

    plt.xlabel("Número de Robots")
    plt.ylabel("Precisión (%)")
    plt.title(f"Precisión Total por Grupo de Robots ({group_name})")
    plt.grid(True)
    plt.legend(title="Mapas")
    plt.show()

#path where results are stored, match result files with regex
results_path = "../../results/"
pattern = r'_(\d+)\.dat$'
all_simulations = {}

#all of the maps, including range of files for which success rate is computed
grouped_maps = {
    "mapas_reales": {
        'Paris_1_256': (126, 250),
        'warehouse-10-20-10-2-2': (151, 275),
        'warehouse-20-40-10-2-1': (126,250)
    },
    "mapas_conflictivos": {
        'AR0015SR': (1, 204),
        'den520d': (126, 250)
    }
}

#store the success rate values for each robot group and map
robot_groups_precisions = {}

for map_group, maps in grouped_maps.items():
    for map_name, (min_range, max_range) in maps.items():
        tmp_path = results_path + map_name + '/repeated_sims/'
        result_files = find_files(tmp_path, map_name)

        #parse files
        for results in result_files:
            #if the file doesn't follow the naming convention for results, don't read it
            match = re.search(pattern, results)
            if not match:
                continue

            sim_num = int(match.group(1))
            if not (min_range <= sim_num <= max_range):
                continue

            with open(tmp_path + results, 'r') as f:
                lines = f.readlines()

            #current_section determines wether robot data or global data is being read
            current_section = None

            #number of total robots
            num_robots = 0

            #number of robots which have reached the goal
            valid_robots = 0

            #read each line
            for line in lines:
                line = line.strip()
                if not line or line.startswith("#"):
                    if line.startswith("#Robot Data"):
                        current_section = "robot"
                    elif line.startswith("#Global Data"):
                        current_section = "global"
                    continue

                if current_section == "robot":
                    #if the robot has reached it's goal, count it as a valid robot
                    #the robot counts to the total robots regardless
                    parts = line.split()
                    total_time = parts[1]
                    num_robots += 1
                    if total_time != "null":
                        valid_robots += 1

            #compute precision based on robots that reached their goal and total robots
            precision = (valid_robots / num_robots) * 100 if num_robots > 0 else 0

            #accumulate precision values by robot group and map
            if map_name not in robot_groups_precisions:
                robot_groups_precisions[map_name] = {}

            if num_robots not in robot_groups_precisions[map_name]:
                robot_groups_precisions[map_name][num_robots] = []

            robot_groups_precisions[map_name][num_robots].append(precision)

#after accumulating the precision values in robot_groups_precisions
for map_name, robot_groups in robot_groups_precisions.items():
    print(f"\nPrecisions for map: {map_name}")
    
    #iterate through each robot group in the map
    for num_robots, precisions in sorted(robot_groups.items()):
        avg_precision = sum(precisions) / len(precisions) if precisions else 0
        print(f"  Number of robots: {num_robots}")
        print(f"    Average precision: {avg_precision:.2f}%")


#calculate average and standard deviation of success_rate for each robot group and map
avg_precisions = {}
std_precisions = {}
for map_name, robot_counts in robot_groups_precisions.items():
    avg_precisions[map_name] = {}
    std_precisions[map_name] = {}
    print(f"{map_name}")
    for robot_count, precisions in robot_counts.items():
        avg_precisions[map_name][robot_count] = np.mean(precisions)
        std_precisions[map_name][robot_count] = np.std(precisions)
        print(f"Number of robots: {robot_count}")
        print(f"    Standard deviation: {std_precisions[map_name][robot_count]:.2f}%")

# Plot for "mapas reales"
plot_group(grouped_maps["mapas_reales"], "Mapas Reales")

# Plot for "mapas conflictivos"
plot_group(grouped_maps["mapas_conflictivos"], "Mapas Conflictivos")
