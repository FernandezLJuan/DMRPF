import os
import math
import re
import matplotlib.pyplot as plt
import numpy as np

# Function to find files
def find_files(path, map_name):
    return [file for file in os.listdir(path) if map_name in file]

# Path and map name
results_path = "../results/"
map_name = str(input("What map do you want results for: "))
results_path += map_name + "/"

result_files = find_files(results_path, map_name)
min_range = int(input("Starting simulation: "))
max_range = int(input("Ending simulation: "))

pattern = r'_(\d+)\.dat$'
all_simulations = {}

# Parse files
for results in result_files:
    match = re.search(pattern, results)
    if not match:
        continue

    sim_num = int(match.group(1))
    if not (min_range <= sim_num <= max_range):
        continue

    with open(results_path + results, 'r') as f:
        lines = f.readlines()

    current_section = None
    num_robots = 0
    valid_robots = 0

    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            if line.startswith("#Robot Data"):
                current_section = "robot"
            elif line.startswith("#Global Data"):
                current_section = "global"
            continue

        if current_section == "robot":
            # Parse robot data
            parts = line.split()
            total_time = parts[1]
            num_robots += 1
            if total_time != "null":
                valid_robots += 1

    # Store precision
    precision = (valid_robots / num_robots) * 100 if num_robots > 0 else 0
    all_simulations.setdefault(num_robots, []).append(precision)

# Calculate averages and standard deviations for precision
avg_precisions = []
std_precisions = []
robot_counts = sorted(all_simulations.keys())

for count in robot_counts:
    values = all_simulations[count]
    avg_precisions.append(np.mean(values))
    std_precisions.append(np.std(values))

print("Average precisions: ", avg_precisions)
print("std dev: ", std_precisions)

# Plot precision
plt.figure(figsize=(8, 6))
plt.errorbar(robot_counts, avg_precisions, yerr=std_precisions, fmt='o', capsize=5, label="Precision (%)")
plt.xlabel("Number of Robots")
plt.ylabel("Precision (%)")
plt.title("Precision of Method")
plt.grid()
plt.legend()
plt.show()
