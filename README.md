# Decentralized Multi-Robot Path Finding

This repository contains an implementation of the behavior described in the paper [Decentralized Multi-Agent Path Finding in Dynamic Warehouse Environments](https://ieeexplore.ieee.org/document/10406648) by Anders Lyne Christensen and Abderraouf Maoudj.

The code implements a simulation of a grid environment in which each cell is a node in an 8-connected graph. Robots move across the different cells and solve conflicts locally.

## Installation

To build and run this project, follow these steps:

### Prerequisites
Make sure you have the following installed:
- [Clang++](https://clang.llvm.org/) (g++ should do the trick as well, I just like Clang's warnings a bit more)
- [Raylib](https://www.raylib.com/)
- [X11](https://www.x.org/)
- [GL](https://www.khronos.org/opengl/)
- [pthread, dl, rt, config++] (These are typically available by default on Linux)

### Build Instructions
1. Compile the code:
   ```bash
   cd codigos_TFG/src
   make

## Running the Simulator

### Configuration File (`default.cfg`)
The simulator uses a configuration file named `default.cfg` to set up the environment and other parameters at runtime. This file defines settings such as the window size, grid dimensions, obstacle probabilities, and the number of robots. Below is an example configuration:

```plaintext
application:{
    window:{
        title = "simulator"
        size = {w = 900, h = 900}
    }
    environment:{
        cellDims = {w = 3, h = 3} # Width and height of each cell (affects rendering)
        origin = {x = 0, y = 0} # Origin of the grid (affects rendering)
        gridDims = {rows = 8, cols = 8} # Grid dimensions (rows and columns)
        obsProb = 0.0 # Probability of static obstacles in the grid
        dynamicProb = 0.0 #Density of dynamic obstacles
        waitProbability = 0.0 # Probability of robots waiting at a time step
        n_robots = 200 # Number of randomly placed robots

        map = "den520d.map" # Name of the map to load
    }
}

```
The map must be placed inside the `/codigos_TFG/maps/` directory. If no map name is provided, the code will use a default harcoded name (this name can be changed inside the main.cpp file). If the map is saved in a directory outside of `/codigos_TFG/maps/`, the full path to the map can be provided instead of just the map name. This allows the code to locate and load maps stored in other directories.

When a map is loaded, it will generate a map of rows $\times$ cols dimensions, where static obstacles will randomly generate at each cell with a probability of `obsProb`.


Depending on the map you're using, you can customize parameters like `cellDims` and `origin`. For example, the *den520d* map has large dimensions of $256\times 256$, so setting the `cellDims` to `3x3` pixels allows the entire map to fit inside a 900x900 window.

For smaller maps, you can increase the cell size to better fit the window. These parameters are purely for rendering purposes and do not affect the simulation.

Change the values of `dynamicProb` to set the density of transient obstacles during the simulation, and `waitProbability` to set the probability of a robot waiting at each time step.

When running the executable generated by `make`, a command line argument can be provided, indicating the number of simulations to be run in that map. If no number is provided, the default is 25 simulations. For example:  
``` bash
    ./sim 10
```

will run 10 simulations. 

When a simulation finishes, the results are saved in the `codigos_TFG/results/<map_name>/` directory, in a file with the name `<map_name>_<sim_num>.dat` where `<sim_num>` is the number of that simulation. If there are already simulation files inside that directory, `<sim_num>` will be the number of the latest simulation plus one.

If the `codigos_TFG/results/<map_name>` directory does not exist, it will be automatically created during the execution of the program to ensure that results can be saved.

Results of already run simulations are saved in `codigos_TFG/results/`, in case someone wants to test out the scripts with already run simulations. These results are not organized in a perfect structure by default, it is up to the user to manually organize them, modify the code or write a script to automatically move the files (or any other way I'm not thinking of now). I manually organized them inside the directory for easier use and categorization :)

Results where no dynamic obstacles or delays are present are under the `codigos_TFG/<map_name>/deterministic/` subdirectory of each map. For the maps `AR0015SR` and `warehouse-10-20-10-2-2`, there are two more subdirectories:

- `codigos_TFG/<map_name>/dynamic_probs/`: storing results where dynamic obstacles have been placed in the environment, has 3 subdirectories `5`, `10` and `15` where density is 5%, 10% and 15%, respectively.

- `codigos_TFG/<map_name>/delay_probs/`: storing results where random delays are introduced to the movement, has 3 subdirectories `5`, `10` and `20` where delay probability is 5%, 10% and 20%, respectively.


### Python Scripts
**results directories must be available for the Python scripts to work**

Several Python scripts are provided to compute and visualize various metrics, grouped by parameters such as the number of robots, dynamic obstacle density, and delay probability.

These scripts are stored in the `codigos_TFG/scripts/` directory and will read from the `codigos_TFG/results/` directory to compute and plot metrics. They should work out of the box with the provided organized results, but must be adapted to read from results stored in other directores and/or with different file structures.

- **`success.py`**: Calculates and displays the mean and standard deviation of the success rate across the performed experiments.  
- **`robot_metrics.py`**: Plots the mean and standard deviation for `total_time` and `relative_path_size`.
- **`global_metrics.py`**: Plots the mean of total conflicts for each map, including both opposite and intersection conflicts. It also displays the mean activation counts for each priority rule.  

Scripts with the prefix **`dynamic_`** perform the same calculations for experiments involving dynamic obstacles, while scripts with the prefix **`delay_`** handle experiments where random delays are introduced to robot movement.
