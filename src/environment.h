#pragma once

#include <array>
#include <ctime>
#include <memory>
#include <iostream>
#include <random>
#include <vector>
#include <raylib.h>
#include <tuple>
#include <set>
#include <map>
#include <thread>
#include <mutex>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <unordered_map>
#include <cassert>
#include "robot.h"
#include "cell.h"

namespace fs = std::filesystem;
typedef std::mt19937 MyRNG;

class Env{

public:
    /*constructor of class environment*/
	Env(int cellW, int cellH, int posX, int posY, int r, int c, int n_robots, float step, const float obsProb, const float dynProb, float robotDelay, const std::string& mapName);

    /*SIMULATION BEHAVIOUR FUNCTIONS*/
    void pauseSim(); /*pause the simualtion (environment doesn't update)*/
    void resumeSim(); /*resume the simulation*/
    void handle_input(unsigned long); /*capture keyPresses and mouse clicks, NOT USED DURING TESTS*/

    /*ENVIRONMENT MODIFICATION*/
    bool updateEnvironment(float); /*updates robot positions, cell neighbors and spawns random obstacles*/
    void addObstacle(int, bool); /*add an obstacle to the environment*/
    bool removeObstacle(int); /*remove an obstacle from the environment*/
    void addEdge(int, int, float); /*add connection between two cells in the graph*/
    void removeEdge(int, int); /*remove connection between two cells in the graph*/
    int load_benchmark(); /*load a .map file downlaoded from benchmark site*/
    void randomGrid(); /*fills the cell vector with rectangles*/
    void resetEnv(int, int, bool); /*clears cells and robot vectors*/
    void clearTransients(); /*remove all transient obstacles from environment*/
    void updateNeighborConnections(int, bool); /*update connections for a cell and all it's neighbours*/

    /*ROBOT MANAGEMENT FUNCTIONS*/
    int placeRobot(Robot*); /*place robot in the environment*/
    int moveRobot(Robot* , Cell*); /*update robot position*/
    int detectConflict(Robot*, Robot*); /*return the type of conflict it has detected*/
    void remakePaths(); /*remake all robot paths*/
    void addGoal(int); /*mark a cell as a goal*/
    void removeGoal(int); /*remove a cell as goal*/
    void randomizeRobots(); /*place robots and goals in random free cells*/

    /*GETTERS*/
    std::array<int, 2> getDims(); /*returns rows and cols*/
    std::array<int, 2> cellDims(); /*returns cellWidth and cellHeight*/
    std::array<int, 2> origin(); /*returns originX and originY*/
    Cell* getCellByPos(int , int );
    Cell* getCellByID(int );
    std::vector<std::unique_ptr<Cell>>& getCells();
    std::vector<std::unique_ptr<Robot>>& getRobots();
    float getStep();

    /*other environment info*/
    bool isRunning(); /*return true if the environment is paused*/
    bool isConnected(Cell&, Cell&); /*are the two cells connected?*/
    float cellDistance(Cell&, Cell&);
    float connectionCost(Cell&, Cell&); /*get the cost of the edge between to cells*/
    void dumpResults(); /*dumps the results of the simulation into a file*/

private:

    /*creates edges between every cell of the graph*/
    void connectCells();

    /*rendering parameters*/
    const float cellWidth, cellHeight;
    const int originX, originY;

    /*dimensions of the grid*/
    [[maybe_unused]] int rows;
    [[maybe_unused]] int cols; 

    /*Adjacency matrix to represent the graph used during planning*/
    std::unordered_map<int, std::unordered_map<int,float>> adjMatrix; /*SHOULD ALWAYS BE UPDATED BEFORE CELL NEIGHBORS*/

    float diagonalCost = sqrt(2); /*cost of diagonal connections is 1.414*/
    bool running = false; /*the environment is paused by default*/
    const int nRobots; /*number of robots expected in the environment*/
    float timeStep; /*duration of a time step in the simulation*/
    
    /*random number generator for obstacles*/
    MyRNG rng; /*random device*/
    std::uniform_real_distribution<float> obstacleDist; /*distribution of obstacle probability*/
    std::uniform_int_distribution<> cellDist = std::uniform_int_distribution<>(0,this->rows*this->cols); /*distribution for random cell obstacles*/
    
    /*probability parameters*/
    const float obstacleProbability; /*probability of a cell being an obstacle*/
    const float transientProbability; /*probability of a random obstacle appearing in the environment*/
    const float robotWaitProb; /*probability of a robot waiting each time step*/
    
    /*used to load and save results*/
    std::string mapName; /*map to load*/
    std::string resultsPath; /*path to save results to UNUSED*/
    
    std::vector<std::unique_ptr<Cell>> cells; /*each cell in the environment*/
    std::vector<std::unique_ptr<Robot>> robots; /*each robot in the environment*/
    std::set<Cell*> transients; /*each transient obstacle in the environment*/
    std::vector<Cell*> freeCells;
    std::map<Robot*, int> robotsAtGoal; /*each robot at goal with it's associated time of arrival*/

    Robot* selectedRobot = nullptr; /*robot selected by the mouse*/
};

class GridRenderer{
public:

    GridRenderer(int width, int height, int rows, int cols) : cellW(width), cellH(height), rows(rows), cols(cols){
        zoom = 1.0f;
        offsetX = 0.0f;
        offsetY = 0.0f;
        baseH = width;
        baseW = height;
    }

    Color colorFromID(int, int);
    void setZoom(float);
    void pan(float, float);
    void cacheRobotColors(int nRobots);
    void draw(Env&, float t);

    float getZoom(){return zoom;}

private:

    int cellW, cellH;
    int baseW, baseH;
    float offsetX, offsetY;
    int rows, cols;
    float zoom;

    std::unordered_map<int,Color> robotColors;
};
