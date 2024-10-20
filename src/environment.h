#pragma once

//TODO: MOVE RECTANGLE TO DRAWER CLASS AND SET DIMENSIONS BY env->cellDims()

#include <array>
#include <ctime>
#include <memory>
#include <iostream>
#include <random>
#include <vector>
#include <raylib.h>
#include <set>
#include <algorithm>
#include "cell.h"

class Robot;

typedef std::mt19937 MyRNG;

class Env{

public:

    Env(int cellW, int cellH, int posX, int posY, int r, int c, int n_robots)
    : cellWidth(cellW), cellHeight(cellH), rows(r), cols(c),
      originX(posX), originY(posY), nRobots(n_robots),obstacleDist(0, 1)

        {
            envWidth = cellW * cols;
            envHeight = cellH * rows;

            rng.seed(static_cast<uint32_t>(std::time(nullptr)));
            adjMatrix= std::vector<std::vector<int> >(rows*cols, std::vector<int>(rows*cols, 0));

            cells.reserve(rows*cols);

            createGrid();
        }

    void pauseSim();
    void resumeSim();
    void onClick(int);
    int cellDistance(Cell&, Cell&);

    /*ENVIRONMENT MODIFICATION*/
    void updateEnvironment(); /*updates robot positions, cell neighbors and spawns random obstacles*/
    void addObstacle(int);
    void removeObstacle(int);
    void addEdge(int, int, int); /*add connection between two cells in the graph*/
    void removeEdge(int, int); /*remove connection between two cells in the graph*/

    /*ROBOT MANAGEMENT FUNCTIONS*/
    int placeRobot(Robot*); /*place robot in the environment*/
    int moveRobot(Robot* , std::shared_ptr<Cell>); /*update robot position*/
    int detectConflict(Robot*, Robot*); /*return the type of conflict it has detected*/
    void remakePaths();
    void addGoal(int);
    void removeGoal(int);
    void randomizeRobots(); /*place robots and goals in random free cells*/

    /*GETTERS*/
    std::array<int, 2> getDims();
    std::array<int, 2> cellDims();
    std::array<int, 2> origin();
    std::shared_ptr<Cell> getCellByPos(int , int );
    std::shared_ptr<Cell> getCellByID(int );
    std::vector<std::shared_ptr<Cell>> getCells();
    std::vector<std::shared_ptr<Robot>> getRobots();

    /*other environment info*/
    bool isRunning();
    bool isConnected(Cell&, Cell&); /*are the two cells connected?*/
    int connectionCost(Cell&, Cell&); /*get the cost of the edge between to cells*/
    void logAdj(); /*show the adjacency matrix on screen, for debug purposes*/


private:

    void createGrid(); /*fills the cell vector with rectangles*/
    void connectCells();

    std::vector<std::vector<int>> adjMatrix; /*SHOULD ALWAYS BE UPDATED BEFORE CELL NEIGHBORS*/

    int envWidth, envHeight;
    const float cellWidth, cellHeight;
    const int rows, cols;
    const float obstacleProbability = 0.0f; /*probability of a cell being an obstacle*/
    const float transientProbability = 0.3f;

    const int originX, originY;
    bool running = false;
    const int nRobots;

    MyRNG rng;
    std::uniform_real_distribution<float> obstacleDist; /*distribution of obstacle probability*/
    std::uniform_int_distribution<> cellDist = std::uniform_int_distribution<>(0,this->rows*this->cols); /*distribution for random cell obstacles*/

    std::vector<std::shared_ptr<Cell>> cells; /*each cell on the environment*/
    std::vector<std::shared_ptr<Robot>> robots; /*each robot on the environment*/
    Robot* selectedRobot = nullptr;

    std::set<std::pair<Robot*, Robot*>> checkedConflicts; 
    /*pairs of robots for which conflicts have been checked, this way we don't check the same robots over and over*/

};

class GridRenderer{

public:

    GridRenderer(int width, int height, int rows, int cols) : envWidth(width), envHeight(height), rows(rows), cols(cols){}

    Color colorFromID(int id){
        /*encode color using robot (or cell) IDs*/

        /*multiply the ID with prime numbers to create a more uniform distribution and minimize repeated colors*/
        unsigned char r = (id * 57) % 256;
        unsigned char g = (id * 37) % 256;
        unsigned char b = (id * 97) % 256;

        return Color{r,g,b,255}; /*keep opacity at max value (255)*/

    }

    void draw(Env&, float t);

private:

    int envWidth, envHeight;
    int rows, cols;

};
