#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <memory>

class Env; /*forward declaration of class Env so C++ knows it exists*/
class Robot; /*forward declaration of class Robot so C++ knows it exists*/

enum class cellType {CELL_FREE, CELL_OBSTACLE, CELL_ROBOT, CELL_GOAL};

class Cell{
public:

    Cell(int id, int x, int y) : id(id), x(x), y(y){
        objectID = nullptr;
        type = cellType::CELL_FREE;
        isTransient = false;
    };

    /*NEIGHBOR RELATED*/
    void addNeighbor(Cell*);
    void updateNeighbors(std::vector<std::vector<int>>&,Env& env); /*update neighbors based on adjacency matrix of environment, might replace addNeighbor*/

    /*get info about type*/
    bool isObstacle();
    bool isGoal();
    bool isCellTransient();

    /*GETTERS AND SETTERS*/
    int getID();
    const std::vector<Cell*>& getNeighbors(); /*get the neighbors in the graph*/
    std::array<int, 2> getPos(); /*get x, y position on the grid*/

    void setType(cellType);
    void setTransient();
    void logType();
    void logPos();

    Robot* getObjID();
    void setObjID(Robot*);

private:

    int id;
    int x, y;
    
    bool isTransient;

    std::vector<Cell*> neighbors; /*neighbors to the cell in the grid graph*/
    Robot* objectID; /*object inside the cell, NULL if none*/

    cellType type; /*type of the cell*/

};
