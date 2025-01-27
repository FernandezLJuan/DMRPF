#pragma once

#include <vector>
#include <string>
#include <ostream>
#include <memory>
#include <unordered_map>

class Env; /*forward declaration of class Env so C++ knows it exists*/
class Robot; /*forward declaration of class Robot so C++ knows it exists*/

class Cell{
public:
    Cell(int id, int x, int y) : id(id), x(x), y(y){

        //by default the cell doesn't have a robot and is free without any properties
        objectID = nullptr;
        isTransient = false;
        obstacle = false;
        goal = false;
    };

    /*NEIGHBOR RELATED*/
    void addNeighbor(Cell*);
    void updateNeighbors(std::unordered_map<int,std::unordered_map<int,float>>&,Env& env); /*update neighbors based on adjacency matrix of environment, might replace addNeighbor*/
    void removeNeighbor(const Cell*);

    /*get info about type*/
    bool isObstacle(); //is the cell an obstacle
    bool isGoal(); //is the cell a goal
    bool isCellTransient(); //is the cell a dynamic obstacle

    /*GETTERS AND SETTERS*/
    int getID(); //return id of the cell
    const std::vector<Cell*>& getNeighbors(); //get the neighbors in the graph
    std::array<int, 2> getPos(); //get x, y position on the grid

    void setObstacle(); //mark the cell as an obstacle
    void setTransient(); //mark the cell as a dynamic obstacle
    void setGoal(); //mark the cell as a goal
    void removeObstacle(); //remove obstacle property from cell
    void removeTransient(); //remove dynamic obstacle property from cell
    void removeGoal(); //remove goal property from cell
    void logType(); 

    Robot* getObjID(); //return robot occupying the cell
    void setObjID(Robot*); //set reference to robot occupying the cell

    friend std::ostream& operator<<(std::ostream& str, const Cell& c);

private:

    int id; //identifier of the cell
    int x, y; //position of the cell in the grid
    
    bool isTransient; //dynamic obstacle property
    bool obstacle; //static obstacle property
    bool goal; //goal property

    std::vector<Cell*> neighbors; /*neighbors to the cell in the grid graph*/
    Robot* objectID; /*object inside the cell, NULL if none*/
};
