#include "cell.h"
#include "environment.h"

void Cell::setType(cellType newType){
    this->type = newType;
}

int Cell::getID(){return this->id;}

void Cell::setObjID(Robot* newID){
    this->objectID = newID;
}

Robot* Cell::getObjID(){return this->objectID;}
std::array<int, 2> Cell::getPos(){
    std::array<int, 2> p{x,y};
    return p;
}

const std::vector<Cell*>& Cell::getNeighbors(){
    return this->neighbors;
}

void Cell::addNeighbor(Cell* newNeighbor){
    /*if the cell to be added as a neighbor is an obstacle, do not add it*/
    if(!newNeighbor->isObstacle())
        this->neighbors.push_back(newNeighbor);
}

void Cell::updateNeighbors(std::vector<std::vector<int>>& adjMatrix,Env& env) {
    /* clear the current neighbors vector */
    neighbors.clear();
    neighbors.reserve(8); /* reserve enough space to avoid multiple reallocations */

    /* iterate through the row of the adjacency matrix corresponding to this cell */
    for (size_t j = 0; j < adjMatrix[this->id].size(); ++j) {
        if (adjMatrix[this->id][j] != 0) { /* check if there is a connection to this cell */
            Cell* neighbor = env.getCellByID(j);
            if (neighbor != nullptr && !neighbor->isObstacle()) { /* check for null pointer and obstacle */
                neighbors.push_back(neighbor);
            }
        }
    }
}

bool Cell::isObstacle(){
    if(this->type == cellType::CELL_OBSTACLE){
        return true;
    }
    return false;
}

void Cell::setTransient(){
    /*mark a cell as a transient obstacle*/
    if(isObstacle())
        isTransient = true;
}

void Cell::removeTransient(){
    isTransient = false;
}

bool Cell::isCellTransient(){
    return isTransient;
}

bool Cell::isGoal(){
    return this->type==cellType::CELL_GOAL;
}

void Cell::logType(){

    std::string strType = "";

    switch ((type))
    {
    case cellType::CELL_FREE:
        strType = "FREE";
        break;

    case cellType::CELL_OBSTACLE:
        strType = "OBSTACLE";
        break;

    case cellType::CELL_ROBOT:
        strType = "ROBOT";
        break;

    case cellType::CELL_GOAL:
        strType = "GOAL";
        break;
    
    default:
        break;
    }

    std::cout<<strType<<std::endl;
}

void Cell::logPos(){
    std::cout<<"("<<x<<","<<y<<")";
}
