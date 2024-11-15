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
    std::cout<<"Size of neighbors "<<this->neighbors.size()<<std::endl;
    return this->neighbors;
}

void Cell::addNeighbor(Cell* newNeighbor){
    /*if the cell to be added as a neighbor is an obstacle, do not add it*/
    if(!newNeighbor->isObstacle())
        this->neighbors.push_back(newNeighbor);
}

void Cell::updateNeighbors(std::vector<std::vector<int>>& adjMatrix, Env& env){
    /*uses adjacency matrix of the graph to update the neighbors vector*/

    Cell* tempCell;
    std::vector<Cell*> updatedNeighbors;

    /*iterate through the row of the current cell and use connections to update neighbors*/
    for(size_t j = 0; j<adjMatrix[this->id].size();j++){
        /*check if the current cell is in the neighbors vector*/
        if(adjMatrix[this->id][j]!=0){
            Cell* neighbor = env.getCellByID(j);
            if(!neighbor->isObstacle()){
                updatedNeighbors.push_back(neighbor);
            }
        }
    }

    /*use move to update neighbors list, more efficient than previous code*/
    this->neighbors = std::move(updatedNeighbors);
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
