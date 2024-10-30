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

std::vector<std::shared_ptr<Cell>> Cell::getNeighbors(){
    return this->neighbors;
}

void Cell::addNeighbor(std::shared_ptr<Cell> newNeighbor){
    /*if the cell to be added as a neighbor is an obstacle, do not add it*/
    if(!newNeighbor->isObstacle())
        this->neighbors.push_back(newNeighbor);
}

void Cell::updateNeighbors(std::vector<std::vector<int>>& adjMatrix, Env& env){
    /*uses adjacency matrix of the graph to update the neighbors vector*/

    std::shared_ptr<Cell> tempCell;
    std::vector<std::shared_ptr<Cell>> updatedNeighbors;

    /*iterate through the row of the current cell and use connections to update neighbors*/
    for(size_t j = 0; j<adjMatrix[this->id].size();j++){
        /*check if the current cell is in the neighbors vector*/
        if(adjMatrix[this->id][j]!=0){
            auto neighbor = env.getCellByID(j);
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
