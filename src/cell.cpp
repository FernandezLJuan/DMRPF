#include "cell.h"
#include "environment.h"

void Cell::setGoal(){
    //mark the cell as a goal
    goal = true;
}

void Cell::removeGoal(){
    //remove goal property from cell
    goal = false;
}

int Cell::getID(){
    //return the cell's id
    return this->id;
}

void Cell::setObjID(Robot* newID){
    //set the objID to a robot occupying the cell
    if(!newID){
        return;
    }
    this->objectID = newID;
}

Robot* Cell::getObjID(){
    //return the robot in the cell
    return this->objectID;
}

std::array<int, 2> Cell::getPos(){
    //return the cell's position
    std::array<int, 2> p{x,y};
    return p;
}

const std::vector<Cell*>& Cell::getNeighbors(){
    //return the neighbors in the adjacency list
    return this->neighbors;
}

void Cell::addNeighbor(Cell* newNeighbor){
    //adds a new neighbor to the vector if it is not an obstacle
    if(!newNeighbor->isObstacle())
        this->neighbors.push_back(newNeighbor);
}

void Cell::updateNeighbors(std::unordered_map<int,std::unordered_map<int,float>>& adjMatrix, Env& env){
    //update adjacency list of the cell based on the adjacency matrix of the graph
    neighbors.clear();
    neighbors.reserve(8);

    //iterate through the row in the adjacency matrix and set each cell with a connection as a neighbor
    for(const auto& pair : adjMatrix[this->id]){
        Cell* neighbor = env.getCellByID(pair.first);
        if(neighbor != nullptr && !neighbor->isObstacle()){
            neighbors.push_back(neighbor);
        }
    }
}

void Cell::removeNeighbor(const Cell* c){
    //remove a neighbor from the adjacency list
    auto it = std::find(neighbors.begin(), neighbors.end(), c);

    if(it!=neighbors.end())
        neighbors.erase(it);
}

bool Cell::isObstacle(){
    //return true if the cell is an obstacle
    return obstacle;
}

bool Cell::isCellTransient(){
    //return true if the cell is a transient obstacle
    return isTransient;
}

bool Cell::isGoal(){
    //return true if the cell is a goal
    return goal;
}

void Cell::setTransient(){
    /*mark a cell as a transient obstacle*/
    isTransient = true;
}

void Cell::setObstacle(){
    //mark a cell as an obstacle
    obstacle = true;
}

void Cell::removeObstacle(){
    //remove obstacle property from cell
    obstacle = false;
}

void Cell::removeTransient(){
    //remove dynamic obstacle property from cell
    isTransient = false;
}

void Cell::logType(){
    //print out the type of the cell
    if(obstacle){
        std::cout<<"OBSTACLE\n";
    }
    if(isTransient){
        std::cout<<"DYNAMIC OBSTACLE\n";
    }
    if(goal){
        std::cout<<"DYNAMIC\n";
    }

    if(!isObstacle() && !isGoal() && !isCellTransient()){
        std::cout<<"FREE CELL\n";
    }
}

std::ostream& operator<<(std::ostream& str, const Cell& c){
    str<<"{"<<c.x<<","<<c.y<<"}";
        
    return str;
}