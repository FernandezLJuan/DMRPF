#include "robot.h"

void Robot::takeAction(){
    /*robot will either stop for this step or follow the path to it's next node*/

    /*generate random number between 0 and 1*/
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    double willIStop = dis(gen);

    if(willIStop<waitProbability){
        //std::cout<<"I'm "<<id<<" and I'm taking a COFFEE BREAK"<<std::endl;
        this->stopRobot();
    }
    else{
        /*move the robot to the next cell*/
        this->resumeRobot();
        if(!path.empty()) {
            this->move(path.front());
        }
    }
}

void Robot::generatePath(){

    if(goal==nullptr){
        return;
    }

    path = {};

    Vector2 start = {posX, posY};

    /*TODO: ADD PATH FINDING ALGORITHM TO GENERATE PATHS FROM POSITION TO GOAL*/

    std::array<int, 2> eDims = environment->getDims();

    int startID = start.x * eDims[1] + start.y;

    for(int i = startID; i<=this->goal->getID(); i++){
        this->path.push(environment->getCellByID(i));
    }
    if(path.back()!=goal){
        path.push(goal);
    }

    path.pop(); /*eliminate current position*/
}

void Robot::move(std::shared_ptr<Cell> newPos){ /*change the position of the robot*/

    if (currentCell == goal) {
        std::cout << "Robot reached the goal!" << std::endl;
        path = std::queue<std::shared_ptr<Cell>>(); // Vacía el path
        return;
    }

    if(newPos==nullptr){
        return;
    }

    std::array<int, 2> eDims = environment->getDims();

    if(!(environment->moveRobot(this, newPos)<0) && this->moving){
        currentCell = environment->getCellByID(newPos->getID());
        updateDetectionArea(); /*update detection area after moving*/

        posX = newPos->getPos()[0];
        posY = newPos->getPos()[1];        

        if(!path.empty()){
            path.pop();
        }
    }    
}

void Robot::updateDetectionArea() {
    /*use BFS to get the communication range of the robot*/
    detectionArea.clear();

    std::unordered_set<std::shared_ptr<Cell>> visited{currentCell}; /*mark current cell as visited so it is not included in the range*/
    std::deque<std::pair<std::shared_ptr<Cell>, int>> toVisit; /*cells to visit and determine distance*/

    /*visit the neighbors of the current cell and determine the initial distance, these will be used for BFS*/
    for (const auto& neighbor : currentCell->getNeighbors()) {
        int initialCost = environment->connectionCost(*currentCell, *neighbor);
        if (initialCost <= detectionRadius) {
            toVisit.push_back({neighbor, initialCost});
            detectionArea.push_back(neighbor);
            visited.insert(neighbor);  /*mark neighbor as visited*/
        }

        /*if there isn't a robot in the neighbor, use it as give way node*/
        if(neighbor->getObjID()!=nullptr){
            giveWayNode = neighbor;
        }
    }

    while(!toVisit.empty()){
        auto [curr, accumulatedDistance] = toVisit.front();
        toVisit.pop_front(); /*pop first element*/

        for(auto n : curr->getNeighbors()){ /*visit neighoring cells*/
        
            /*calculate new distance using the cost of traversal and the accumulated distance*/
            int conCost = environment->connectionCost(*curr, *n);
            int newDistance = conCost + accumulatedDistance;

            /*if the new distance is less or equal than detection radius and the neighbor hasn't been visited, add it to the area*/
            if(newDistance<=detectionRadius && visited.find(n) == visited.end()){
                toVisit.push_back({n, newDistance});
                detectionArea.push_back(n);
                visited.insert(n);
            }
        }
    }
}

void Robot::anyoneThere(){
    /*check for robots in the detection range of the robot*/

    for(auto& cell:detectionArea){
        Robot* john;
        if((john = cell->getObjID())!=nullptr){
            /*if the robot's next cell is our current cell, one follower*/
            if(john->step()==currentCell){
                neighborsRequestingNode.push_back(john);
                numberFollowers++;
            }else{
                /*if we had followers and the robot's next cell is not our cell, decrease followers*/
                if(john->step() != currentCell && numberFollowers>0){
                    numberFollowers--;
                    neighborsRequestingNode.pop_back();
                }
            }
        }
    }
}

/*return next step of the robot*/
std::shared_ptr<Cell> Robot::step(){
    return path.front();
}

std::vector<std::shared_ptr<Cell>> Robot::getArea(){
    return this->detectionArea;
}

void Robot::stopRobot(){
    this->moving = false;
}

void Robot::resumeRobot(){
    this->moving = true;
}

void Robot::logPos(){
    std::cout<<"("<<this->posX<<","<<this->posY<<")"<<std::endl;
}

void Robot::setGoal(Vector2 goalPos){
    goal = environment->getCellByPos(goalPos.x, goalPos.y);
    if(goal != currentCell){
        environment->addGoal(goal->getID());
    }
    this->generatePath();
}

void Robot::removeGoal(){
    environment->removeGoal(goal->getID());
    goal = nullptr;
}

bool Robot::isMoving(){
    return moving;
}

int Robot::getID(){
    return id;
}

std::shared_ptr<Cell> Robot::getCurrentCell(){return this->currentCell;}

std::shared_ptr<Cell> Robot::getGoal(){return this->goal;}

std::array<int, 2> Robot::getPos(){
    std::array<int,2> p{static_cast<int>(posX), static_cast<int>(posY)};
    return p;
}
