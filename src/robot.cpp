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
        if(!path.empty()){
            this->move(path.top());
        }
    }
}

void Robot::reconstructPath(std::unordered_map<std::shared_ptr<Cell>, std::shared_ptr<Cell>> recordedPath, std::shared_ptr<Cell> cell){

    auto it = recordedPath.find(cell);
    while(it != recordedPath.end()){
        path.push(cell);
        cell = it->second;

        it = recordedPath.find(cell);
    }

}

void Robot::generatePath() {
    if (goal == nullptr) {
        return;
    }

    std::cout << "Generating path" << std::endl;

    Vector2 start = {posX, posY};
    std::shared_ptr<Cell> tmp = currentCell;
    std::unordered_map<std::shared_ptr<Cell>, int> gScore;
    std::unordered_map<std::shared_ptr<Cell>, int> fScore;

    std::unordered_map<std::shared_ptr<Cell>, std::shared_ptr<Cell>> recordedPath;
    
    auto compareFScore = [&](const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>& b) {
        return fScore[a] < fScore[b];
    };

    std::multiset<std::shared_ptr<Cell>, decltype(compareFScore)> openSet(compareFScore);
    
    openSet.insert(currentCell);

    std::vector<std::shared_ptr<Cell>> cells = environment->getCells();
    
    for (auto c : cells) {
        gScore[c] = 9999999;
        fScore[c] = 9999999;
    }

    gScore[tmp] = 0;
    fScore[tmp] = environment->cellDistance(*tmp, *goal);

    while (!openSet.empty()) {
        tmp = *openSet.begin();
        if (tmp == goal) {
            std::cout << "Reached the goal at " << tmp->getID() << " reconstructing path!!" << std::endl;
            reconstructPath(recordedPath, tmp);
            return;
        }

        openSet.erase(openSet.begin());

        std::vector<std::shared_ptr<Cell>> tmpNeighbors = tmp->getNeighbors();

        for (auto& n : tmpNeighbors) {
            int tentGScore = gScore[tmp] + environment->cellDistance(*tmp, *n);
            if (tentGScore < gScore[n]) {
                recordedPath[n] = tmp;
                gScore[n] = tentGScore;
                fScore[n] = tentGScore + environment->cellDistance(*n, *goal);
                openSet.insert(n);
            }
        }
    }
}


void Robot::move(std::shared_ptr<Cell> newPos){ /*change the position of the robot*/

    if (currentCell == goal) {
        std::cout << "Robot reached the goal!" << std::endl;
        path = std::stack<std::shared_ptr<Cell>>(); // Vacía el path
        return;
    }

    if(newPos==nullptr){
        return;
    }

    std::cout<<"Moving robot to "<<newPos->getID()<<std::endl;

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
    return path.top();
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
    this->generatePath();
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
