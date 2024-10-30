#include "robot.h"

int Robot::currentID = 0;

void Robot::takeAction(){
    /*robot will either stop for this step or follow the path to it's next node*/

    /*generate random number between 0 and 1*/
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    double willIStop = dis(gen);

    if(willIStop<waitProbability){
        this->stopRobot();
    }
    else{
        /*move the robot to the next cell*/
        if(!moving)
            this->resumeRobot();

        if(!path.empty()){
            this->move(path.front());
        }
    }
}

void Robot::reconstructPath(std::unordered_map<std::shared_ptr<Cell>, std::shared_ptr<Cell>> recordedPath, std::shared_ptr<Cell> cell){

    auto it = recordedPath.find(cell);
    while(it != recordedPath.end()){
        path.push_back(cell);
        cell = it->second;

        it = recordedPath.find(cell);
    }

    std::reverse(path.begin(), path.end());
}

void Robot::generatePath() {

    this->path.clear();

    if (goal == nullptr) {
        return;
    }

    std::shared_ptr<Cell> tmp = currentCell;
    std::unordered_map<std::shared_ptr<Cell>, int> gScore;
    std::unordered_map<std::shared_ptr<Cell>, int> fScore;
    std::unordered_map<std::shared_ptr<Cell>, std::shared_ptr<Cell>> recordedPath;

    //use unordered set to avoid duplicates in openSet
    std::unordered_set<std::shared_ptr<Cell>> openSet;
    auto compareFScore = [&](const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>& b) {
        return fScore[a] < fScore[b];
    };

    auto getLowestFScore = [&](const std::unordered_set<std::shared_ptr<Cell>>& set) {
        return *std::min_element(set.begin(), set.end(), compareFScore);
    };

    openSet.insert(currentCell);

    //only initialize gscore and fscore for the current cell (start position)
    gScore[currentCell] = 0;
    fScore[currentCell] = environment->cellDistance(*currentCell, *goal);

    while (!openSet.empty()) {
        tmp = getLowestFScore(openSet);
        if (tmp == goal) {
            reconstructPath(recordedPath, tmp);
            return;
        }

        openSet.erase(tmp);

        std::vector<std::shared_ptr<Cell>> tmpNeighbors = tmp->getNeighbors();
        for (auto& n : tmpNeighbors) {
            int tentativeGScore = gScore[tmp] + environment->cellDistance(*tmp, *n);

            if (gScore.find(n) == gScore.end() || tentativeGScore < gScore[n]) {
                recordedPath[n] = tmp;
                gScore[n] = tentativeGScore;
                fScore[n] = tentativeGScore + environment->cellDistance(*n, *goal);

                if (openSet.find(n) == openSet.end()) {
                    openSet.insert(n);
                }
            }
        }
    }
}

void Robot::move(std::shared_ptr<Cell> newPos){ /*change the position of the robot*/

    if(newPos==nullptr){
        return;
    }

    std::array<int, 2> eDims = environment->getDims();

    if(!(environment->moveRobot(this, newPos)<0) && this->moving){
        currentCell = environment->getCellByID(newPos->getID());
        updateDetectionArea(); /*update detection area after moving*/      

        if(!path.empty()){
            lastCell = *path.begin();
            pathHistory.push_back(currentCell);
            path.erase(path.begin());
        }
    }    
}

void Robot::updateDetectionArea() {
    /*use BFS to get the communication range of the robot*/
    detectionArea.clear();
    neighbors.clear();

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

        if(neighbor->getObjID()){
            neighbors.push_back(neighbor->getObjID());
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

            if(n->getObjID() && n!=currentCell){
                neighbors.push_back(n->getObjID());
            }
        }
    }
}

void Robot::getGiveWayNode(){
    /*scan surrounding cells of the current cell*/
    for (const auto& neighbor : currentCell->getNeighbors()) {
        /*if there isn't a robot in the neighbor, use it as give way node*/
        if(neighbor->getObjID()==nullptr && (std::find(path.begin(), path.end(), neighbor)==path.end())){
            giveWayNode = neighbor;
            break;
        }
    }
}

void Robot::fetchNeighborInfo(){
    /*check for robots in the detection range of the robot*/

    std::shared_ptr<Cell> rStep; /*immediate next node of neighbor robot*/
    std::vector<std::shared_ptr<Cell>> remainingNodes; /*remaining nodes of neighbor robot rn*/
    
    for(auto n : neighbors){
            if(n){
                switch(environment->detectConflict(*this, *n)){
                    case 0:
                        if(follower != nullptr){
                            if(follower->getPath().size() > path.size()){
                                this->giveWay();
                            }
                        }
                        break;
                    case 1:
                        //std::cout<<"OPPOSITE CONFLICT BETWEEN "<<id<<" AND "<<n->getID()<<std::endl;
                        //solveIntersectionConflict(n);
                        break;
                    case 2:
                        //std::cout<<"INTERSECTION CONFLICT BETWEEN "<<id<<" AND "<<n->getID()<<std::endl;
                        solveIntersectionConflict(*n);
                        break;
                }
            }
    }
}

void Robot::findLeader(){
    numberFollowers = 0;
    std::shared_ptr<Cell> nextCell = this->step();

    auto isInHistory = [this](std::shared_ptr<Cell> c)->bool{
        return std::find(this->pathHistory.begin(), this->pathHistory.end(), c) != this->pathHistory.end();
    };

    for(auto& n : neighbors){
        if(n){
            if(isInHistory(n->step()) && isInHistory(n->getCurrentCell())){ /*else, check if the robot's next cell is in my history of cells, then set as follower*/
                this->follower = n;
                numberFollowers++;
            }

            if(n == follower){
                Robot* tmp = follower;

                /*if the follower has a follower*/
                while(tmp->follower){
                    numberFollowers++;
                    tmp = tmp->follower;
                }

                /*if the follower has stopped being in the path history*/
                if(!isInHistory(n->getCurrentCell())){
                    follower = nullptr;
                }
            }
        }
    }
}

void Robot::solveIntersectionConflict(Robot& rn){
    /*check for each priority rule and solve the conflict accordingly*/

    std::cout<<"Checking priority rule number 1"<<std::endl;
    /*move the robot occupying the intersection node*/
    if(rn.getCurrentCell() == this->step()){
        this->giveWay();
        return;
    }
    else if(this->getCurrentCell() == rn.step()){
        rn.giveWay();
        return;
    }
    
    std::cout<<"Checking priority rule number 2"<<std::endl;
    /*if a robot is giving way, it has priority*/
    if(rn.isGivingWay()){
        rn.takeAction();
        return;
    }
    else if(this->isGivingWay()){
        this->takeAction();
        return;
    }

    std::cout<<"Checking priority rule number 3"<<std::endl;
    /*the robot with the highest nFollowers has priority*/
    if(rn.getNFollowers()>this->numberFollowers){
        rn.takeAction();
        return;
    }
    else if(this->numberFollowers<rn.getNFollowers()){
        this->takeAction();
        return;
    }

    std::cout<<"Checking priority rule number 4"<<std::endl;
    

    std::cout<<"Checking priority rule number 5"<<std::endl;

    std::cout<<"Checking priority rule number 6"<<std::endl;
}

void Robot::giveWay(){
    getGiveWayNode();
    if(giveWayNode){
        path.insert(path.begin(), lastCell);
        path.insert(path.begin(), giveWayNode);

        takeAction();
    }
}

/*return next step of the robot*/
std::shared_ptr<Cell> Robot::step(){
    return path.front();
}

std::vector<std::shared_ptr<Cell>> Robot::getPath(){
    return this->path;
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

bool Robot::atGoal(){
    return currentCell == goal;
}

void Robot::setGoal(std::shared_ptr<Cell> goalPos){
    goal = goalPos;
    if(goalPos != currentCell){
        environment->addGoal(goalPos->getID());
    }
    this->generatePath();
}

void Robot::removeGoal(){
    if(!goal){
        return;
    }
    environment->removeGoal(goal->getID());
    goal = nullptr;
    this->generatePath();
}

bool Robot::isMoving(){
    return moving;
}

bool Robot::isGivingWay(){
    return currentCell == giveWayNode;
}

int Robot::getID(){
    return id;
}

int Robot::getNFollowers(){
    return numberFollowers;
}

std::shared_ptr<Cell> Robot::getCurrentCell(){return this->currentCell;}

std::shared_ptr<Cell> Robot::getGoal(){return this->goal;}
