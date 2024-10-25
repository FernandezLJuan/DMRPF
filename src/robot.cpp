#include "robot.h"

int Robot::currentID = 0;

void Robot::takeAction(){
    /*robot will either stop for this step or follow the path to it's next node*/

    /*generate random number between 0 and 1*/
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    double willIStop = dis(gen);
    this->fetchNeighborInfo(); /*get info about neighbors and solve conflicts*/
    this->findLeader();

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

    if (currentCell == goal) {
        path.clear(); // Vacía el path
        return;
    }

    if(newPos==nullptr){
        return;
    }

    std::array<int, 2> eDims = environment->getDims();

    if(!(environment->moveRobot(this, newPos)<0) && this->moving){
        currentCell = environment->getCellByID(newPos->getID());
        updateDetectionArea(); /*update detection area after moving*/      

        if(!path.empty()){
            lastCell = *path.begin();
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

        /*if there isn't a robot in the neighbor, use it as give way node*/
        if(neighbor->getObjID()!=nullptr){
            giveWayNode = neighbor;
        }
        else if(neighbor != currentCell){
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

void Robot::fetchNeighborInfo(){
    /*check for robots in the detection range of the robot*/

    std::shared_ptr<Cell> rStep; /*immediate next node of neighbor robot*/
    std::vector<std::shared_ptr<Cell>> remainingNodes; /*remaining nodes of neighbor robot rn*/
    
    for(auto& n : neighbors){
            if(n){
                switch(environment->detectConflict(this, n)){
                    case 0:
                        std::cout<<"No conflict detected"<<std::endl;
                        break;
                    case 1:
                        std::cout<<"OPPOSITE CONFLICT"<<std::endl;
                        break;
                    case 2:
                        std::cout<<"INTERSECTION CONFLICT"<<std::endl;
                        break;
                }
            }
    }
}

void Robot::findLeader(){
    std::shared_ptr<Cell> nextCell = this->step();

    if(nextCell){
        Robot* rn = nextCell->getObjID();
        if(rn){
            if(rn->step() != currentCell && !rn->atGoal()){/*si el robot que hay en la siguiente celda no se quiere mover a la mia */
                this->leader = rn;
            }
        }
    }

    Robot* rn = lastCell->getObjID();
    if(rn){
        if(rn->step() == currentCell){
            this->follower = rn;
            this->numberFollowers++;
        }
        else{
            if(follower){
                if(rn->getID() == follower->getID()){
                    this->follower = nullptr;
                    this->numberFollowers--;
                }
            }
        }
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

int Robot::getID(){
    return id;
}

int Robot::getNFollowers(){
    return numberFollowers;
}

std::shared_ptr<Cell> Robot::getCurrentCell(){return this->currentCell;}

std::shared_ptr<Cell> Robot::getGoal(){return this->goal;}
