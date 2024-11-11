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
        /*resume movement if the leader is not waiting*/
        if(noConflictDetected){
            if(!path.empty()){
                leader = path.front()->getObjID();

                if(leader){
                    if(!leader->isMoving()){
                        this->stopRobot();
                    }
                    else if(leader->step() == currentCell){
                        this->giveWay();
                    }
                }
            }
        }
        else{
            plannedAction = 1;
        }
    }

    if(plannedAction){
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
        std::cout<<"sorry, null position"<<std::endl;
        return;
    }

    std::array<int, 2> eDims = environment->getDims();

    std::vector<std::shared_ptr<Cell>> something = currentCell->getNeighbors();

    if(std::find(something.begin(), something.end(), newPos) == something.end()){
        std::cout<<id<<" [CANNOT MOVE TO NEXT CELL] "<<std::endl;
    }

    if(!(environment->moveRobot(this, newPos)<0)){
        currentCell = environment->getCellByID(newPos->getID());

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

void Robot::getNeighbors(){
    neighbors.clear();

    for(auto& c : detectionArea){
        if(c->getObjID()){
            neighbors.push_back(c->getObjID());
        }
    }
}

bool Robot::findGiveWayNode(){

    /*scan surrounding cells of the current cell*/
    for (const auto& cellNeighbor : currentCell->getNeighbors()) {
        /*if there isn't a robot in the neighbor, use it as give way node*/

        for(auto& rn : neighbors){
            if(cellNeighbor != rn->step()){    
                if(cellNeighbor->getObjID()==nullptr && !isInPath(cellNeighbor)){
                    giveWayNode = cellNeighbor;
                    break;
                }
            }
        }
    }

    /*if a giveWayNode is found, return true*/
    return (giveWayNode) ? true : false;
}

void Robot::fetchNeighborInfo(){
    /*check for robots in the detection range of the robot*/

    if(neighbors.empty()){
        noConflictDetected=true;
        return;
    }

    noConflictDetected = false;

    std::shared_ptr<Cell> rStep; /*immediate next node of neighbor robot*/
    std::vector<std::shared_ptr<Cell>> remainingNodes; /*remaining nodes of neighbor robot rn*/
    
    for(auto n : neighbors){

        if(!n){
            continue;
        }
        switch(environment->detectConflict(*this, *n)){
            case 0:
                /*NO CONFLICT IS DETECTED*/
                if(follower){
                    noConflictDetected = true;
                    plannedAction = 1;

                    /*if our follower has a longer path than ours, move out of the way*/
                    if(follower->getPath().size() > path.size()){
                        this->giveWay();
                    }
                }
                break;
            case 1:
                /*OPPOSITE CONFLICT*/
                solveOppositeConflict(n);
                break;
            case 2:
                /*INTERSECTION CONFLICT*/
                std::cout<<"[INTERSECTION CONFLICT BETWEEN] <"<<id<<" AND "<<n->getID()<<">"<<std::endl;
                solveIntersectionConflict(n);
                break;
        }
    }
}

void Robot::findFollowers(){
    neighborsRequestingNode.clear(); /*reset neighbors requesting node*/
    numberFollowers = 0; /*assume there are no followers before searching for them*/
    follower = nullptr;

    std::shared_ptr<Cell> nextCell = this->step();

    auto isInHistory = [this](std::shared_ptr<Cell> c)->bool{
        return std::find(this->pathHistory.begin(), this->pathHistory.end(), c) != this->pathHistory.end();
    };

    /*for any neighboring robots*/
    for(auto& n : neighbors){
        if(!n){
            continue;
        }

        if(n->step() == currentCell){ 
            /*check if the robot's next cell is in my history of cells, then set as follower*/
            if(!isInFollowerChain(n)){
                /*if the robot is already in the chain of followers, don't set it as my follower*/
                this->follower = n;
                numberFollowers++;
            }
        }

        if(n == follower){
            Robot* tmp = follower;
            std::unordered_set<Robot*> visited;

            while(tmp){
                if(visited.find(tmp) != visited.end()){
                    break;
                }
                numberFollowers++;
                visited.insert(tmp);
                tmp = tmp->follower;
            }
        }

        auto it = std::find(neighborsRequestingNode.begin(), neighborsRequestingNode.end(), n);

        /*check if the robot's next cell or n(t+2) cell are my node*/
        if(n->step() == currentCell || (n->getPath().size()>2 && n->getPath()[1] == currentCell)){
            neighborsRequestingNode.push_back(n);
        }
    }
}

bool Robot::isInFollowerChain(Robot* candidate){
    Robot* current = this;
    std::unordered_set<Robot*> visited;

    /*iterate through follower chain*/
    while (current) {
        if (visited.count(current) > 0) {
            /*if that robot has been visited, there is a loop in follower chain*/
            return true;
        }
        visited.insert(current);
        current = current->follower;
    }

    return visited.count(candidate) > 0;
}

bool Robot::isInPath(std::shared_ptr<Cell> c){
    return std::find(path.begin(), path.end(), c) != path.end();
}

Robot* Robot::determinePriority(Robot* r1, Robot* r2) {
    /* check which robot has the highest priority and return it */

    std::shared_ptr<Cell> criticalNode = (r1->step() == r2->step()) ? r1->step() : nullptr;

    /* priority rule number 1: robot occupying critical node is given priority */
    if(!(r1->getCurrentCell() == r2->step() && r1->step() == r2->getCurrentCell())){
        if (r1->getCurrentCell() == criticalNode) return r1;
        if (r2->getCurrentCell() == criticalNode) return r2;
    }

    /* priority rule number 2: a robot giving way to another robot is given priority */
    if (r1->isGivingWay()) return r1;
    if (r2->isGivingWay()) return r2;

    /* priority rule number 3: the robot with the highest nFollowers is given priority */
    if (r1->getNFollowers() > r2->getNFollowers()) return r1;
    if (r2->getNFollowers() > r1->getNFollowers()) return r2;

    /* priority rule number 4: a robot having a free neighboring node is given priority */
    if (r1->findGiveWayNode()) return r1;
    if (r2->findGiveWayNode()) return r2;

    /* priority rule number 5: the robot with the highest neighborsRequestingMyNode is given priority */
    if (r1->getNeighborsRequestingNode() > r2->getNeighborsRequestingNode()) return r1;
    if (r2->getNeighborsRequestingNode() > r1->getNeighborsRequestingNode()) return r2;

    /* priority rule number 6: the robot with the longest path is given priority */
    if (r1->getPath().size() > r2->getPath().size()) return r1;
    if (r2->getPath().size() > r1->getPath().size()) return r2;

    /* Default case, if none of the above rules apply */
    return r1;
}

void Robot::solveIntersectionConflict(Robot* n){
    /*solves intersection conflict with robot n*/

    /*first, we must determine which robot has priority*/
    Robot* priorityRobot = determinePriority(this, n);
    Robot* nonPriorityRobot = (priorityRobot == this) ? n : this;
    
    std::vector<std::shared_ptr<Cell>> priorityPath = priorityRobot->getPath();
    if(priorityPath.size()<2 && !priorityRobot->atGoal()){
        priorityRobot->stopRobot();
        return;
    }
    else if(priorityRobot->atGoal()){
        priorityRobot->giveWay();
        return;
    }

    Robot* aheadRobot = priorityPath[1]->getObjID(); /*points to the robot occupying n(t+2) of high priority robot*/

    /*based on that, the robot of more priority checks if it's n(t+2) node is free
    (a free node is one wich is not an obstacle or is not occupied by another robot in that time step)*/
    if(!priorityPath[1]->isObstacle() && !aheadRobot){
        /*if the node is free, the low priority robot and all robots requesting the node stop*/
        for(auto r : priorityRobot->neighborsRequestingNode){
            r->stopRobot();
            r->noConflictDetected = false;
        }
    }
    else if(aheadRobot){
        if(aheadRobot->findGiveWayNode()){
            std::cout<<aheadRobot->getID()<<" moving out of the way on an intersection conflict with "<<priorityRobot->getID()<<std::endl;
            aheadRobot->giveWay();
        }
        else{
            /*if no neighboring node is found, ask neighbor robot to move out of the way*/
            for(auto& n : aheadRobot->neighbors){
                if(n->findGiveWayNode()){
                    std::cout<<"neighbor "<<n->getID()<<" asked to give way for robot "<<aheadRobot->getID()<<std::endl;
                    n->giveWay();
                    aheadRobot->path.insert(path.begin(), n->getCurrentCell());
                    break;
                }
            }
        }
    }
}

void Robot::solveOppositeConflict(Robot* n){
    /*SOLVE SWAPPING CONFLICT BETWEEN TWO ROBOTS*/
    
    Robot* priorityRobot = determinePriority(this, n);
    Robot* lowPriorityRobot = (priorityRobot == this) ? n : this;

    if(lowPriorityRobot->findGiveWayNode()){
        std::cout<<lowPriorityRobot->getID()<<" in opposite conflict giving way to "<<priorityRobot->getID()<<std::endl;
        lowPriorityRobot->giveWay();
    }
    else if(priorityRobot->findGiveWayNode()){
        std::cout<<lowPriorityRobot->getID()<<" in opposite conflict giving way to "<<priorityRobot->getID()<<std::endl;
        priorityRobot->giveWay();
    }
    else{
        std::cout<<lowPriorityRobot->getID()<<" retreating to follower cell"<<std::endl;
        if(follower){
            if(follower->findGiveWayNode()){
                follower->giveWay();
                path.insert(path.begin(), follower->getCurrentCell());
            }
        }
    }

}

void Robot::giveWay(){
    if(findGiveWayNode()){

        std::vector<std::shared_ptr<Cell>> currNeighbors = this->step()->getNeighbors();

        auto isReachable = [this, &currNeighbors]() -> bool {
            return std::find(currNeighbors.begin(), currNeighbors.end(), giveWayNode) == currNeighbors.end();
        };

        /*if we cant reach next cell in path from givewaynode, return to current cell after giving way*/
        if((!isReachable() || atGoal()) && !isInPath(currentCell)){
            giveWayNode->logPos();
            std::cout<<" not reachable from ";
            this->step()->logPos();
            std::cout<<" inserting current cell into path"<<std::endl;

            path.insert(path.begin(), currentCell);
            std::cout<<"path after inserting current cell:"<<std::endl;
            logPath();
        }

        path.insert(path.begin(), giveWayNode);

        std::cout<<"path after inserting give way node: "<<std::endl;
        logPath();
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
    this->plannedAction = 0;
}

void Robot::resumeRobot(){
    this->plannedAction = 1;
}

void Robot::logPath(){
    std::cout<<"Path for "<<id<<":"<<std::endl;
    for(auto& c : path){
        c->logPos();
        std::cout<<", ";
    }
    std::cout<<std::endl;
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

void Robot::setLeader(Robot* l){
    this->leader = l;
}

Robot* Robot::getLeader(){
    return leader;
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
    return plannedAction;
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

int Robot::getNeighborsRequestingNode(){
    return neighborsRequestingNode.size();
}

std::shared_ptr<Cell> Robot::getCurrentCell(){return this->currentCell;}

std::shared_ptr<Cell> Robot::getGoal(){return this->goal;}
