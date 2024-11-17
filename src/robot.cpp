#include "robot.h"

int Robot::currentID = 0;

void Robot::takeAction(){
    /*robot will either stop for this step or follow the path to it's next node*/

    std::cout<<id<<" taking action "<<std::endl;

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
                if(!path.front()){
                    std::cout<<"WTF"<<std::endl;
                }

                leader = path.front()->getObjID();

                if(leader){
                    if(!leader->isMoving()){
                        this->stopRobot();
                    }
                    else if(leader->step() == currentCell){
                        this->giveWay();
                    }
                }
                else{
                    plannedAction = 1;
                }
            }
        }
    }

    if(plannedAction){
        if(!path.empty()){
            this->move(path.front());
        }
    }

    if(path.empty()){
        done = true;
    }
    else{
        done = false;
    }

    if(isGivingWay()){
        giveWayNode = nullptr;
        givingWay = false;
    }
}

void Robot::resetPath(){
    pathHistory.clear();
    path.clear();
    pathLength = 0;
}

void Robot::insertCell(Cell* c){
    path.insert(path.begin(), c);
}

void Robot::reconstructPath(std::unordered_map<Cell*, Cell*> recordedPath, Cell* cell){

    auto it = recordedPath.find(cell);
    size_t oldPathLength = pathLength;

    while(it != recordedPath.end()){
        path.push_back(cell);
        cell = it->second;

        it = recordedPath.find(cell);
    }

    std::reverse(path.begin(), path.end());

    pathLength = (path.size()>0) ? path.size()+oldPathLength : oldPathLength;

    std::cout<<id<<"With path length: "<<pathLength<<std::endl;
}

void Robot::generatePath() {
    /*generates a path from the current cell to the goal of the robot, it keeps the old path history*/

    this->path.clear();

    if (goal == nullptr) {
        return;
    }

    Cell* tmp = currentCell;
    std::unordered_map<Cell*, int> gScore;
    std::unordered_map<Cell*, int> fScore;
    std::unordered_map<Cell*, Cell*> recordedPath;

    //use unordered set to avoid duplicates in openSet
    std::unordered_set<Cell*> openSet;
    auto compareFScore = [&](Cell* a, Cell* b) {
        return fScore[a] < fScore[b];
    };

    auto getLowestFScore = [&](const std::unordered_set<Cell*>& set) {
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

        std::vector<Cell*> tmpNeighbors = tmp->getNeighbors();
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

void Robot::move(Cell* newPos){ 
    /*change the position of the robot*/

    if(newPos==nullptr){
        std::cout<<"sorry, null position"<<std::endl;
        return;
    }

    const std::vector<Cell*>& something = currentCell->getNeighbors();

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

    std::unordered_set<Cell*> visited{currentCell}; /*mark current cell as visited so it is not included in the range*/
    std::deque<std::pair<Cell*, int>> toVisit; /*cells to visit and determine distance*/

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

void Robot::findNeighbors(){
    neighbors.clear();

    for(auto& c : detectionArea){
        auto robot = c->getObjID();
        if(robot){
            neighbors.push_back(robot);
        }
    }
}

bool Robot::findGiveWayNode(){
    /*scan neighbors of current cell and look for a free node to move into*/

    /*assume there is no free neighboring node*/
    freeNeighboringNode = nullptr;

    auto isInHistory = [this](Cell* c)->bool{
        return std::find(this->pathHistory.begin(), this->pathHistory.end(), c) != this->pathHistory.end();
    };

    for (const auto& cellNeighbor : currentCell->getNeighbors()) {
        /*if there isn't a robot in the neighbor, use it as give way node*/

        for(auto& rn : neighbors){
            if(cellNeighbor != rn->step()){
                
                /*if a neighboring cell is not part of our current path and is not occupied by a robot, 
                consider it as a valid give-way node, provided it has not been visited before.
                This prevents the robot from looping between nodes in its movement history*/
                if((cellNeighbor->getObjID()==nullptr && !isInPath(cellNeighbor)) && !isInHistory(cellNeighbor)){
                    freeNeighboringNode = cellNeighbor;
                    break;
                }
                else if((cellNeighbor->getObjID()==nullptr && !isInPath(cellNeighbor))){
                    /* if no strict give-way node is found, allow a previously visited cell (from history),
                    as long as it’s unoccupied and not part of the current path. */
                    freeNeighboringNode = cellNeighbor;
                    break;
                }
            }
        }
    }

    /*if a giveWayNode is found, return true*/
    return (freeNeighboringNode) ? true : false;
}

void Robot::fetchNeighborInfo(){
    /*check for robots in the detection range of the robot*/

    if(neighbors.empty()){
        noConflictDetected=true;
        this->resumeRobot();
        return;
    }

    noConflictDetected = false;
    
    for(auto n : neighbors){

        if(!n){
            continue;
        }

        std::cout<<"checking conflict between "<<this->getID()<<" AND "<<n->getID()<<std::endl;

        switch(environment->detectConflict(this, n)){
            case 0:
                noConflictDetected = true;
                this->resumeRobot();
                
                /*NO CONFLICT IS DETECTED*/
                if(follower){
                    std::cout<<follower->getID()<<" following "<<id<<std::endl;
                    /*if our follower has a longer path than ours, move out of the way*/
                    if(follower->getPath().size() > path.size()){
                        std::cout<<id<<" [GIVING WAY TO FOLLOWER] ";
                        std::cout<<follower->getID()<<std::endl;
                        this->giveWay();
                    }
                }
                break;
            case 1:
                /*OPPOSITE CONFLICT*/
                ++conflictCount[0];
                solveOppositeConflict(n);
                break;
            case 2:
                /*INTERSECTION CONFLICT*/
                ++conflictCount[1];
                std::cout<<"[INTERSECTION CONFLICT BETWEEN] <"<<id<<" AND "<<n->getID()<<">"<<std::endl;
                solveIntersectionConflict(n);
                break;
        }
    }
}

void Robot::findFollowers(){
    neighborsRequestingNode.clear(); /*reset neighbors requesting node*/
    numberFollowers = 0; /*assume there are no followers before searching for them*/

    /*for any neighboring robots*/
    for(auto& n : neighbors){
        if(!n){
            continue;
        }

        if(n->step() == currentCell){ 
            /*check if the robot's next cell is in my history of cells, then set as follower*/
            if(!isInFollowerChain(n)){
                /*if the robot is already in the chain of followers, don't set it as my follower*/
                std::cout<<n->getID()<<" started to follow "<<id<<std::endl;
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
            if(n->step() != currentCell){
                std::cout<<n->getID()<<" stopped following "<<id<<std::endl;
                follower = nullptr;
            }
        }

        /*check if the robot's next cell or n(t+2) cell are my node*/
        if(n->step() == currentCell || (n->getPath().size()>2 && n->getPath()[1] == currentCell)){
            neighborsRequestingNode.push_back(n);
        }
    }

    nodeRequests += neighborsRequestingNode.size();
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

bool Robot::isInPath(Cell* c){
    return std::find(path.begin(), path.end(), c) != path.end();
}

Robot* Robot::determinePriority(Robot* r1, Robot* r2) {
    /* check which robot has the highest priority and return it */

    Cell* criticalNode = (r1->step() == r2->step()) ? r1->step() : nullptr;

    /* priority rule number 1: robot occupying critical node is given priority */
    if(!(r1->getCurrentCell() == r2->step() && r1->step() == r2->getCurrentCell())){
        if (r1->getCurrentCell() == criticalNode){ 
            ++ruleCount[0];
            return r1;
        }
        if (r2->getCurrentCell() == criticalNode){
            ++ruleCount[0];
            return r2;
        }
    }

    /* priority rule number 2: a robot giving way to another robot is given priority */
    if (r1->isGivingWay()){
        ++ruleCount[1];
        return r1;
    }
    if (r2->isGivingWay()){
        ++ruleCount[1];
      return r2;  
    } 

    /* priority rule number 3: the robot with the highest nFollowers is given priority */
    if (r1->getNFollowers() > r2->getNFollowers()){ 
        ++ruleCount[2];
        return r1;
    }
    if (r2->getNFollowers() > r1->getNFollowers()){
        ++ruleCount[2];
        return r2;
    }

    /* priority rule number 4: a robot having a free neighboring node is given priority */
    if (r1->findGiveWayNode()){
        ++ruleCount[3];
        return r1;
    
    }
    if (r2->findGiveWayNode()){ 
        ++ruleCount[3];
        return r2;
    }

    /* priority rule number 5: the robot with the highest neighborsRequestingMyNode is given priority */
    if (r1->getNeighborsRequestingNode() > r2->getNeighborsRequestingNode()){
        ++ruleCount[4];
        return r1;
    }
    if (r2->getNeighborsRequestingNode() > r1->getNeighborsRequestingNode()){
        ++ruleCount[4];
        return r2;
    }

    /* priority rule number 6: the robot with the longest path is given priority */
    if (r1->getPath().size() > r2->getPath().size()){
        ++ruleCount[5];
        return r1;
    }
    if (r2->getPath().size() > r1->getPath().size()){ 
        ++ruleCount[5];
        return r2;
    }

    /* Default case, if none of the above rules apply */
    return r1;
}

void Robot::solveIntersectionConflict(Robot* n){
    /*solves intersection conflict with robot n*/

    /*first, we must determine which robot has priority*/
    auto priorityRobot = determinePriority(this, n);
    auto nonPriorityRobot = (priorityRobot == this) ? n : this;
    
    std::vector<Cell*> priorityPath = priorityRobot->getPath();

    if(priorityPath.size()<2 && !priorityRobot->atGoal()){
        priorityRobot->stopRobot();
        return;
    }
    else if(priorityRobot->atGoal()){
        priorityRobot->giveWay();
        return;
    }

    if(priorityPath.size()<2){
        priorityRobot->stopRobot();
        return;
    }

    Robot* aheadRobot = priorityPath[1]->getObjID(); /*points to the robot occupying n(t+2) of high priority robot*/

    /*based on that, the robot of more priority checks if it's n(t+2) node is free
    (a free node is one wich is not an obstacle or is not occupied by another robot in that time step)*/
    if(!aheadRobot){
        nonPriorityRobot->stopRobot();
        /*if the node is free, the low priority robot and all robots requesting the node stop*/
        for(auto r : priorityRobot->neighborsRequestingNode){
            if(!r){
                continue;
            }

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
            std::vector<Robot*> aheadNeighbors = aheadRobot->getNeighbors();
            /*if no neighboring node is found, ask neighbor robot to move out of the way*/
            for(auto& n : aheadNeighbors){
                if(!n){
                    continue;
                }
                if(n->findGiveWayNode()){
                    std::cout<<"neighbor "<<n->getID()<<" asked to give way for robot "<<aheadRobot->getID()<<std::endl;
                    n->giveWay();
                    std::cout<<"INSERTING FOLLOWER PATH INTO ROBOT"<<std::endl;
                    aheadRobot->insertCell(n->getCurrentCell());
                    std::cout<<"REACHED THIS POINT, NO PROBLEM"<<std::endl;
                    std::cout<<n->getID()<<", "<<aheadRobot->getID()<<std::endl;
                    break;
                }
            }
        }
    }

    std::cout<<"EXITING FUNCTION NOW"<<std::endl;
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

    if(givingWay){
        return;
    }

    if(findGiveWayNode()){

        giveWayNode = freeNeighboringNode;

        const std::vector<Cell*>& nextNeighbors = this->step()->getNeighbors();

        /*returns true if the cell can be reached from the next cell in the path*/
        auto isReachable = [this, &nextNeighbors]() -> bool {
            return std::find(nextNeighbors.begin(), nextNeighbors.end(), giveWayNode) != nextNeighbors.end();
        };

        /*if we cant reach next cell in path from givewaynode, return to current cell after giving way*/
        if((!isReachable() || atGoal())){
            path.insert(path.begin(), currentCell);
        }

        path.insert(path.begin(), giveWayNode);
        givingWay = true;
        givenWay++;
    }
}

/*return next step of the robot*/
Cell* Robot::step(){
    if(path.empty()){
        return goal;
    }
    return path.front();
}

/*return vector of pointers as const reference to avoid early 
delete of memory inside function callers*/
const std::vector<Cell*>& Robot::getPath(){
    return this->path;
}

const std::vector<Cell*>& Robot::getArea(){
    return this->detectionArea;
}

const std::vector<Robot*>& Robot::getNeighbors(){
    return this->neighbors;
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
    return done;
}

void Robot::setGoal(Cell* goalPos){
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
    return plannedAction;
}

bool Robot::isGivingWay(){
    return givingWay;
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

const std::array<int, 6> Robot::getRuleCount(){
    return ruleCount;
}

const std::array<int,2> Robot::getConflictCount(){
    return conflictCount;
}

int Robot::howManyGiveWays(){
    return givenWay;
}

size_t Robot::totalRequests(){
    return nodeRequests;
}

size_t Robot::relativePathSize(){

    if(pathLength<=0){
        std::cout<<"NO PATH??"<<std::endl;
        return 0;
    }

    return pathHistory.size()/pathLength;
}

Cell* Robot::getCurrentCell(){return this->currentCell;}

Cell* Robot::getGoal(){return this->goal;}
