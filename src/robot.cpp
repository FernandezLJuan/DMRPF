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
        plannedAction = 0;
    }
    else if(noConflictDetected){ //THIS CONDITION IS HERE TO AVOID THE RNG OVERRIDING WAIT BEHAVIOUR OF CONFLICT SOLVING
        plannedAction = 1;
    }

    if(noConflictDetected){
        if(!path.empty()){
            leader = path.front()->getObjID();

            if(leader){
                if(leader->plannedAction == 0){
                    plannedAction = 0;
                }
                else if(leader->plannedAction && leader->step() == currentCell){
                    plannedAction = 1;
                    path.insert(path.begin(), giveWayNode);
                }
            }
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

    if(!(environment->moveRobot(this, newPos)<0) && this->moving){
        currentCell = environment->getCellByID(newPos->getID());
        updateDetectionArea(); /*update detection area after moving*/      

        if(!path.empty()){
            if(currentCell == giveWayNode){
                giveWayNode=nullptr;
            }
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

    for(auto& c:detectionArea){
        if(c->getObjID() && c!=currentCell){
            neighbors.push_back(c->getObjID());
        }
    }
}

bool Robot::findGiveWayNode(){

    std::cout<<id<<" at ";
    currentCell->logPos();
    std::cout<<std::endl;

    /*scan surrounding cells of the current cell*/
    for (const auto& cellNeighbor : currentCell->getNeighbors()) {
        std::cout<<"Checking if ";
        cellNeighbor->logPos();
        std::cout<<" is a valid giveWayNode"<<std::endl;

        /*if there isn't a robot in the neighbor, use it as give way node*/
        for(auto& n : neighbors){
            if(n->step() != cellNeighbor){
                if(cellNeighbor->getObjID()==nullptr && (std::find(path.begin(), path.end(), cellNeighbor)==path.end())){
                    giveWayNode = cellNeighbor;
                    break;
                }
            }
        }
    }

    if(giveWayNode){
        std::cout<<"Found ";
        giveWayNode->logPos();
        std::cout<<" as valid give way node for "<<id<<std::endl;

        return true;
    }
    else{
        return false;
    }
}

void Robot::fetchNeighborInfo(){
    /*check for conflicts with neighbors in the detection range*/

    if(neighbors.size() == 0){
        /*if there are no neighbors, there can't be a conflict*/
        noConflictDetected = true;
        return;
    }

    noConflictDetected = false; /*assume there is a conflict*/

    for(auto& n:neighbors){
        if(!n){
            continue;
        }
        
        std::cout<<"[DETECTING CONFLICT FOR] <"<<id<<","<<n->getID()<<">"<<std::endl;
        switch(environment->detectConflict(*this, *n)){
            case 0:
                //std::cout<<"[NO CONFLICT BETWEEN] <"<<id<<","<<n->getID()<<">"<<std::endl;
                plannedAction = 1;
                noConflictDetected = true;

                /*insert giveWayNode into path if my follower has a larger path*/
                if(this->follower){
                    if(follower->getPath().size() > path.size()){
                        plannedAction = 1;
                        this->giveWay();
                    }
                }

                break;
            
            case 1:
                std::cout<<"[OPPOSITE CONFLICT BETWEEN] <"<<id<<","<<n->getID()<<">"<<std::endl;
                solveOppositeConflict(n);
                noConflictDetected = false;
                break;

            case 2:
                std::cout<<"[INTERSECTION CONFLICT BETWEEN] <"<<id<<","<<n->getID()<<">"<<std::endl;
                solveIntersectionConflict(n);
                noConflictDetected = false;
                break;
        }
    }
}

void Robot::findFollowers(){
    /*assume zero followers at the start of the function*/
    numberFollowers = 0;
    for(auto& n:neighbors){
        if(!n){
            continue;
        }
        if(n->step() == currentCell || (n->path.size()>1 && n->path[1]==currentCell)){
            /*if the next cell is the current cell or n_id(k+2) is the current cell it can be a follower and a neighbor requesting node*/

            if(!isInFollowerChain(n) && !n->isGivingWay()){/*to avoid loops in follower chain, check if the robot is already following another robot*/
                this->follower = n;
                numberFollowers++;
            }

            /*even if it is not a follower, it is requesting the node nonetheless*/
            neighborsRequestingNode.push_back(n);
        }
        else if(n==follower){
            auto it = std::find(neighborsRequestingNode.begin(), neighborsRequestingNode.end(), n);
            if(it != neighborsRequestingNode.end()){
                neighborsRequestingNode.erase(it);
            }
            this->follower = nullptr;
        }
        else{
            auto it = std::find(neighborsRequestingNode.begin(), neighborsRequestingNode.end(), n);
            if(it != neighborsRequestingNode.end()){
                neighborsRequestingNode.erase(it);
            }
        }

        if(n==follower){
            if(n->isGivingWay()){
                follower = nullptr;
            }
        }

        /*while my follower has a follower, count that towards the total number of followers*/
        if(n==follower){
            Robot* tmp = this->follower;
            std::unordered_set<Robot*> uniqueFollowers;
            while(tmp && uniqueFollowers.count(tmp) == 0){
                uniqueFollowers.insert(tmp);
                numberFollowers++;
                tmp = tmp->follower;
            }
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

    std::cout<<"[CHECKING RULE 4]"<<std::endl;

    bool r1_hasFreeNode = r1->findGiveWayNode();
    bool r2_hasFreeNode = r2->findGiveWayNode();

    /* priority rule number 4: a robot having a free neighboring node is given priority */
    if(r1_hasFreeNode && !r2_hasFreeNode){
        return r1;
    }
    else if(!r1_hasFreeNode && r2_hasFreeNode){
        return r2;
    }

    /* priority rule number 5: the robot with the highest neighborsRequestingMyNode is given priority */
    if (r1->getNeighborsRequestingNode() > r2->getNeighborsRequestingNode()) return r1;
    if (r2->getNeighborsRequestingNode() > r1->getNeighborsRequestingNode()) return r2;

    /* priority rule number 6: the robot with the longest path is given priority */
    if (r1->getPath().size() > r2->getPath().size()) return r1;
    if (r2->getPath().size() > r1->getPath().size()) return r2;

    /* Default case, if none of the above rules apply */
    return nullptr;
}

void Robot::solveIntersectionConflict(Robot* n){
    /*solves intersection conflict with robot n*/

    /*first, we must determine which robot has priority*/
    Robot* priorityRobot = determinePriority(this, n);
    Robot* lowPriorityRobot = (priorityRobot == n) ? this : n; /*if n has priority, then we are the lower priority robot*/
    
    std::vector<std::shared_ptr<Cell>> priorityPath = priorityRobot->getPath();

    //TODO: COME UP WITH A SOLUTION FOR THIS
    if(priorityPath.size()<2 && !priorityRobot->atGoal()){
        priorityRobot->plannedAction = 0;
        return;
    }
    else if(priorityRobot->atGoal()){
        priorityRobot->giveWay();
        std::cout<<id<<" [GIVING WAY AT GOAL]"<<std::endl;
        return;
    }

    Robot* aheadRobot = priorityPath[1]->getObjID(); /*robot at n(t+2) (next node is path[0], n(t+2) would be path[1])*/

    if(aheadRobot){
        std::cout<<"[AHEAD ROBOT FINDING GIVEWAYNODE] <"<<aheadRobot->getID()<<">"<<std::endl;
        if(aheadRobot->findGiveWayNode()){
            /*if it has a free neighboring node, move to it*/
            std::cout<<aheadRobot->getID()<<" [FREEING NODE ";
            priorityPath[1]->logPos();
            std::cout<<" TO LET <"<<priorityRobot->getID()<<"> PASS]"<<std::endl;

            aheadRobot->giveWay();
        }
        else{
            std::cout<<"[AHEAD ROBOT ASKING NEIGHBORS TO MOVE]"<<std::endl;
            for(auto& n : aheadRobot->neighbors){
                /*if it doesn't find a free neighboring node, ask a neighbor to give way and move to it's node*/
                if(n->findGiveWayNode()){
                    n->giveWay();
                    aheadRobot->path.insert(path.begin(), n->getCurrentCell());
                    break;
                }
            }
        }
    }
    else{
        for(auto& n : neighborsRequestingNode){
            n->plannedAction = 0;
        }
    }

    std::cout<<"[AFTER SOLVING INTERSECTION CONFLICT] ";
    std::cout<<"Path for "<<priorityRobot->getID()<<std::endl;;
    priorityRobot->logPath();

    std::cout<<"Path for "<<lowPriorityRobot->getID()<<std::endl;
    lowPriorityRobot->logPath();

    std::cout<<"\n"<<std::endl;

}

void Robot::solveOppositeConflict(Robot* n){
    /*SOLVE SWAPPING CONFLICT BETWEEN TWO ROBOTS*/

    Robot* priorityRobot = determinePriority(this, n);
    Robot* lowPriorityRobot = (priorityRobot == n) ? this : n;

    if(lowPriorityRobot->findGiveWayNode()){
        lowPriorityRobot->giveWay();
    }
    else if(priorityRobot->findGiveWayNode()){
        priorityRobot->giveWay();
    }
    else{
        if(lowPriorityRobot->follower){
            /*retreat back to follower cell if no giveWayNode is available for either robot*/
            lowPriorityRobot->follower->giveWay();
            lowPriorityRobot->path.insert(lowPriorityRobot->path.begin(),lowPriorityRobot->follower->getCurrentCell());
        }
    }

}

void Robot::giveWay(){
    if(findGiveWayNode()){
        std::cout<<"Path before: "<<std::endl;
        logPath();

        std::cout<<"Inserting ";
        giveWayNode->logPos();
        std::cout<<" as giveWayNode, path looks like: "<<std::endl;

        path.insert(path.begin(), giveWayNode);
        logPath();
    }
}

/*return next step of the robot*/
std::shared_ptr<Cell> Robot::step(){
    if(!path.empty()){
        return path.front();
    }
    return nullptr;
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

void Robot::logPath(){
    std::cout<<"I'm "<<id<<" at ";
    currentCell->logPos();
    std::cout<<" and my path is: "<<std::endl;
    for(auto& c: path){
        c->logPos();
        std::cout<<", ";
    }
    std::cout<<std::endl;
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
