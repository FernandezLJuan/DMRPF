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
        if(leader){
            if(leader->isMoving()){
                this->resumeRobot();
            }
        }
        else{
            /*if the robot has no leader, resume movement*/
            this->resumeRobot();
        }

    }

    if(moving){
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
            lastCell = *path.begin();
            if(currentCell!=giveWayNode) pathHistory.push_back(currentCell);
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

bool Robot::findGiveWayNode(){
    /*scan surrounding cells of the current cell*/
    for (const auto& neighbor : currentCell->getNeighbors()) {
        /*if there isn't a robot in the neighbor, use it as give way node*/
        if(neighbor->getObjID()==nullptr && (std::find(path.begin(), path.end(), neighbor)==path.end())){
            giveWayNode = neighbor;
            break;
        }
    }

    /*if a giveWayNode is found, return true*/
    return (giveWayNode) ? true : false;
}

void Robot::fetchNeighborInfo(){
    /*check for robots in the detection range of the robot*/

    std::shared_ptr<Cell> rStep; /*immediate next node of neighbor robot*/
    std::vector<std::shared_ptr<Cell>> remainingNodes; /*remaining nodes of neighbor robot rn*/
    
    for(auto n : neighbors){
            if(n){
                switch(environment->detectConflict(*this, *n)){
                    case 0:
                        /*NO CONFLICT IS DETECTED*/
                        if(follower){
                            /*if our follower has a longer path than ours, move out of the way*/
                            if(follower->getPath().size() > path.size() && !follower->isGivingWay()){
                                this->giveWay();
                            }
                        }
                        // if(leader){
                        //     if(!leader->isMoving() && !leader->atGoal()){
                        //         std::cout<<id<<" waiting for leader "<<leader->getID()<<std::endl;
                        //         this->stopRobot();
                        //     }
                        //     else if(leader->isMoving() && leader->step() == currentCell && !leader->isGivingWay()){
                        //         //if the leader's next step is trying to move to our cell and it is not giving way to another robot, give way to it
                        //         std::cout<<id<<" giving way to leader "<<leader->getID()<<std::endl;
                        //         this->giveWay();
                        //     }
                        // }
                        break;
                    case 1:
                        /*OPPOSITE CONFLICT*/
                        solveOppositeConflict(n);
                        break;
                    case 2:
                        /*INTERSECTION CONFLICT*/
                        solveIntersectionConflict(n);
                        break;
                }
            }
    }
}

void Robot::findFollowers(){
    numberFollowers = 0;
    std::shared_ptr<Cell> nextCell = this->step();

    auto isInHistory = [this](std::shared_ptr<Cell> c)->bool{
        return std::find(this->pathHistory.begin(), this->pathHistory.end(), c) != this->pathHistory.end();
    };

    for(auto& n : neighbors){
        if(n){
            if(n->step() == currentCell){ 
                /*check if the robot's next cell is in my history of cells, then set as follower*/
                if(!isInFollowerChain(n) && !n->getLeader()){
                    /*if the robot is already in the chain of followers, don't set it as my follower*/
                    n->setLeader(this);
                    this->follower = n;
                    numberFollowers++;
                }
            }

            if(n == follower){                
                /*if the follower has stopped being in the path history*/
                if(n->step() != currentCell){
                    numberFollowers--;
                    follower = nullptr;
                    n->setLeader(nullptr);
                }
            }

            /*find if neighbor is requesting node and remove it if it is not requesting anymore*/
            auto it = std::find(neighborsRequestingNode.begin(), neighborsRequestingNode.end(), n);
            if (it != neighborsRequestingNode.end()) {
                neighborsRequestingNode.erase(it);
            }
            else if(n->step() == currentCell){
                neighborsRequestingNode.push_back(n);
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
    return nullptr;
}

void Robot::solveIntersectionConflict(Robot* n){
    /*solves intersection conflict with robot n*/

    /*first, we must determine which robot has priority*/
    Robot* priorityRobot = determinePriority(this, n);
    Robot* nonPriorityRobot = (priorityRobot == this) ? n : this;
    
    std::vector<std::shared_ptr<Cell>> priorityPath = priorityRobot->getPath();
    if(priorityPath.size()<3){
        if(!priorityRobot->isGivingWay()){
            std::cout<<priorityRobot->getID()<<" giving way because path is too short"<<std::endl;
            priorityRobot->giveWay();
        }
        else nonPriorityRobot->stopRobot();
        return;
    }
    Robot* obstacleRobot = priorityPath[2]->getObjID(); /*points to the robot occupying n(t+2) of high priority robot*/

    /*based on that, the robot of more priority checks if it's n(t+2) node is free
    (a free node is one wich is not an obstacle or is not occupied by another robot in that time step)*/
    if(!obstacleRobot){
        for(auto& r : priorityRobot->neighborsRequestingNode){
            /*robots requesting that node must wait*/
            r->stopRobot();
        }
    }
    else if(obstacleRobot->findGiveWayNode()){
        obstacleRobot->giveWay();
    }
    else{
        for(auto& n:obstacleRobot->neighbors){
            if(n->findGiveWayNode()){
                n->giveWay();
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
        std::cout<<"need to retreat"<<std::endl;
    }

}

void Robot::giveWay(){
    if(findGiveWayNode()){
        path.insert(path.begin(), currentCell);
        path.insert(path.begin(), giveWayNode);
    }
}

void Robot::retreat(){
    /*move to the last cell you've been at*/
    if(lastCell && !isGivingWay()){
        path.insert(path.begin(), lastCell);
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
    return moving;
}

bool Robot::isGivingWay(){
    auto isGivewayInPath = [this](std::shared_ptr<Cell> c)->bool{
        return std::find(this->path.begin(), this->path.end(), this->giveWayNode) != this->path.end();
    };
    return isGivewayInPath(giveWayNode);
}

void Robot::logPath(){
    std::cout<<"I'm "<<id<<" and my path is: "<<std::endl;
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
