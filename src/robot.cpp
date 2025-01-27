#include "robot.h"

int Robot::currentID = 0;

std::ostream& operator<<(std::ostream& str, const Robot& r){
    Cell* curr = r.currentCell;

    str<<*curr;
    return str;
}

void Robot::takeAction(){
    /*robot will either stop for this step or follow the path to it's next node*/

    /*generate random number between 0 and 1*/
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    double willIStop = dis(gen);

    /*check again if there is a conflict with the next action to be taken*/
	for(auto& n : neighbors){
        switch ((environment->detectConflict(n, this)))
        {
        case 1:
            solveOppositeConflict(n);
            break;

        case 2:
            solveIntersectionConflict(n);
        
        default:
            break;
        }
    }

    //wait randomly
    if(willIStop<waitProbability){
        this->stopRobot();
    }
    else{
        /*if the robot is not randomly waiting, check for leader actions*/
        if(noConflictDetected){
            if(!path.empty()){
                //get leader of robot
                leader = path.front()->getObjID();

                if(leader){
                    //if a robot has a leader, imitate action if the leader is waiting or
                    //give way if it's moving to the curren cell
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

    //if the robot has been waiting for more than 10 time steps, resume movement
    if(waitingSteps>=10){
        plannedAction = 1;
        waitingSteps = 0;
    }

    //move the robot
    if(plannedAction){
        waitingSteps = 0; //if it is moving it can't be waiting, reset number of steps the robot has been waiting for
        if(!path.empty()){
            this->move(path.front());
        }
    }
    else{
        //if the robot is not moving, add 1 to the time steps it has been waiting for 
        waitingSteps+=1;
    }

    //if the path is empty the robot has reached it's goal
    if(path.empty()){
        done = true;
    }
    else{
        done = false;
    }
}

void Robot::resetPath(){
    /*completely resets robot path and path length*/
    pathHistory.clear();
    path.clear();
    pathLength = 0;
}

void Robot::insertCell(Cell* c){
    /*insert a new cell at the start of a robot's path*/
    const std::vector<Cell*>& nextNeighbors = this->step()->getNeighbors();

    //if the next cell in the path cannot be reached from the new inserted cell, add current position before to go back to path
    auto isReachable = [&](const Cell* cell) -> bool {
        return std::find(nextNeighbors.begin(), nextNeighbors.end(), cell) != nextNeighbors.end();
    };

    if((!isReachable(c) || atGoal())){
        std::cout<<id<<"[ACTIVATED MECHANISM]\n";
        path.insert(path.begin(), currentCell);
    }

    path.insert(path.begin(), c);
}

void Robot::reconstructPath(std::unordered_map<Cell*, Cell*>& recordedPath, Cell* cell){
    /*reconstruct the path found by A* search algorithm
    from cell=goal, computes path length*/

    if(recordedPath.size()<=0){
        return;
    }

    auto it = recordedPath.find(cell);
    float oldPathLength = static_cast<float>(pathHistory.size()); //how many cells has the robot visited

    //while we can still find the current cell in the recorded path map, trace back the path.
    while(it != recordedPath.end()){
        path.push_back(cell);
        cell = it->second; //move to the previous cell (as recorded in the map)

        it = recordedPath.find(cell); //find the previous cell in the recorded path map.
    }

    //reverse the path to go from start to goal (since it was constructed backward).
    std::reverse(path.begin(), path.end());

    //set the path length by adding the new path length to the robot's path history in case of replanning
    pathLength = (path.size()>0) ? path.size()+oldPathLength : oldPathLength;
}

void Robot::generatePath() {
    /* 
        Generates a path from the robot's current position to the goal using the A* search algorithm.
        It clears the current path, performs the A* search.
    */

    //reset existing path
    this->path.clear();

    //if there is no goal, exit the function (no path to be found)
    if (goal == nullptr) {
        return;
    }

    Cell* tmp = currentCell; //temporary pointer to the current cell
    std::unordered_map<Cell*, float> gScore; //cost to reach each cell
    std::unordered_map<Cell*, float> fScore; //estaimated cost to reach the goal from start
    std::unordered_map<Cell*, Cell*> recordedPath;

    //lambda to compute f-scores, used for ordering the priority queue
    auto compareFScore = [&](Cell* a, Cell* b){
        return fScore[a] > fScore[b];
    };

    //priority queue to store cells to be processed (open set), with the comparison lambda for sorting.
    std::priority_queue<Cell*, std::vector<Cell*>,decltype(compareFScore)> openSet(compareFScore);
    std::unordered_set<Cell*> openSetElements; //unordered set to track which cells are in the open set

    //push the starting cell into the open set
    openSet.push(currentCell);

    //only initialize gscore and fscore for the current cell (start position)
    gScore[currentCell] = 0.0;
    fScore[currentCell] = environment->cellDistance(*currentCell, *goal);

    //begin A* search loop
    while (!openSet.empty()) {
        //get the cell with the lowest f-score
        tmp = openSet.top();
        openSet.pop();
        openSetElements.erase(tmp); //remove from the set as it's being processed

        //when the goal is expanded, recnostruct the path and exit the function
        if (tmp == goal) {
            reconstructPath(recordedPath, tmp);
            return;
        }

        //get neighbors of the current cell to explore
        std::vector<Cell*> tmpNeighbors = tmp->getNeighbors();
        for (auto& n : tmpNeighbors) {
            //skip blacklisted cells (always transient obstacles)
            if(blacklistedCells.find(n) != blacklistedCells.end()){
                continue;
            }

            //calculate the tentative gScore for this neighbor
            float tentativeGScore = gScore[tmp] + environment->connectionCost(*tmp, *n);

            //if the neighbor hasn't been visited or a shorter path is found, update the scores
            if (gScore.find(n) == gScore.end() || tentativeGScore < gScore[n]) {
                recordedPath[n] = tmp; //record the predecessor for path reconstruction
                gScore[n] = tentativeGScore; //update g-score of this neighbor
                fScore[n] = tentativeGScore + environment->cellDistance(*n, *goal); //update f-score (g-score + heuristic)

                //add the neighbor if it is not in the open set
                if (openSetElements.find(n) == openSetElements.end()) {
                    openSet.push(n);
                    openSetElements.insert(n);
                }
            }
        }
    }

    //is no path to the goal is found, the function exits here
}

void Robot::move(Cell* newPos){ 
    /*change the position of the robot*/
    if(newPos==nullptr){
        return;
    }

    auto& currNeighbors = currentCell->getNeighbors();
    auto isReachable = [&](const Cell* c){
        return std::find(currNeighbors.begin(), currNeighbors.end(), c) != currNeighbors.end();
    };

    //if a dynamic obstacle is found, search for a new path until the next position in it is not a transient obstacle
    while(newPos->isCellTransient() && path.size()>0){
        std::cout<<id<<"^^^^^^^^^^^^^^^FOUND DYNAMIC OBSTACLE AT: "<<*newPos<<"^^^^^^^^^^^^^^^\n";

        //number of times the robot has moved whithout stumbling upon a transient obstacle
        noTransientMoves = 0;

        //blacklist transient obstacles so they are not explored in the search
        blacklistedCells.insert(newPos); 

        generatePath();
        newPos = path[0];
    }

    //if movement is possible, update position
    if(!(environment->moveRobot(this, newPos)<0)){

        //erase the cell from the path and add the current cell to pathHistory
        if(!path.empty()){
            pathHistory.push_back(currentCell);
            path.erase(path.begin());
        }
        currentCell = newPos; //update current cell

        std::cout<<id<<"[UPDATED CURRENT CELL] "<<*currentCell<<"\n";

        //if the robot was giving way and it moved, it is not giving it's way anymore
        if(isGivingWay()){
            givingWay = false;
            giveWayNode = nullptr;
        }

        //update the number of times the robot has moved without finding a transient obstacle
        noTransientMoves++;
        //if more than 10 movements have been made without finding a transient obstacle, reset blacklisted cells (assume there are no obstacles there anymore)
        if(noTransientMoves>=10){
            noTransientMoves = 0;
            blacklistedCells.clear();
        }
         
    }
}

void Robot::updateDetectionArea() {
    /*deletes old detection area of robot and re-calculates it using BFS
    taking into account weighted connections and limiting traversal based on detectionRadius*/

    detectionArea.clear();

    std::unordered_set<Cell*> visited{currentCell};
    std::deque<std::pair<Cell*, double>> toVisit;

    //add all valid neighbors to the detection area if they are inside the radius
    for (const auto& neighbor : currentCell->getNeighbors()) {
        double initialCost = environment->connectionCost(*currentCell, *neighbor);
        if (initialCost <= detectionRadius) {
            toVisit.push_back({neighbor, initialCost});
            detectionArea.push_back(neighbor);
            visited.insert(neighbor);
        }
    }

    //perform BFS traversal to explore further cells within detectionRadius
    while (!toVisit.empty()) {
        auto [curr, accumulatedDistance] = toVisit.front(); //extract current cell and accumulated distance
        toVisit.pop_front(); //mark that cell as visited

        //check neighbors of the current cell
        for (auto n : curr->getNeighbors()) {
            //calculate distance of neighbor to current cell
            double conCost = environment->connectionCost(*curr, *n);
            double newDistance = conCost + accumulatedDistance;

            //if it is withing the detection radius and it has not been visited, add it to the detection area
            if (newDistance <= detectionRadius && visited.find(n) == visited.end()) {
                toVisit.push_back({n, newDistance});
                detectionArea.push_back(n);
                visited.insert(n);
            }
        }
    }
}

void Robot::findNeighbors(){
    /*scans detection area for other robots*/
    neighbors.clear(); //clear old neighbors
    neighborsRequestingNode.clear(); /*reset neighbors requesting node*/

    for(auto& c : detectionArea){
        //look for cells with a robot in the detection area and add that robot as a neighbor
        auto robot = c->getObjID();
        if(robot){
            neighbors.push_back(robot);
        }
    }

    for(auto& n : neighbors){
        //check if the robot wants to move to my cell in n(t+1) or n(t+2)
        if(n->step() == currentCell || (n->getPath().size()>2 && n->getPath()[1] == currentCell)){
                neighborsRequestingNode.push_back(n);
        }
    }

    nodeRequests += neighborsRequestingNode.size();
}

bool Robot::findGiveWayNode(){
    /*scan neighbors of current cell and look for a free node to move into*/

    /*assume there is no free neighboring node*/
    freeNeighboringNode = nullptr;

    std::vector<Cell*> currNeighbors = currentCell->getNeighbors();
    
    static std::random_device rd;
    static std::mt19937 g(rd());

    //randomize the neighboring cells to reduce probability of selecting the same giveWayNode
    std::shuffle(currNeighbors.begin(), currNeighbors.end(), g);

    //lambda to check if the cell has been visited before
    auto isInHistory = [this](Cell* c)->bool{
        return std::find(this->pathHistory.begin(), this->pathHistory.end(), c) != this->pathHistory.end();
    };

    for (const auto& cellNeighbor : currNeighbors) {
        if(cellNeighbor->isObstacle()){
            continue;
        }
        /*if a neighboring cell is not part of our current path and is not occupied by a robot, 
        consider it as a valid give-way node, provided it has not been visited before.
        This prevents the robot from looping between nodes in its movement*/
        if(!cellNeighbor->getObjID() && !isInPath(cellNeighbor) && !isInHistory(cellNeighbor)){
            freeNeighboringNode = cellNeighbor;
            break;
        }
    }

    //if a free neighboring node hasn't been found, allow a node in the history that has not been visited more than 20 times
    if(!freeNeighboringNode){
        for(const auto& cellNeighbor : currNeighbors){
            if(cellNeighbor->isObstacle()){
                continue;
            }
            if((!cellNeighbor->getObjID() && !isInPath(cellNeighbor)) && std::count(pathHistory.begin(), pathHistory.end(), cellNeighbor)<20){
                freeNeighboringNode = cellNeighbor;
                break;
            }
        }
    }

    //if a free neighboring node hasn't been found, allow a node in the path that has not been visited more than 50 times
    if(!freeNeighboringNode){
        for(const auto& cellNeighbor : currNeighbors){
            if(cellNeighbor->isObstacle()){
                continue;
            }
            if((!cellNeighbor->getObjID()) && std::count(pathHistory.begin(), pathHistory.end(), cellNeighbor)<50){
                freeNeighboringNode = cellNeighbor;
                break;
            }
        }
    }

    /*if a giveWayNode is found, return true*/
    return (freeNeighboringNode) ? true : false;
}

void Robot::fetchNeighborInfo(){
    /*check for any conflicts arising from planned movement with neighbors in the detection area*/

    //if the robot has no neighbors, it's pointless to look for conflicts
    if(neighbors.empty()){
        noConflictDetected=true;
        this->resumeRobot();
        return;
    }

    //assume there is a conflict beforehand
    noConflictDetected = false;
    
    //for each neighbor
    for(auto& n : neighbors){

        if(!n){
            continue;
        }

        switch(environment->detectConflict(this, n)){
            case 0:
                /*NO CONFLICT IS DETECTED*/
                noConflictDetected = true;
                this->resumeRobot();
                this->findFollowers();
                //check if a follower has a longer path than the robot's and give way if so
                for(const auto& f : followers){
                    if(f->getPath().size()>path.size()){
                        std::cout<<id<<"[GIVING WAY TO FOLLOWER]"<<f->getID()<<"\n";
                        giveWay();
                        break;
                    }
                }
                break;
            case 1:
                /*OPPOSITE CONFLICT*/
                std::cout<<"[OPPOSITE CONFLICT BETWEEN] <"<<id<<" AND "<<n->getID()<<">"<<std::endl;
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
    /*finds followers among neighboring robots*/
    follower = nullptr;
    followers.clear();
    followers.reserve(8); //a robot can only have 8 followers (current cell has only 8 neighboring cells)
    numberFollowers = 0; /*assume there are no followers before searching for them*/

    /*for any neighboring robots*/
    for(auto& n : neighbors){
        if(!n){
            continue;
        }

        if(n->step() == currentCell && n->step()!=n->getGoal()){ 
            /*check if the robot's next cell is in my history of cells, then set as follower*/
            if(!isInFollowerChain(n)){
                /*if the robot is already in the chain of followers, don't set it as my follower*/
                this->follower = n;
                followers.push_back(n);
                numberFollowers++;
            }
        }

        if(n == follower){
            //if n was a follower and will not move to the next cell, it is not a follower anymore
            if(n->step() != currentCell){
                follower = nullptr;

                auto it = std::find(followers.begin(), followers.end(), n);
                followers.erase(it);
            }

            //count followers of follower and add them up to the current robot's
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
    }
}

bool Robot::isInFollowerChain(Robot* candidate){
    /*check if the candidate is already in the following chain, for example:
        1->2->4
    where 'r1->r2' indicates r1 follows r2.
    if candidate = 4, it is in the chain, and it won't be allowed to follow 1*/

    Robot* current = this;
    std::unordered_set<Robot*> visited;

    /*iterate through follower chain*/
    while (current) {
        if (visited.count(current) > 0) {
            /*if that robot has been visited, there is a loop in follower chain*/
            return true;
        }
        visited.insert(current);
        current = current->follower; //move to next robot in the chain
    }

    return visited.count(candidate) > 0;
}

bool Robot::isInPath(Cell* c){
    /*returns true when a cell is in the path*/
    return std::find(path.begin(), path.end(), c) != path.end();
}

Robot* Robot::determinePriority(Robot* r1, Robot* r2) {
    /* check which robot has the highest priority and return it 
    counts rule activations too
    */

    //determine critical node (common node between the robots)
    Cell* criticalNode = (r1->step() == r2->step()) ? r1->step() : nullptr;

    /* priority rule number 1: robot occupying critical node is given priority */
    if(!(r1->getCurrentCell() == r2->step() && r1->step() == r2->getCurrentCell())){
        if (r1->getCurrentCell() == criticalNode){
            ++r1->ruleCount[0];
            return r1;
        }
        if (r2->getCurrentCell() == criticalNode){
            ++r2->ruleCount[0];
            return r2;
        }
    }

    /* priority rule number 2: a robot giving way to another robot is given priority */
    if (r1->isGivingWay()){
        ++r1->ruleCount[1];
        return r1;
    }
    if (r2->isGivingWay()){
        ++r2->ruleCount[1];
      return r2;  
    } 
    
    /* priority rule number 3: the robot with the highest nFollowers is given priority */
    if (r1->getNFollowers() > r2->getNFollowers()){ 
        ++r1->ruleCount[2];
        return r1;
    }
    if (r2->getNFollowers() > r1->getNFollowers()){
        ++r2->ruleCount[2];
        return r2;
    }

    bool r1_hasNode = r1->findGiveWayNode();
    bool r2_hasNode = r2->findGiveWayNode();

    /* priority rule number 4: a robot having a free neighboring node is given priority */
    if(r1_hasNode && !r2_hasNode){
        ++r1->ruleCount[3];
        return r1;
    }
    else if(r2_hasNode && !r1_hasNode){
        ++r2->ruleCount[3];
        return r2;
    }

    /* priority rule number 5: the robot with the highest neighborsRequestingMyNode is given priority */
    if (r1->getNeighborsRequestingNode() > r2->getNeighborsRequestingNode()){
        ++r1->ruleCount[4];
        return r1;
    }
    if (r2->getNeighborsRequestingNode() > r1->getNeighborsRequestingNode()){
        ++r2->ruleCount[4];
        return r2;
    }

    /* priority rule number 6: the robot with the longest path is given priority */
    if (r1->getPath().size() > r2->getPath().size()){
        ++r1->ruleCount[5];
        return r1;
    }
    if (r2->getPath().size() > r1->getPath().size()){ 
        ++r2->ruleCount[5];
        return r2;
    }

    /* Default case, if none of the above rules apply*/
    return r1;
}

void Robot::solveIntersectionConflict(Robot* n){
    /*solves intersection conflict with robot n*/

    /*first, we must determine which robot has priority*/
    auto priorityRobot = determinePriority(this, n);
    auto nonPriorityRobot = (priorityRobot == this) ? n : this;

    //path of the robot with priority in this conflict
    std::vector<Cell*> priorityPath = priorityRobot->getPath();

    //if the robot with priority has a path of less than 2 cells and it is not at it's goal, wait for the robot with less priority
    if(priorityPath.size()<2 && !priorityRobot->atGoal()){
        priorityRobot->stopRobot();
        return;
    }
    else if(priorityRobot->atGoal()){
        //if it if at it's goal, give way to the robot and return to goal
        priorityRobot->giveWay();
        return;
    }

    Robot* aheadRobot = priorityPath[1]->getObjID(); /*points to the robot occupying n(t+2) of high priority robot*/

    /*based on that, the robot of more priority checks if it's n(t+2) node is free*/
    if(!aheadRobot){
        //if the robot with the lowest priority is not occupying the next node in the priority path, it'll give it's way and not move to the conflicting node
        if(nonPriorityRobot->getCurrentCell() != priorityPath[0]){
            std::cout<<"Here"<<std::endl;
            nonPriorityRobot->giveWay();
            nonPriorityRobot->resumeRobot();
            return;
        }
        /*if the node is free, the low priority robot and all robots requesting the node stop*/
        nonPriorityRobot->stopRobot();
        for(auto r : priorityRobot->neighborsRequestingNode){
            if(!r){
                continue;
            }
            r->stopRobot();
            r->noConflictDetected = false;//the robot is in a conflict, this avoids the agent accidentally resuming movement
        }
    }
    else if(aheadRobot){
        //if there is a robot in the node, it must give it's way
        if(aheadRobot->findGiveWayNode()){
            aheadRobot->giveWay();
        }
        else{
            /*if no valid give way node is found, ask a neighborign robot to move out of the way*/
            std::vector<Robot*> aheadNeighbors = aheadRobot->getNeighbors();
            for(auto& n : aheadNeighbors){
                if(!n){
                    continue;
                }
                if(n->findGiveWayNode()){
                    //only insert a neighbor's cell that is within moving range to avoid unreachable positions
                    if(environment->cellDistance(*aheadRobot->getCurrentCell(), *n->getCurrentCell())<2.0f){
                        n->giveWay();
                        aheadRobot->insertCell(n->getCurrentCell());
                        break; //if a valid cell is found, stop asking neighbors for one
                    }
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
        std::cout<<lowPriorityRobot->getID()<<" [OP LOW PRIO GIVE WAY]\n";
        lowPriorityRobot->giveWay();
    }
    else if(priorityRobot->findGiveWayNode()){
        std::cout<<priorityRobot->getID()<<" [OP HIGH PRIO GIVE WAY]\n";
        priorityRobot->giveWay();
    }
    else{
        if(followers.size()>0){
            for(auto& f:followers){
                if(f->findGiveWayNode()){
                    if(environment->cellDistance(*lowPriorityRobot->getCurrentCell(), *f->getCurrentCell())<2.0f){
                            std::cout<<f->getID()<<"[FOLLOWER GIVING WAY TO]"<<lowPriorityRobot->getID()<<"\n";
                            f->giveWay();
                            lowPriorityRobot->insertCell(f->getCurrentCell());
                            break;
                    }
                }
            }
        }
    }
}

void Robot::giveWay(){
    /*ROBOT INSERTS AVAILABLE GIVE WAY NODE INTO PATH*/

    if(findGiveWayNode()){
        //if a valid neighboring node is found, set it as the give way node
        giveWayNode = freeNeighboringNode;

        //if the node is already in the path it makes no sense to insert it again
        if(isInPath(giveWayNode)){
            givingWay = true;
            return;
        }

        const std::vector<Cell*>& nextNeighbors = this->step()->getNeighbors();

        /*returns true if the cell can be reached from the next cell in the path*/
        auto isReachable = [this, &nextNeighbors]() -> bool {
            return std::find(nextNeighbors.begin(), nextNeighbors.end(), giveWayNode) != nextNeighbors.end();
        };

        /*if we cant reach next cell in path from givewaynode, return to current cell after giving way*/
        if((!isReachable() || atGoal())){
            path.insert(path.begin(), currentCell);
        }

        //insert the new position and mark the state of the robot as giving way
        path.insert(path.begin(), giveWayNode);
        givingWay = true;

        std::cout<<id<<"[GAVE IT'S WAY] "<<*giveWayNode<<std::endl;
    }
}

Cell* Robot::step(){
    /*return next node in the path of the robot*/
    if(path.empty()){
        return currentCell;
    }
    return path.front();
}

const std::vector<Cell*>& Robot::getPath(){
    //returns robot's path
    return this->path;
}

const std::vector<Cell*>& Robot::getArea(){
    //returns robot's detection area
    return this->detectionArea;
}

const std::vector<Robot*>& Robot::getNeighbors(){
    //returns robot's neighbors
    return this->neighbors;
}

void Robot::stopRobot(){
    //makes the robot's planned action to wait
    this->plannedAction = 0;
}

void Robot::resumeRobot(){
    //makes the robot's planned action to move
    noConflictDetected = true;
    this->plannedAction = 1;
}

void Robot::logPath(){
    std::cout<<"Path for "<<id<<":"<<std::endl;
    for(auto& c : path){
        std::cout<<*c<<",";
    }
    std::cout<<"\n";
}

void Robot::logHistory(){
    //prints path history of the robot
    std::cout<<"History for "<<id<<":"<<std::endl;
    for(auto& c : pathHistory){
        std::cout<<*c<<",";
    }
    std::cout<<std::endl;
}

bool Robot::atGoal(){
    //returns true if the robot is at it's goal
    return done;
}

bool Robot::setGoal(Cell* goalPos){
    /*sets the goal of a robot if it is not equal to it's current cell*/
    if(goalPos != currentCell){
        goal = goalPos;
        environment->addGoal(goalPos->getID());
        this->generatePath();
        return true;
    }
    return false;
}

void Robot::removeGoal(){
    //removes the goal of a robot if it has one
    if(!goal){
        return;
    }
    environment->removeGoal(goal->getID());
    goal = nullptr;
    this->generatePath();
}

bool Robot::isMoving(){
    //returns 1 if the planned action is to move
    return plannedAction;
}

bool Robot::isGivingWay(){
    //returns true if the robot is giving way to another robot
    return givingWay;
}

int Robot::getID(){
    //return a robot's id
    return id;
}

int Robot::getNFollowers(){
    //return the number of followers a robot has
    return numberFollowers;
}

int Robot::getNeighborsRequestingNode(){
    //return how many neighbors are requesting this robot's node
    return neighborsRequestingNode.size();
}

const std::array<int, 6> Robot::getRuleCount(){
    //return how many times each rule has been activated
    return ruleCount;
}

const std::array<int,2> Robot::getConflictCount(){
    //return how many conflicts the robot has been in
    return conflictCount;
}

size_t Robot::totalRequests(){
    //return how many times the robot has had it's node requested
    return nodeRequests;
}

float Robot::relativePathSize(){
    //returns the relative path size between the cells visited and the planned path's length

    if(pathLength<=0){
        std::cout<<"NO PATH??"<<std::endl;
        return 0;
    }

    return pathHistory.size()/pathLength;
}

Cell* Robot::getCurrentCell(){
    //return a robot's current cell
    return this->currentCell;
}

Cell* Robot::getGoal(){
    //return a robot's goal
    return this->goal;
}
