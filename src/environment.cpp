#include "environment.h"
#include "robot.h"

void Env::createGrid(){
    float isObstacle = 1.0f;

    for(int i = 0; i < this->rows; i++){
        for(int j = 0; j < this->cols; j++){
            int currentIdx = i * this->cols + j;

            cells.emplace_back(std::make_shared<Cell>(currentIdx, i, j)); /*avoids creating unnecesary copy of Cell*/
            isObstacle = obstacleDist(rng);
            if(isObstacle < this->obstacleProbability){
                cells.back()->setType(cellType::CELL_OBSTACLE);
            }
        }
    }

    this->connectCells();
    this->randomizeRobots();
}

void Env::connectCells(){
    /*directions of cells to be added*/
    std::vector<std::pair<int, int>> directions = {
        {0,-1},{0,1},{-1,0},{1,0},
        {-1,-1},{-1,1},{1,-1},{1,1}
    };

    /*add edges to the graph*/
    for(int i = 0; i<this->rows; i++){
        for(int j = 0; j<this->cols; j++){

            int currentIdx = i * this->cols+j;

            /*add neighbor on each direction*/
            for(auto &dir : directions){
                int ni = i + dir.first;
                int nj = j + dir.second;

                int connectionCost = (abs(dir.first) == abs(dir.second)) ? 2 : 1; /*cost of two if cells are diagonal, 1 if not*/

                if(ni>=0 && ni<this->rows && nj>=0 && nj<this->cols){
                    int neighborIdx = ni * this->cols+nj;
                    addEdge(currentIdx, neighborIdx, connectionCost);
                }
            }
            cells[currentIdx]->updateNeighbors(adjMatrix, *this);
        }
    }
}

void Env::updateEnvironment(){

    checkedConflicts.clear();

    float isObstacle = 1.0f;
    int randomID = 0;
    isObstacle = obstacleDist(rng);
    srand(time(NULL));
    randomID = rand()%cells.size();

    /*if the simulation is paused don't update environment*/
    if(!running){
        return;
    }

    /*add transient obstacle*/
    /* if(isObstacle<transientProbability){
        addObstacle(randomID);
    }
    else{
        removeObstacle(randomID);
    } */

    /*update all cell neighbors to match added and removed obstacles*/
    for(auto& c : cells){
        c->updateNeighbors(adjMatrix, *this);
    }

    /*all robots in the environment shall take an action, be it wait or move*/
    for(auto& r : robots){
        if(!r->atGoal()){
            r->updateDetectionArea(); /*update robot detection area after updating cell neighbors*/
            r->takeAction();
        }
    }
}

bool Env::isConnected(Cell& cell1, Cell& cell2){
    if(adjMatrix[cell1.getID()][cell2.getID()]){
        return true;
    }
    return false;
}

bool Env::isRunning(){
    return running;
}

int Env::connectionCost(Cell& cell1, Cell& cell2){
    return adjMatrix[cell1.getID()][cell2.getID()];
}

void Env::addEdge(int id1, int id2, int cost){
    if(getCellByID(id1)->isObstacle() || getCellByID(id2)->isObstacle()){
        return;
    } /*don't add edge if one of the cells is an obstacle*/

    adjMatrix[id1][id2]= cost;
    adjMatrix[id2][id1]= cost;
}

void Env::removeEdge(int id1, int id2){
    adjMatrix[id1][id2]= 0;
    adjMatrix[id2][id1]= 0;
}

void Env::addObstacle(int id){
    if(id<0 || id>=(this->cols*this->rows) || cells[id]->getObjID()!=nullptr || cells[id]->isGoal()){
        return;
    }

    /*remove all neighbors of current cell and remove current cell from all neighbor list*/
    cells[id]->setType(cellType::CELL_OBSTACLE);
    for(int i = 0; i<this->cols*this->rows;i++){
        removeEdge(id, i);
        cells[i]->updateNeighbors(adjMatrix, *this);
    }

    cells[id]->updateNeighbors(adjMatrix, *this);

}

void Env::removeObstacle(int id){
    if(id<0 || id>=(this->cols*this->rows)){
        return;
    }

    if(!cells[id]->isObstacle()){
        return;
    }

    cells[id]->setType(cellType::CELL_FREE);

    std::vector<std::pair<int, int>> directions = {
        {0,-1},{0,1},{-1,0},{1,0},
        {-1,-1},{-1,1},{1,-1},{1,1}
    };

    /*get the row and col of the cell*/
    int i = id / this->cols;
    int j = id % this->cols;

    /*add the neighbors to cell and update neighbors of surrounding cells to include it*/
    for(auto &pair : directions){
        int ni = i + pair.first;
        int nj = j + pair.second;

        int connectionCost = (abs(pair.first) == abs(pair.second)) ? 2 : 1;

        if(ni>=0 && ni<this->rows && nj>=0 && nj<this->cols){
            int neighborIdx = ni*this->cols + nj;

            addEdge(id, neighborIdx, connectionCost);
            cells[neighborIdx]->updateNeighbors(adjMatrix, *this);
        }
    }

    /*update neighbor list of current cell*/
    cells[id]->updateNeighbors(adjMatrix, *this);
}

/*ROBOT MANIPULATION FUNCTIONS*/
int Env::placeRobot(Robot* r){
    std::array<int, 2> robotPos = r->getPos();
    int posID = robotPos[0] * cols + robotPos[1]; /*id of cell at robot position*/

    if(posID<0 || posID>=cells.size()){
        return -1;
    }

    if(cells[posID]->isObstacle()){
        removeObstacle(posID);
    }

    cells[posID]->setObjID(r);
    return 0;
}

int Env::moveRobot(Robot* r, std::shared_ptr<Cell> nextCell){
    /*never move robot if the next cell already has a robot in it*/
    if(nextCell->getObjID()){
        return -1;
    }

    std::shared_ptr<Cell> currentCell = r->getCurrentCell();

    /* get neighbors of current cell and check if next cell is in them */
    std::vector<std::shared_ptr<Cell>> currNeighbors = currentCell->getNeighbors();

    auto it = std::find_if(currNeighbors.begin(), currNeighbors.end(),[nextCell](std::shared_ptr<Cell> neighbor) {
        return neighbor == nextCell;
    });

    /* move the robot when cells are neighbors */
    if(it != currNeighbors.end()){

        currentCell->setObjID(nullptr);
        nextCell->setObjID(r);

        return 0;
    }
    else{
        return -1;
    }
}

void Env::remakePaths(){
    this->pauseSim();
    for(auto& r : robots){
        r->generatePath();
    }
    this->resumeSim();
}

void Env::addGoal(int id){
    /*if the cell was an obstacle, remove it from the environment to redo connections*/
    if(cells[id]->isObstacle()){
        removeObstacle(id);
    }

    cells[id]->setType(cellType::CELL_GOAL);
}

void Env::removeGoal(int id){
    std::cout<<"Removing goal from: "<<id<<std::endl;
    cells[id]->setType(cellType::CELL_FREE);
}

void Env::randomizeRobots(){
    
    std::shared_ptr<Cell> randomCell;
    std::array<int, 2> rPos = {0,0};
    Vector2 rGoal;
    int placedRobots = 0;

    srand(time(NULL));

    while(placedRobots<nRobots){

        randomCell = cells[rand()%cells.size()];
        rPos = randomCell->getPos();

        if(randomCell->getObjID()){
            continue;
        }

        robots.emplace_back(std::make_shared<Robot>(rPos[0],rPos[1], this));
        randomCell = cells[rand()%cells.size()];

        rPos = randomCell->getPos();
        rGoal = {static_cast<float>(rPos[0]), static_cast<float>(rPos[1])};

        this->placeRobot(robots.back().get());

        robots.back()->setGoal(rGoal);

        placedRobots++;

    }
}

int Env::detectConflict(Robot* r1, Robot* r2){

    /*ROBOTS SOLVE THE CONFLICTS INTERNALLY*/

    int conflictType = 0; /*assume no conflict exists*/

    auto makeOrderedPair = [](Robot* ri, Robot* rj){
        return (ri->getID() < rj->getID()) ? std::make_pair(ri,rj) : std::make_pair(rj,ri);
    };

    std::pair<Robot*, Robot*> confPair = makeOrderedPair(r1,r2);
    if(checkedConflicts.find(confPair) != checkedConflicts.end()){
        return -1;
    }
    checkedConflicts.insert(confPair);
    std::cout<<"Conflict checked for "<<r1->getID()<<" and "<<r2->getID()<<std::endl;

    /*collect the necessary info to check for conflicts*/
    std::shared_ptr<Cell> r1_curr = r1->getCurrentCell();
    std::shared_ptr<Cell> r1_nn = r1->step();
    std::vector<std::shared_ptr<Cell>> r1_remNodes = r1->getPath();
    int r1_nFollowers = r1->getNFollowers();

    std::shared_ptr<Cell> r2_curr = r2->getCurrentCell();
    std::shared_ptr<Cell> r2_nn = r2->step();
    std::vector<std::shared_ptr<Cell>> r2_remNodes = r2->getPath();
    int r2_nFollowers = r2->getNFollowers();
    
    /*OPPOSITE CONFLICT*/
    if(r1_curr == r2_nn && r1_nn == r2_curr){
        conflictType = 1;
    }

    /*INTERSECTION CONFLICT*/
    if(r1_nn == r2_nn){
        conflictType = 2;
    }
    
    return conflictType;
}

void Env::onClick(int id){

    if(id<0 || id>cells.size()){
        return;
    }

    if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
        if(cells[id]->getObjID()){ /*if the cell has a robot, select that robot*/
            selectedRobot = cells[id]->getObjID();
            std::cout<<"Selected robot "<<selectedRobot->getID()<<std::endl;
        }
        else if (selectedRobot && !selectedRobot->getGoal()){ /*if the selected robot doesn't have a goal already*/
            selectedRobot->setGoal((Vector2){static_cast<float>(id / cols), static_cast<float>(id % cols)});
        }
        else if(!selectedRobot){/*if there is no selected robot*/
            addObstacle(id);
        }
    }

    if(IsMouseButtonDown(MOUSE_BUTTON_RIGHT)){
        if(selectedRobot){ /*if there is a selected robot*/
            std::cout<<"Unselected robot "<<selectedRobot->getID()<<std::endl;
            selectedRobot = nullptr; /*if the cell is not a goal but a robot is selected, un-select the robot*/
        }
        else{
            removeObstacle(id);
        }
    }

    if(IsKeyPressed(KEY_DELETE)){
        if(selectedRobot){
            selectedRobot->removeGoal();
        }
    }
}
void Env::pauseSim(){
    running = false;
}

void Env::resumeSim(){
    running = true;
}

std::shared_ptr<Cell> Env::getCellByID(int id){
    return cells[id];
}

std::shared_ptr<Cell> Env::getCellByPos(int x, int y){
    return cells[x * cols + y];
}

std::array<int, 2> Env::getDims(){
    std::array<int,2> eDims{this->rows, this->cols};
    return eDims;
}

std::array<int, 2> Env::cellDims(){
    std::array<int,2> cDims{static_cast<int>(this->cellWidth), static_cast<int>(this->cellHeight)};
    return cDims;
}

std::array<int, 2> Env::origin(){
    std::array<int,2> ox{this->originX, this->originY};
    return ox;
}

int Env::cellDistance(Cell& cell1, Cell& cell2){

    std::array<int, 2> pos1 = cell1.getPos();
    std::array<int, 2> pos2 = cell2.getPos();

    int mDistance = abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]); /*manhattan distance between cells*/

    return mDistance;
}

std::vector<std::shared_ptr<Cell>> Env::getCells(){return cells;}
std::vector<std::shared_ptr<Robot>> Env::getRobots(){return robots;}

void Env::logAdj(){
    for(auto v : adjMatrix){
        for(int element : v){
            std::cout<<element<<", ";
        }
        std::cout<<std::endl;
    }
}

/* DRAWING THE ENVIRONMENT */
void GridRenderer::draw(Env& env, float t) {
    std::array<int, 2> eDims = env.getDims();
    std::array<int, 2> cDims = env.cellDims();
    std::array<int, 2> ox = env.origin();

    int cellWidth = cDims[0];
    int cellHeight = cDims[1];

    const std::vector<std::shared_ptr<Cell>>& cells = env.getCells();
    const std::vector<std::shared_ptr<Robot>>& robots = env.getRobots();

    std::unordered_set<std::shared_ptr<Cell>> detectionCells;
    int offsetY = 0;
    int spacing = 30;

    Color rColor = BLACK;

    for (auto& robot : robots) {

        rColor = colorFromID(robot->getID());

        if(robot->atGoal()){
            DrawText(TextFormat("Robot %d at goal", robot->getID()), 10,20+offsetY, 20, rColor);
        }
        else{
            if(!robot->isMoving()){
                DrawText(TextFormat("Robot %d is waiting", robot->getID()), 10, 20 + offsetY, 20, rColor);
            }
            else{
                DrawText(TextFormat("Robot %d is moving", robot->getID()), 10, 20 + offsetY, 20, rColor);
            }
        }

        offsetY += spacing;

        const std::vector<std::shared_ptr<Cell>>& rDetectArea = robot->getArea();
        detectionCells.insert(rDetectArea.begin(), rDetectArea.end());
    }
    
    DrawText(TextFormat("%f", t), envWidth-200, 20, 20, BLACK);

    int nextX, nextY = ox[1];

    for (int i = 0; i < eDims[0]; i++) {
        nextX = ox[0];

        for (int j = 0; j < eDims[1]; j++) {
            const std::shared_ptr<Cell>& cell = cells[i * eDims[1] + j];
            Rectangle rect = { (float)nextX, (float)nextY, (float)cellWidth, (float)cellHeight };
            float radius = rect.width / 2;

            if (detectionCells.find(cell) != detectionCells.end()) {
                DrawRectangle(rect.x, rect.y, rect.width, rect.height, ORANGE);
            }

            if (cell->isObstacle()) {
                DrawRectangle(rect.x, rect.y, rect.width, rect.height, GRAY);
            }

            if (cell->isGoal()) {
                //std::cout << "Drawing goal at cell: " << cell->getID() << std::endl;
                DrawCircle(rect.x + rect.width / 2, rect.y + rect.height / 2, radius / 2, GREEN);
            }

            if (cell->getObjID() != nullptr) {
                rColor = colorFromID(cell->getObjID()->getID());
                DrawCircle(rect.x + rect.width / 2, rect.y + rect.height / 2, radius, rColor);
            }

            DrawRectangleLines(rect.x, rect.y, rect.width, rect.height, BLACK);
            nextX += cellWidth;
        }

        nextY += cellHeight;
    }

}
