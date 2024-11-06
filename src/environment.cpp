#include "environment.h"
#include "robot.h"

/*PRIVATE FUNCTIONS*/
void Env::randomGrid(){
    this->resetEnv(rows, cols,true);
    this->randomizeRobots();
}

void Env::connectCells() {
    std::vector<std::pair<int, int>> directions = {
        {0, -1}, {0, 1}, {-1, 0}, {1, 0},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            int currentIdx = i * this->cols + j;

            for (auto &dir : directions) {
                int ni = i + dir.first;
                int nj = j + dir.second;

                if (ni >= 0 && ni < this->rows && nj >= 0 && nj < this->cols) {
                    int neighborIdx = ni * this->cols + nj;
                    addEdge(currentIdx, neighborIdx, (abs(dir.first) == abs(dir.second)) ? 0 : 1);
                }
            }
            cells[currentIdx]->updateNeighbors(adjMatrix, *this);

        }
    }
}


/*SIMULATION RELATED FUNCTIONS*/
void Env::pauseSim(){
    running = false;
}

void Env::resumeSim(){
    running = true;
}

bool Env::onClick(int id){

    bool success = true;

    if(IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)){
        if(IsKeyDown(KEY_S)){
            this->dump_map();
        }
        else if(IsKeyDown(KEY_O)){
            std::string path;
            std::cout<<"Provide the path to the map you want to read: ";
            std::cin>>path;
            this->load_map(path);
        }
        else if(IsKeyDown(KEY_LEFT_SHIFT)){
            if(IsKeyDown(KEY_R))
                this->resetEnv(rows,cols, false);
        }
        else if(IsKeyDown(KEY_R)){
            this->randomGrid();
        }
    }

    if(IsKeyPressed(KEY_R)){
        remakePaths();
    }

    if(IsKeyPressed(KEY_SPACE)){
        if(isRunning()){
            pauseSim();
        }
        else{
            resumeSim();
        }
    }

    if(IsKeyPressed(KEY_DELETE)){
        if(selectedRobot){
            selectedRobot->removeGoal();
        }
    }

    if(id<0 || id>cells.size()){
        return false;
    }

    if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
        if(cells[id]->getObjID()){ /*if the cell has a robot, select that robot*/
            selectedRobot = cells[id]->getObjID();
            std::cout<<"Selected robot "<<selectedRobot->getID()<<std::endl;
        }
        else if (selectedRobot && !selectedRobot->getGoal()){ /*if the selected robot doesn't have a goal already*/
            selectedRobot->setGoal(cells[id]);
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
    
    if(IsKeyPressed(KEY_G)){
        std::shared_ptr<Robot> R = std::make_shared<Robot>(cells[id], this);

        robots.push_back(R);
        this->placeRobot(R.get());

        selectedRobot = R.get();
    }

    return true;

}


/*ENVIRONMENT MODIFICATION*/
void Env::updateEnvironment(){
    /*if all robots are at goal, pause the simulation*/
    if(robotsAtGoal.size() == nRobots && nRobots!=0){
        std::cout<<"All robots at goal, pausing simulation"<<std::endl;
        this->pauseSim();
    }

    /*if the simulation is paused don't update environment*/
    if(!running){
        return;
    }

    /*add transient obstacle*/

    /*all robots in the environment shall take an action, be it wait or move*/
    for(auto& r : robots){
        if(r){
            r->updateDetectionArea();
            r->findFollowers();
            r->fetchNeighborInfo();
            r->takeAction();
            if(r->atGoal()){
                robotsAtGoal.insert(r);
            }

            /*if a robot is saved as a robot at goal but is no longer at his goal, remove it from the set*/
            if(robotsAtGoal.find(r) != robotsAtGoal.end() && !r->atGoal()){
                robotsAtGoal.erase(r);
            }
        }
    }
}

void Env::addObstacle(int id){
    if(id < 0 || id >= (this->cols * this->rows) || cells[id]->getObjID() != nullptr || cells[id]->isGoal()) {
        return;
    }


    cells[id]->setType(cellType::CELL_OBSTACLE);
    updateNeighborConnections(id, false);
}

void Env::removeObstacle(int id){
    if(id<0 || id>=(this->cols*this->rows)){
        return;
    }

    if(!cells[id]->isObstacle()){
        return;
    }

    cells[id]->setType(cellType::CELL_FREE);
    updateNeighborConnections(id, true);
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

int Env::load_map(std::string& path){

    this->pauseSim();

    std::ifstream mapFile(path);
    if(!mapFile.is_open()){
        std::cout<<"File not found"<<std::endl;
        return -1;
    }

    std::cout<<"LOADING "<<path<<std::endl;

    std::string line;
    std::string obstacleList, robotList, strDims;
    bool obstaclesFound = false, robotsFound = false, dimsFound = false;

    while(std::getline(mapFile, line)){
        if(line.rfind("obstacles = ", 0) == 0){
            obstacleList = line;
            obstaclesFound = true;
        }
        if(line.rfind("robots = ", 0) == 0){
            robotList = line;
            robotsFound = true;
        }
        if(line.rfind("dims = ", 0) == 0){
            strDims = line;
            dimsFound = true;
        }
    }

    if(!obstaclesFound || !robotsFound){
        std::cout<<"Missing necessary data in the map file"<<std::endl;
        return -1;
    }

    if(dimsFound){
        size_t start = strDims.find("[") + 1;
        size_t end = strDims.find("]");
        std::string mapDims = strDims.substr(start, end-start);
        size_t commaPos = mapDims.find(',');

        int nRows = std::stoi(mapDims.substr(0, commaPos));
        int nCols = std::stoi(mapDims.substr(commaPos+1));

        this->resetEnv(nRows, nCols, false);
    }
    else{
        this->resetEnv(rows,cols,false);
    }

    std::vector<int> obstacleIds;
    std::string numbers = obstacleList.substr(obstacleList.find('[') + 1, obstacleList.find(']') - obstacleList.find('[') - 1);
    std::stringstream ssObs(numbers);
    std::string cellID;

    while (std::getline(ssObs, cellID, ',')) {
        obstacleIds.push_back(std::stoi(cellID));
        cells[std::stoi(cellID)]->setType(cellType::CELL_OBSTACLE);
    }

    for (int id : obstacleIds) {
        this->addObstacle(id);
    }

    std::string robotsData = robotList.substr(robotList.find('[') + 1, robotList.find(']') - robotList.find('[') - 1);
    std::stringstream ssRob(robotsData);
    std::string robotPair;

    while (std::getline(ssRob, robotPair, ',')) {
        size_t colonPos = robotPair.find(':');
        if (colonPos != std::string::npos) {
            int startID = std::stoi(robotPair.substr(0, colonPos));
            int goalID = std::stoi(robotPair.substr(colonPos + 1));

            std::shared_ptr<Robot> robot = std::make_shared<Robot>(cells[startID], this);
            robots.push_back(robot);
            this->placeRobot(robot.get());

            if (goalID >= 0) {
                robot->setGoal(cells[goalID]);
            }
        }
    }

    return 0;
}

void Env::resetEnv(int nRows, int nCols, bool randomize) {

    this->rows = nRows;
    this->cols = nCols;
    selectedRobot = nullptr;

    for(auto& c : cells){
        if(c->getObjID()){
            c->setObjID(nullptr);
        }
    }

    Robot::resetID();
    robots.clear();
    std::cout<<"robots size: "<<robots.size()<<std::endl;

    robotsAtGoal.clear();
    adjMatrix.clear();
    adjMatrix= std::vector<std::vector<int> >(rows*cols, std::vector<int>(rows*cols, 0));

    cells.clear();
    cells.resize(nRows * nCols);

    for (int i = 0; i < nRows; ++i) {
        for(int j = 0; j<nCols; ++j){
            int currIndx = i*cols+j;
            cells[currIndx] = std::make_shared<Cell>(currIndx, i, j);
            if(randomize){
                float isObstacle = obstacleDist(rng);
                if(isObstacle<obstacleProbability){
                    cells[currIndx]->setType(cellType::CELL_OBSTACLE);
                }
            }
        }
    }

    this->connectCells();
}

void Env::updateNeighborConnections(int id, bool isAdding){

    std::vector<std::pair<int, int>> directions = {
        {0, -1}, {0, 1}, {-1, 0}, {1, 0},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    int row = id / this->cols;
    int col = id % this->cols;

    for (const auto &dir : directions) {
        int newRow = row + dir.first;
        int newCol = col + dir.second;

        if (newRow >= 0 && newRow < this->rows && newCol >= 0 && newCol < this->cols) {
            int neighborID = newRow * this->cols + newCol;

            if (isAdding) {
                addEdge(id, neighborID, (abs(dir.first) == abs(dir.second)) ? 2 : 1);
            } else {
                removeEdge(id, neighborID);
            }
            cells[neighborID]->updateNeighbors(adjMatrix, *this);
        }
    }
    cells[id]->updateNeighbors(adjMatrix, *this);

}


/*ROBOT MANIPULATION FUNCTIONS*/
int Env::placeRobot(Robot* r){
    
    if(r->getCurrentCell()->getObjID()){
        return -1;
    }

    int posID = r->getCurrentCell()->getID(); /*id of cell at robot position*/

    if(posID<0 || posID>=cells.size()){
        return -1;
    }

    if(cells[posID]->isObstacle()){
        removeObstacle(posID);
    }

    cells[posID]->setObjID(r);

/*     std::sort(robots.begin(), robots.end(),[](const std::shared_ptr<Robot>& r1, const std::shared_ptr<Robot>& r2){
        return r1->getID()>r2->getID();
    }); */

    return 0;
}

int Env::moveRobot(Robot* r, std::shared_ptr<Cell> nextCell){
    /*never move robot if the next cell already has a robot in it*/
    if(nextCell->getObjID()){
        return -1;
    }

    /*don't move robot if it wants to go to the same position*/
    if(nextCell == r->getCurrentCell()){
        std::cout<<"stay there pal"<<std::endl;
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

int Env::detectConflict(Robot& r1, Robot& r2){
    /*ROBOTS SOLVE THE CONFLICTS INTERNALLY*/
    int conflictType = 0; /*assume no conflict exists*/
    auto makeOrderedTuple = [](int id1, int id2, int type) {
            return (id1 < id2) ? std::make_tuple(id1, id2, type) : std::make_tuple(id2, id1, type);
    };

    /*collect the necessary info to check for conflicts*/
    std::shared_ptr<Cell> r1_curr = r1.getCurrentCell();
    std::shared_ptr<Cell> r1_nn = r1.step();
    std::vector<std::shared_ptr<Cell>> r1_remNodes = r1.getPath();
    int r1_nFollowers = r1.getNFollowers();

    std::shared_ptr<Cell> r2_curr = r2.getCurrentCell();
    std::shared_ptr<Cell> r2_nn = r2.step();
    std::vector<std::shared_ptr<Cell>> r2_remNodes = r2.getPath();
    int r2_nFollowers = r2.getNFollowers();

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

void Env::remakePaths(){
    this->pauseSim();
    for(auto& r : robots){
        r->generatePath();
    }
}

void Env::addGoal(int id){
    /*if the cell was an obstacle, remove it from the environment to redo connections*/
    if(cells[id]->isObstacle()){
        removeObstacle(id);
    }

    cells[id]->setType(cellType::CELL_GOAL);
}

void Env::removeGoal(int id){
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

        if(randomCell->getObjID()){
            continue;
        }

        robots.emplace_back(std::make_shared<Robot>(randomCell, this));
        randomCell = cells[rand()%cells.size()];


        /*don't place a goal there if it is already a goal cell*/
        do{
            randomCell = cells[rand()%cells.size()];
        }while(randomCell->isGoal());

        this->placeRobot(robots.back().get());
        robots.back()->setGoal(randomCell);

        placedRobots++;
    }
}


/*GETTERS AND SETTERS*/
std::array<int, 2> Env::getDims(){
    /*returns number of rows and cols*/
    std::array<int,2> eDims{this->rows, this->cols};
    return eDims;
}

std::array<int, 2> Env::cellDims(){
    /*returns width and height of each cell*/
    std::array<int,2> cDims{static_cast<int>(this->cellWidth), static_cast<int>(this->cellHeight)};
    return cDims;
}

std::array<int, 2> Env::origin(){
    /*returns origin position of the environment*/
    std::array<int,2> ox{this->originX, this->originY};
    return ox;
}

std::shared_ptr<Cell> Env::getCellByID(int id){
    return cells[id];
}

std::shared_ptr<Cell> Env::getCellByPos(int x, int y){
    return cells[x * cols + y];
}

std::vector<std::shared_ptr<Cell>> Env::getCells(){return cells;}
std::vector<std::shared_ptr<Robot>> Env::getRobots(){return robots;}


/*OTHER ENVIRONMENT INFO*/
bool Env::isRunning(){
    return running;
}

bool Env::isConnected(Cell& cell1, Cell& cell2){
    if(adjMatrix[cell1.getID()][cell2.getID()]){
        return true;
    }
    return false;
}

int Env::connectionCost(Cell& cell1, Cell& cell2){
    return adjMatrix[cell1.getID()][cell2.getID()];
}

int Env::cellDistance(Cell& cell1, Cell& cell2){

    std::array<int, 2> pos1 = cell1.getPos();
    std::array<int, 2> pos2 = cell2.getPos();

    int mDistance = abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]); /*manhattan distance between cells*/

    return mDistance;
}

void Env::logAdj(){
    for(auto v : adjMatrix){
        for(int element : v){
            std::cout<<element<<", ";
        }
        std::cout<<std::endl;
    }
}

void Env::dump_map(){

    std::ofstream mapFile;

    std::string response;
    std::string mapName;

    std::string obstacleIDs = "["; /*cell ids of obstacles*/
    std::string robotIDs = "["; /*cell ids of robots*/
    std::string dims = "dims = [" + std::to_string(rows)+","+std::to_string(cols)+"]\n";

    std::cout<<"Want to save this map? (Y/n) ";
    std::cin>>response;
    if(response == "n" || response == "N"){
        return;
    }

    std::cout<<"what do you want to name this map? ";
    std::cin>>mapName;

    std::fstream dummyFile(mapName);

    if(dummyFile.is_open()){
        std::cout<<"This file already exists, want to overwrite it? (Y/n) ";
        std::cin>>response;
        if(response == "n" || response == "N"){
            std::cout<<"Not saving map"<<std::endl;
            mapFile.close();
            return;
        }
    }

    mapFile.open(mapName);
    for(int i = 0; i<rows*cols; i++){
        Robot* tempRobot;
        if(cells[i]->isObstacle()){
            if(obstacleIDs.size()>1) obstacleIDs.append(",");
            obstacleIDs.append(std::to_string(i));
        }
        if((tempRobot = cells[i]->getObjID())){
            if(robotIDs.size()>1) robotIDs.append(",");
            robotIDs.append(std::to_string(i));
            if(tempRobot->getGoal()){
                robotIDs.append(":"+std::to_string(tempRobot->getGoal()->getID()));
            }
            else{
                std::cout<<"Robot has no goal"<<std::endl;
                robotIDs.append(":"+std::to_string(-1));
            }
        }
    }

    obstacleIDs = std::string("obstacles = ") + obstacleIDs;
    robotIDs = std::string("robots = ") + robotIDs;

    obstacleIDs.append("]\n");
    robotIDs.append("] # starting cell ID : goal ID\n");

    mapFile<<dims;
    mapFile<<obstacleIDs;
    mapFile<<robotIDs;

    std::cout<<"Map dumped succesfully"<<std::endl;
    mapFile.close();

}


/* DRAWING THE ENVIRONMENT */
void GridRenderer::draw(Env& env, float t) {
    
    if(!env.isRunning()){
        DrawText("SIMULATION PAUSED", 10,10,20,RED);
    }

    std::array<int, 2> eDims = env.getDims();

    std::array<int,2> ox = env.origin();

    std::vector<std::shared_ptr<Robot>> robots = env.getRobots();
    std::vector<std::shared_ptr<Cell>> cells = env.getCells();

    float robotRadius = cellH/2;
    float goalRadius = cellH/3;

    for(auto& c : cells){
        Robot* tmpRob;
        std::array<int, 2> pos = c->getPos();
        std::array<int,2> drawPos = {pos[1]*cellW+1+ox[0], pos[0]*cellH+1+ox[1]};

        if (pos[0] == 0) {
            DrawText(TextFormat("%d", pos[1]), drawPos[0] + cellW / 2 - 10, drawPos[1] - 30, 20, WHITE);
        }
        if (c->getID() % eDims[1] == 0) {
            DrawText(TextFormat("%d", pos[0]), drawPos[0] - 25, drawPos[1] + cellH / 2 - 10, 20, WHITE);
        }

        if(c->isObstacle()){
            DrawRectangle(drawPos[0], drawPos[1], cellH, cellW, GRAY);
        }
        else{
            DrawRectangle(drawPos[0], drawPos[1], cellH, cellW, WHITE);
        }

        if(c->isGoal()){
            DrawCircle(drawPos[0]+cellH/2, drawPos[1]+cellW/2, goalRadius, GREEN);
        }

        if((tmpRob = c->getObjID())){
            DrawCircle(drawPos[0]+cellH/2, drawPos[1]+cellW/2, robotRadius, robotColors[c->getObjID()->getID()]);
        }

        DrawRectangleLines(drawPos[0], drawPos[1], cellH, cellW, BLACK);
    }

    int offsetY = 30;
    const float baseHeight = 80; //minium height for display rectangle
    float height = static_cast<float>(55 * robots.size());
    height = (height < baseHeight) ? baseHeight : height;   
    std::string robotState;
    Color robotDisplayColor = {0xeb,0xed,0xeb,255};
    Rectangle displayRect = {30,40,150,height};
    
    DrawRectangle(displayRect.x,displayRect.y,displayRect.width,displayRect.height, robotDisplayColor);
    DrawRectangleLinesEx(displayRect,5.0f, BLACK);
    DrawText(TextFormat("t = %.2f",t),displayRect.x+20,displayRect.y+20, 20, BLACK);

    for(auto& robot : robots){
        int rID = robot->getID();
        if(robot->atGoal()){
            robotState = " is at goal";
        }
        else if(robot->isMoving()){
            robotState = " is moving";
        }
        else{
            robotState = " is waiting";
        }
        DrawText(TextFormat("%d %s", rID, robotState.c_str()), 50,60+offsetY, 20, robotColors[rID]);
        offsetY+=30;
    }
}
