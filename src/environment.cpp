#include "environment.h"
#include "robot.h"

//if this is not defined, simulation results won't be dumped to a .dat file
#define DUMP_RESULTS

Env::Env(int cellW, int cellH, int posX, int posY, int r, int c, int n_robots, float step, const float obsProb, const float dynProb, float robotDelay, const std::string& mapName)
: cellWidth(cellW), cellHeight(cellH), originX(posX), originY(posY), 
rows(r), cols(c), nRobots(n_robots),timeStep(step),obstacleDist(0, 1), obstacleProbability(obsProb), transientProbability(dynProb), robotWaitProb(robotDelay),
mapName(mapName)
{
	rng.seed(static_cast<uint32_t>(std::time(nullptr))); //seed the random number generator

    /*initialize an empty adjacency matrix of size rows*cols*/
	for(int i = 0; i<rows*cols; i++){
        adjMatrix[i] = std::unordered_map<int,float>();
    }

    /*reserve memory for all the cells in the environment and all the robots*/
	cells.reserve(rows*cols);
	robots.reserve(nRobots);

	std::cout<<"transient obstacle probability is: "<<transientProbability<<std::endl;
}

/*PRIVATE FUNCTIONS*/
void Env::randomGrid(){
    /*CREATES A GRID COMPRISED OF RANDOM OBSTACLES AND ROBOTS*/

    bool randomize = (obstacleProbability == 0.0f) ? false : true;

    /*reset the current environment to one of rows*cols dimensions
      if randomize is true, creates random static obstacles
    */
	this->resetEnv(rows, cols,randomize);

    /*place random nRobots*/
    this->randomizeRobots();
}

void Env::connectCells() {
    /*create edges between all cells in the graph and add them to it's neighbors' adjacency lists*/
    static const std::vector<std::pair<int, int>> directions = {
        {0, -1}, {0, 1}, {-1, 0}, {1, 0},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    }; /*each cell is ortogonally and diagonally connected*/

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            size_t currentIdx = i * this->cols + j; /*calculate the id corresponding to the current iteration*/

            /*assert any invalid cell IDs, this should never happen under normal conditions*/
            assert(currentIdx>=0 && currentIdx< cells.size());

            /*iterate through each direction*/
            for (const auto &dir : directions) {
                /*compute position of the neighbor*/
                int ni = i + dir.first;
                int nj = j + dir.second;

                /*if the neighbor has an invalid position, don't connect it*/
                if (ni >= 0 && ni < this->rows && nj >= 0 && nj < this->cols) {
                    /*add edge between currentIDx and it's neighbor
                      cost is sqrt(2) when {1,1} {-1,-1}, {1,-1} and {-1,1}*/
                    int neighborIdx = ni * this->cols + nj;
                    addEdge(currentIdx, neighborIdx, (abs(dir.first) == abs(dir.second)) ? diagonalCost : 1.0f);
                }
            }

            /*update neighbors for the surrounding cells*/
            cells[currentIdx]->updateNeighbors(adjMatrix, *this);
        }
    }
}

/*SIMULATION RELATED FUNCTIONS*/
void Env::pauseSim(){
    //set the simulator as paused
    running = false;
}

void Env::resumeSim(){
    //set the simulator as running
    running = true;
}

void Env::handle_input(unsigned long id){
    /*handles all kind of input events, this is not necessary for the simulation but makes using the simulator easier*/
    if(IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)){
        if(IsKeyDown(KEY_O)){
            std::cout<<"Provide the name of the map you want to read: ";
            std::cin>>mapName;
            this->load_benchmark();
        }
        else if(IsKeyDown(KEY_LEFT_SHIFT)){
            if(IsKeyDown(KEY_R))
                this->resetEnv(rows,cols, false);
        }
        else if(IsKeyPressed(KEY_R)){
            this->randomGrid();
        }
        else if(IsKeyPressed(KEY_D)){
            dumpResults();
        }
    }

    if(IsKeyPressed(KEY_N)){
        const std::vector<Cell*>& cn = cells[0]->getNeighbors();

        for(auto& c : cn){
            std::cout<<*c<<",";
        }
        std::cout<<"\n";
    }

	if(IsKeyDown(KEY_H) && selectedRobot){
		selectedRobot->logHistory();
	}

    if(IsKeyPressed(KEY_P)){
        if(selectedRobot){
            selectedRobot->logPath();
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
        return;
    }

    if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
        if(cells[id]->getObjID() && !selectedRobot){ /*if the cell has a robot, select that robot*/
            selectedRobot = cells[id]->getObjID();
            std::cout<<"Selected robot "<<selectedRobot->getID()<<std::endl;
        }
        else if (selectedRobot && !selectedRobot->getGoal()){ /*if the selected robot doesn't have a goal already*/
            if(selectedRobot->setGoal(cells[id].get())){
                std::cout<<"Placed goal for "<<selectedRobot->getID()<<" at "<<*cells[id].get()<<"\n";
            }
            else{
                std::cout<<"Couldn't place goal"<<std::endl;
            }
        }
        else if(!selectedRobot){/*if there is no selected robot*/
            if(IsKeyDown(KEY_LEFT_CONTROL)){
                cells[id]->setTransient();
                transients.insert(cells[id].get());
            }
            else{
                addObstacle(id, false);
            }
        }
    }

    if(IsMouseButtonDown(MOUSE_BUTTON_RIGHT)){
        if(selectedRobot){ /*if there is a selected robot*/
            std::cout<<"Unselected robot "<<selectedRobot->getID()<<std::endl;
            selectedRobot = nullptr; /*if the cell is not a goal but a robot is selected, un-select the robot*/
        }
        else{
            
            if(cells[id]->isCellTransient()){
                cells[id]->removeTransient();
                transients.erase(cells[id].get());
            }
            else{
                removeObstacle(id);
            }
        }
    }
    
    if(IsKeyPressed(KEY_G)){
        std::unique_ptr<Robot> R = std::make_unique<Robot>(cells[id].get(), this, robotWaitProb);
        selectedRobot = R.get();

        robots.push_back(std::move(R));
        this->placeRobot(robots.back().get());

    }
}

/*ENVIRONMENT MODIFICATION*/
bool Env::updateEnvironment(float t){
    /*UPDATE LOOP OF THE SIMLUATION*/

    //random number generator for cell positions
    static std::random_device gd;
    static std::mt19937 gen(gd());
    static std::uniform_int_distribution<> dis(0,freeCells.size()-1);

    //if 10 time steps have elapsed
    if(static_cast<int>(t/timeStep) % 10 == 0){

        //remove transient obstacles
        clearTransients();
        std::cout<<transients.size()<<std::endl;

        //places a number of cells equal to density percentage of static obstacles
        //static obstacles = total cells - free cells
        int placedCells = 0;
        int nTransients = (cells.size()-freeCells.size()) * transientProbability; //% de obstaculos dinamicos que se coloca

        //place the transient obstacles at cells that are not a transient obstacle and don't have a robot
        while(placedCells<nTransients){
            int randID = dis(gen);
            if(!freeCells[randID]->isCellTransient() && !freeCells[randID]->getObjID()){
                freeCells[randID]->setTransient();
                transients.insert(freeCells[randID]);
                placedCells++;
            }
        }
    }

    std::cout<<"_____________________Updating for time: "<<t<<"_____________________"<<std::endl;
    /*if all robots are at goal or time elapsed is more than 1200 timeSteps*/
    if((robotsAtGoal.size() == robots.size() && robots.size()!=0) || t>=120.0f){
        #ifdef DUMP_RESULTS
            std::cout<<"WRITING AT TIME: "<<t<<std::endl;
            dumpResults();
            this->pauseSim();
        #endif

        return true;
    }

    /*if the simulation is paused don't update environment*/
    if(!running){
        return false;
    }

    for(auto& r : robots){
        if(!r){
            continue;
        }
        r->updateDetectionArea(); /*robots update their detection area*/
        r->findNeighbors(); /*robots identify neighbors*/
        r->findFollowers(); /*robots identify followers*/
    }

    for(auto& r : robots){
        r->fetchNeighborInfo(); /*detect conflict with neighbors*/
        r->takeAction(); /*take an action*/

        /*when a robot reaches it's goal, add it to the map and associate it with it's time of arrival*/
        if(r->atGoal() && robotsAtGoal.find(r.get()) == robotsAtGoal.end()){
            robotsAtGoal[r.get()] = static_cast<int>(std::ceil(t/timeStep));
        }
    }

    return false;
}

void Env::addObstacle(int id, bool isTransient){
    /*MARK A CELL AS AN STATIC OR TRANSIENT OBSTACLE*/
    if(id < 0 || (unsigned long) id >= (cells.size()) || cells[id]->getObjID() != nullptr || cells[id]->isGoal()) {
        return;
    }

    if(cells[id]->isObstacle()){
        return;
    }

    /*CHANGE THE TYPE OF THE CELL*/
    cells[id]->setObstacle();

    /*MARK THE CELL AS A TRANSIENT OBSTACLE*/
    if(isTransient){
        cells[id]->setTransient();
        transients.insert(cells[id].get());
    }

    /*update connections of all neighbors*/
    updateNeighborConnections(id, false);

    if(freeCells.size()>0){
        auto it = std::find(freeCells.begin(), freeCells.end(), cells[id].get());
        if(it!=freeCells.end()){
            freeCells.erase(it);
        }
    }
}

bool Env::removeObstacle(int id){
    /*REMOVE AN OBSTACLE FROM THE ENVIRONMENT*/
    if(id < 0 || id >= (this->cols * this->rows)){
        return false;
    }

    /*if the cell is not an obstacle, ignore it*/
    if(!cells[id]->isObstacle()){
        return false;
    }

    /*set the cell type as free, if it was a transient obstacle, mark it as an static obstacle first*/
    if(cells[id]->isCellTransient()){
        cells[id]->removeTransient();
    }

    cells[id]->removeObstacle();

    /*update the neighbors for the current cell*/
    updateNeighborConnections(id, true);

    /*re-check and update neighbors for neighboring cells
    this shouldn't be necesssary but a bug occurs where a cell is not
    added to the neighbors list of a cell that was an obstacle
    */

    auto& cellNeighbors = cells[id]->getNeighbors();

    for(auto& neighbor : cellNeighbors){
        if(neighbor)
            updateNeighborConnections(neighbor->getID(), true);
    }

    freeCells.push_back(cells[id].get());

    return true;
}

void Env::addEdge(int id1, int id2, float cost){
    /*ADD AN EDGE IN THE ADJACENCY MATRIX BETWEEN TO CELLS*/
    if(getCellByID(id1)->isObstacle() || getCellByID(id2)->isObstacle()){
        return;
    } /*don't add edge if one of the cells is an obstacle*/

    /*assign the edge between cell 1 and cell2 with the provided cost (1 if orthogonal, 1.414 if diagonal)*/
    adjMatrix[id1][id2] = cost;
}

void Env::removeEdge(int id1, int id2){
    /*erase the edge between two cells*/
    adjMatrix[id1].erase(id2);
    /*if the cell has no edges, remove it from the adjacency list*/
    if(adjMatrix[id1].empty()){
        adjMatrix.erase(id1);
    }
}

int Env::load_benchmark() {
    /*loads benchmark map from database*/

    //pause the simulation to load the map
    this->pauseSim();

    //look for the map provided in the maps directory
    std::ifstream mapFile("../maps/"+mapName);
    if (!mapFile.is_open()) {
        std::cerr << "File not found at path ../maps/"<<mapName<< std::endl;
        return -1;
    }

    //look if the format of the map matches
    std::string line;
    int width = 0, height = 0;
    bool readingMap = false;

    while(std::getline(mapFile,line)){
        if(line.rfind("height",0) == 0){
            height = std::stoi(line.substr(line.find(" ")+1));
        }
        else if(line.rfind("width",0) == 0){
            width = std::stoi(line.substr(line.find(" ")+1));
        }
        else if(line == "map" || line.find("map") != std::string::npos){
            readingMap = true;
            break;
        }
    }

    //if the format is wrong, don't load it
    if(!readingMap){
        std::cerr<<"Map data not found!"<<std::endl;
        return -1;
    }

    //if the format is right, get the layout
    std::vector<std::vector<char>> mapData;
    mapData.reserve(height);

    while(std::getline(mapFile, line)){
        if(line.empty()) continue;
        std::vector<char> row(line.begin(), line.end());
        mapData.push_back(row);
    }

    //close the map file
    mapFile.close();

    std::cout<<"{"<<width<<","<<height<<"}";
    std::cout<<"\n{"<<mapData[0].size()<<","<<mapData.size()<<"}"<<std::endl;

    //make sure the dimensions map with the ones in the file
    if ((unsigned long) height != mapData.size() || (height > 0 && (unsigned long) width != mapData[0].size())) {
        std::cerr << "Map dimensions do not match the provided height and width" << std::endl;
        return -1;
    }

    //re-make environment with new dimensions, don't randomize obstacles
    this->resetEnv(height, width, false);

    //place the obstacles based on map layout in the file
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int cellId = y * width + x;
            char cellType = mapData[y][x];
            if (cellType == 'T' || cellType == '@') {
                this->addObstacle(cellId, false);
            }
        }
    }

    //place all random robots after the environment is succesfully set up
    randomizeRobots();
    return 0;
}

void Env::resetEnv(int nRows, int nCols, bool randomize) {
    /*resets the whole environment:
        -deletes obstacles
        -deletes robots
        -clears all transient obstacles
        -overwrites dimensions with nRows and nCols
        -if randomize is true, add random static obstacles
    */

    //overwrite dimensions
    this->rows = nRows;
    this->cols = nCols;
    selectedRobot = nullptr;

    //delete free cells
    freeCells.clear();

    //reset robot IDs before deleting them
    Robot::resetID();
    robots.clear();
    robotsAtGoal.clear();
    
    //clear the adjacency matrix and create a new one with the new dimensions
    adjMatrix.clear();
    for(int i = 0; i<rows*cols; i++){
        adjMatrix[i] = std::unordered_map<int,float>();
    }

    //delete cells and create new vector with new dimensions
    cells.clear();
    cells.reserve(nRows * nCols);

    //place all cells in the cells vector, assume all cells are free
    for (int i = 0; i < nRows; ++i) {
        for(int j = 0; j<nCols; ++j){
            int currIndx = i*cols+j;
            cells.emplace_back(std::make_unique<Cell>(currIndx, i,j));
            freeCells.push_back(cells.back().get());
        }
    }

    //randomly place obstacles in each cell
    if(randomize){
        for(auto& c:cells){
            float isObstacle = obstacleDist(rng);
            if(isObstacle<obstacleProbability){
                addObstacle(c->getID(), false);
            }
        }
    }

    //connect the graph
    this->connectCells();
}

void Env::updateNeighborConnections(int id, bool isAdding){

    /*UPDATE NEIGHBOR CONNECTION OF CELL WITH id AND IT'S NEIGHBORS
      isAdding = true, adds neighbors
      isAdding = false, removes neighbors
    */

    /*same directions approach used in connectCells*/
    static const std::vector<std::pair<int, int>> directions = {
        {0, -1}, {0, 1}, {-1, 0}, {1, 0},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    //compute row and col from id
    int row = id / cols;
    int col = id % cols;

    for (const auto &dir : directions) {
        int newRow = row + dir.first;
        int newCol = col + dir.second;

        if (newRow >= 0 && newRow < rows && newCol >= 0 && newCol < cols) {
            int neighborID = newRow * cols + newCol;

            if (isAdding) {
                /*create an edge between the cell and it's neighbor, and add it to the cell's neighbors cells*/
                addEdge(id, neighborID, (abs(dir.first) == abs(dir.second)) ? diagonalCost : 1.0f);
                cells[neighborID]->addNeighbor(getCellByID(id));
            } else {
                /*remove the edge with the neighbor, and remove the neighbor from the adjacency list*/
                removeEdge(id, neighborID);
                cells[neighborID]->removeNeighbor(getCellByID(id));
            }

            /*update neighbors of the neighbor cell*/
            cells[neighborID]->updateNeighbors(adjMatrix, *this);
        }
    }

    /*update neighbors of the current cell*/
    cells[id]->updateNeighbors(adjMatrix, *this);
}

void Env::clearTransients(){
    /*remove all transient obstacles from the environment*/
    for(auto& c: transients){
        c->removeTransient();
    }
    transients.clear();
}

/*ROBOT MANIPULATION FUNCTIONS*/
int Env::placeRobot(Robot* r){
    /*place a robot at a cell in the environment*/

    /*if the cell already has a robot, do not place it*/
    if(r->getCurrentCell()->getObjID()){
        return -1;
    }

    int posID = r->getCurrentCell()->getID(); /*id of cell at robot position*/

    if(posID<0 || (unsigned long) posID>=cells.size()){
        return -1;
    }

    /*if the cell was an obstacle, remove it*/
    if(cells[posID]->isObstacle()){
        removeObstacle(posID);
    }

    /*set the robot reference ID of the cell to the robot*/
    cells[posID]->setObjID(r);

    return 0;
}

int Env::moveRobot(Robot* r, Cell* nextCell){

    /*UPDATE THE POSITION OF THE ROBOT IN THE ENVIRONMENT*/

    Cell* currentCell = r->getCurrentCell();

    if(nextCell->isCellTransient()){
        return -1;
    }

    /*if it tries to move to it's current cell, move*/
    if(nextCell == currentCell){
        return 0;
    }

    /*never move robot if the next cell already has a robot in it*/
    if(nextCell->getObjID()){
        return -1;
    }

    /* get neighbors of current cell and check if next cell is in them */
    const std::vector<Cell*>& currNeighbors = currentCell->getNeighbors();

    /*lambda to check if the new cell is a neighbor of the current cell*/
    auto it = std::find_if(currNeighbors.begin(), currNeighbors.end(),[nextCell](Cell* neighbor) {
        return neighbor == nextCell;
    });

    /* move the robot when cells are neighbors */
    if(it != currNeighbors.end()){
        /*remove the robot from it's current cell*/
        currentCell->setObjID(nullptr);

        /*set the reference to the robot at the next cell*/
        nextCell->setObjID(r);

        return 0;
    }
    else{
        return -1;
    }
}

int Env::detectConflict(Robot* r1, Robot* r2){

    /*ROBOTS SOLVE THE CONFLICTS INTERNALLY*/
    int conflictType = 0; /*assume no conflict exists*/

    /*collect the necessary info to check for conflicts*/
    Cell* r1_curr = r1->getCurrentCell();
    Cell* r1_nn = r1->step();

    Cell* r2_curr = r2->getCurrentCell();
    Cell* r2_nn = r2->step();

    /*if both robots are at goal, no conflict is detected*/
    if(!r1_nn && !r2_nn){
        conflictType = 0;
    }

    /*OPPOSITE CONFLICT*/
    if(r1_curr == r2_nn && r1_nn == r2_curr){
        conflictType = 1;
    }

    /*INTERSECTION CONFLICT*/
    if(r1_nn == r2_nn && r1_nn && r2_nn){

        if(!r1_nn || !r2_nn){
            return 2;
        }
        
        conflictType = 2;
    }

    return conflictType;
}

void Env::remakePaths(){
    /*RESET AND REMAKE THE PATH OF ALL ROBOTS*/
    this->pauseSim();
    for(auto& r : robots){

        /*if the robot is at it's goal, don't remake the path*/
        if(r->atGoal()){
            continue;
        }

        /*delete the old path and generate a new one, resetPath also sets pathLength to zero*/
        r->resetPath();
        r->generatePath();
    }
}

void Env::addGoal(int id){
    /*MARK A CELL AS A GOAL*/

    /*if the cell was an obstacle, remove it from the environment
    this avoids the robot's goal from not being connected to it's neighbors, making it unreachable*/
    if(cells[id]->isObstacle()){
        removeObstacle(id);
    }

    cells[id]->setGoal();
}

void Env::removeGoal(int id){
    /*REMOVE A GOAL FROM A CELL*/
    cells[id]->removeGoal();
}

void Env::randomizeRobots(){
    /*PLACE RANDOM nRobots WITH RANDOM GOALS IN THE ENVIRONMENT*/

    /*first, remove all robots and it's goals from the environment*/
    for(auto& r:robots){
        removeGoal(r.get()->getGoal()->getID());
        r.get()->getCurrentCell()->setObjID(nullptr);
    }

    /*reset robot identifiers so they start at 1 again*/
    Robot::resetID();
    robots.clear();

    /*reset list of robotsAtGoal and selectedRobot*/
    robotsAtGoal.clear();
    selectedRobot = nullptr;

    Cell* randomCell;
    std::vector<Cell*> randomNeighbors;
    int placedRobots = 0;

    /*vector of free cells to place robots and goals at*/
    std::vector<Cell*> free_env_cells = freeCells;

    /*random number generator, generates int between zero and the number of cells*/
    static std::random_device rd;
    static std::mt19937 rng(rd());
    static std::uniform_int_distribution<int> dist(0, free_env_cells.size() - 1);

    /*while there are less than nRobots in the environment*/
    while(placedRobots<nRobots){

        /*select a random cell and place the robot at the position*/
        int startIndex = dist(rng);

        randomCell = free_env_cells[startIndex];

        free_env_cells.erase(free_env_cells.begin() + startIndex);
        robots.emplace_back(std::make_unique<Robot>(randomCell, this, robotWaitProb));
        this->placeRobot(robots.back().get());

        /*look for a cell that is not a goal and is suitable to be a goal for the robot*/
        int goalIndex;
        do{
            goalIndex = dist(rng);
            randomCell = free_env_cells[goalIndex];
        }while(randomCell->isGoal() || !robots.back()->setGoal(randomCell));

        free_env_cells.erase(free_env_cells.begin()+goalIndex);
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

Cell* Env::getCellByID(int id){
    //returns a cell given it's id
    return cells[id].get();
}

Cell* Env::getCellByPos(int x, int y){
    //returns a cell given a position
    return cells[x * cols + y].get();
}

float Env::getStep(){
    //returns length of the time step
    return timeStep;
}

std::vector<std::unique_ptr<Cell>>& Env::getCells() {
    //return cells of the environment
    return cells;
}

std::vector<std::unique_ptr<Robot>>& Env::getRobots(){
    //return robots in the environment
    return robots;
}

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

float Env::connectionCost(Cell& cell1, Cell& cell2){

    /*RETURNS THE COST OF THE EDGE BETWEEN TWO CELLS*/

    int id1 = cell1.getID();
    int id2 = cell2.getID();

    if(adjMatrix.find(id1) != adjMatrix.end() && adjMatrix.find(id2) != adjMatrix.end()){
        return adjMatrix[id1][id2];
    }
    else{
        return 0;
    }
}

float Env::cellDistance(Cell& cell1, Cell& cell2){
    /*THIS FUNCTION IS USED AS A HEURISTIC FOR THE ROBOTS*/

    /*computes octile distance between two cells*/
    std::array<int, 2> pos1 = cell1.getPos();
    std::array<int, 2> pos2 = cell2.getPos();

    int dx = abs(pos1[0] - pos2[0]);
    int dy = abs(pos1[1] - pos2[1]);

    float octileDistance = std::max(dx,dy) + (sqrt(2)-1)*std::min(dx,dy);

    return octileDistance;
}

void Env::dumpResults(){
    /*SAVE METRICS OF THE SIMULATION TO A FILE WITH THE NAME:
    <mapname>_<numsim>.dat*/

    /*creates .dat file with the results of the simulation*/
    std::ofstream recordFile; 
    std::string robotDataLabels = "#Robot Data\n#Robot_ID #total_time #p_size #n_requests\n";
    std::string globalDataLabels = "\n#Global Data\n#op_conflicts #in_conflicts #rule_1 #rule_2 #rule_3 #rule_4 #rule_5 #rule_6\n";

    size_t mapDotPos = mapName.find_last_of('.');
    const std::string& noext_mapName = (mapDotPos != std::string::npos) ? mapName.substr(0, mapDotPos) : mapName;

    std::string dirPath = "../results/"+noext_mapName+"/";
    if(!fs::exists(dirPath)){
        fs::create_directories(dirPath);
    }

    /*get simulation num from filename <mapname>-simNum.dat*/
    int lastSim = 0;
    for (const auto& entry : fs::directory_iterator("../results/" + noext_mapName + "/")) {
        if (entry.is_regular_file()) {
            const std::string& file = entry.path().string();
            if (file.find(noext_mapName) != std::string::npos) {
                size_t positionDash = file.rfind("_");
                size_t positionExt = file.find(".dat");

                // Ensure positions are valid
                if (positionDash != std::string::npos && positionExt != std::string::npos && positionExt > positionDash) {
                    std::string simNumStr = file.substr(positionDash + 1, positionExt - positionDash - 1);

                    // Check if simNumStr is a valid number
                    if (!simNumStr.empty() && std::all_of(simNumStr.begin(), simNumStr.end(), ::isdigit)) {
                        int simNum = std::stoi(simNumStr);
                        if (simNum > lastSim) {
                            lastSim = simNum;
                        }
                    }
                }
            }
        }
    }

    const std::string& resultsFile = "../results/"+noext_mapName+"/"+noext_mapName+"_"+std::to_string(lastSim+1)+".dat";

    std::cout<<"Saving results to: "<<resultsFile<<std::endl;

    recordFile.open(resultsFile);
    recordFile<<robotDataLabels;

    std::array<int, 6> totalRuleCounts = {0,0,0,0,0,0};
    std::array<int, 2> totalConflicts = {0,0};

    for(auto& r : robots){

        std::string timeSTR;
        if(robotsAtGoal.find(r.get()) == robotsAtGoal.end()){
            timeSTR = "null"; /*not-at-goal*/
        }
        else{
            timeSTR = std::to_string(robotsAtGoal[r.get()]);
        }

        std::stringstream robotRow;
        int robotID = r->getID();
        float p_size = r->relativePathSize();
        size_t nodeRequests = r->totalRequests();

        robotRow<<std::to_string(robotID)<<" "<<timeSTR<<" "<<std::to_string(p_size)<<std::to_string(nodeRequests)<<"\n";
        recordFile<<robotRow.rdbuf();

        std::array<int, 6> robotRuleCounts = r->getRuleCount();
        std::array<int, 2> robotConflictCounts = r->getConflictCount();

        for(size_t i = 0; i<6; i++){
            totalRuleCounts[i] += robotRuleCounts[i];
        }

        for(size_t j = 0; j<2; j++){
            totalConflicts[j] += robotConflictCounts[j];
        }
    }

    recordFile<<globalDataLabels;
    std::stringstream globalRow;
    
    for(size_t j = 0; j<2; j++){
        std::cout<<"Writing conflict counts"<<std::endl;
        globalRow<<std::to_string(totalConflicts[j])<<" ";
    }
    for(size_t i = 0; i<6; i++){
        globalRow<<std::to_string(totalRuleCounts[i])<<" ";
    }
    globalRow<<"\n";

    recordFile<<globalRow.rdbuf();

    recordFile.close();
}

/* DRAWING THE ENVIRONMENT */
Color GridRenderer::colorFromID(int id, int offset){
    /*encode color using robot (or cell) IDs*/

    /*multiply the ID with prime numbers to create a more uniform distribution and minimize repeated colors*/
    unsigned char r = ((id * 57) + offset) % 256;
    unsigned char g = ((id * 37) + offset) % 256;
    unsigned char b = ((id * 97) + offset) % 256;

    return Color{r,g,b,255}; /*keep opacity at max value (255)*/
}

void GridRenderer::setZoom(float zoomFactor){
    zoom = zoomFactor;
    cellW = static_cast<int>(baseW*zoom);
    cellH = static_cast<int>(baseH*zoom);
}

void GridRenderer::pan(float dx, float dy){
    offsetX+=dx;
    offsetY+=dy;
}

void GridRenderer::cacheRobotColors(int nRobots){
    if(nRobots == 0){
            nRobots = 10;
        }

        for(int i = 0; i<nRobots;i++){
            robotColors[i] = colorFromID(i,0);
        }
}

void GridRenderer::draw(Env& env, float t) {

    auto& cells = env.getCells();
    auto& robots = env.getRobots();

    if (robots.size() >= robotColors.size()) {
        cacheRobotColors(robots.size() * 2);
    }

    //get origin point of the environment to compute rendering positions
    auto ox = env.origin();

    //iterate through the cells vector and draw each of them
    for (auto& c : cells) {
        auto pos = c->getPos();
        auto drawPos = std::array<int, 2>{ pos[1] * cellW + 1 + ox[0], pos[0] * cellH + 1 + ox[1] };

        //draw the cell
        Color cellColor = c->isGoal() ? GREEN
                : c->isCellTransient() ? ORANGE
                : c->isObstacle() ? GRAY
                : WHITE;

        DrawRectangle(drawPos[0]+offsetX, drawPos[1]+offsetY, cellW, cellH, cellColor);

        //draw the robot in the cell (if present)
        if (auto tempRobot = c->getObjID()) {
            DrawCircle(drawPos[0] + cellW / 2 + offsetX, drawPos[1] + cellH / 2 + offsetY, cellH / 2, robotColors[tempRobot->getID()]);
        }
    }

    if (!env.isRunning()) {
        DrawText("SIMULATION PAUSED", 10, 800, 20, RED);
    }

    //render elapsed time in the application
    std::string timeText = "Time: " + std::to_string(t);
    int textWidth = MeasureText(timeText.c_str(), 20);  //get the text width for alignment
    int textHeight = 20;  //text height for alignment
    DrawText(timeText.c_str(), 10, GetScreenHeight() - 2 * textHeight - 10, 20, WHITE);  //left-down corner

    const float dt = env.getStep();
    std::string deltaText = "dt: " + std::to_string(dt);  // \u0394 is the Unicode code for Δ
    int deltaTextWidth = MeasureText(deltaText.c_str(), 20);  //get the text width for alignment
    DrawText(deltaText.c_str(), 10, GetScreenHeight() - textHeight - 10, 20, WHITE);  //render below the time

}
