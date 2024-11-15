#pragma once
#include "environment.h"
#include <format>
#include <stack>
#include <unordered_set>

class Env;
class Cell;

class Robot
{
public:
    Robot(Cell* cell,Env* env): id(assignID()), environment(env), currentCell(cell){
        lastCell = currentCell;
        path.push_back(currentCell);
        pathHistory.push_back(currentCell);
        this->updateDetectionArea();

        leader = nullptr;
        follower = nullptr;

        giveWayNode = nullptr;
        freeNeighboringNode = nullptr;

        givingWay = false;
        done = false;

        plannedAction = 1;
        noConflictDetected = true;
        
        ruleCount = {0,0,0,0,0,0};
    }

    static void resetID(){
        currentID = 0;
    }

    void generatePath(); /*generate path between two positions*/

    bool atGoal(); /*is the robot at goal?*/
    bool isMoving();
    bool isGivingWay();
    bool isInFollowerChain(Robot*);
    void stopRobot();
    void resumeRobot();
    void reconstructPath(std::unordered_map<Cell*, Cell*>, Cell*);
    void findFollowers();
    void giveWay();
    void retreat();
    void getNeighbors();
    void solveIntersectionConflict(Robot*);
    void solveOppositeConflict(Robot*);

    Robot* determinePriority(Robot*, Robot*);

    void updateDetectionArea();
    bool findGiveWayNode();
    bool isInPath(Cell*);
    void takeAction();
    Cell* step();/*next step in path*/

    void fetchNeighborInfo();

    /*GETTERS AND SETTERS*/
    int getID();
    int getNFollowers();
    int getNeighborsRequestingNode(); /*return how many neighbors are requesting this node*/
    size_t relativePathSize(); /*returns the relation between the first generated path and the path history*/
    std::array<int, 2> getPos(); /*get x,y position on the grid (is it redundant?)*/
    Cell* getCurrentCell();
    const std::vector<Cell*>& getArea(); /*retyurn detection area, used for drawing*/
    const std::vector<Cell*>& getPath();
    Cell* getGoal();
    std::array<int, 6> getRuleCount();
    void logPos();
    void setGoal(Cell*);/*set goal at x,y position*/
    void removeGoal();
    void setLeader(Robot*);
    void setFollower(Robot*);
    void logPath();
    Robot* getLeader();

private:

    static int assignID(){
        return ++currentID;
    }

    static int currentID;

    int id, numberFollowers, plannedAction;
    bool noConflictDetected, givingWay, done;

    size_t pathLength;

    void move(Cell*); /*move to a new position, only one cell at a time*/
    void followPath(); /*follow generated path*/

    const int detectionRadius = 2; /*radius in which the robot can detect another robot*/
    const float waitProbability = 0.00f; /*probability of the robot not moving this time step*/

    Cell* goal; /*where does the robot want to move to*/
    Cell* giveWayNode; /*node to move back and give way to a robot of more priority*/
    Cell* freeNeighboringNode;

    Env* environment; /*environment the robot is on*/

    Cell* currentCell; /*in which cell is the robot currently?*/
    Cell* lastCell; /*last cell the robot has been in*/
    std::vector<Cell*> path; /*path the robot follows when moving*/
    std::vector<Cell*> pathHistory;
    std::vector<Cell*> detectionArea; /*cells in which the robot can detect robots*/
    std::vector<Robot*> neighborsRequestingNode; /*robots requesting the current node*/
    std::vector<Robot*> neighbors; /*robots inside detection area*/
    std::array<int, 6> ruleCount; /*counts, locally, how many times each rule has been triggered*/

    Robot* leader; /*pointer to leader of the chain*/
    Robot* follower; /*pointer to the nearest follower*/
};
