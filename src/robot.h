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
    Robot(std::shared_ptr<Cell> cell,Env* env): id(assignID()), environment(env), currentCell(cell){
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
    void stopRobot();
    void resumeRobot();
    void reconstructPath(std::unordered_map<std::shared_ptr<Cell>, std::shared_ptr<Cell>>, std::shared_ptr<Cell>);
    void findFollowers();
    bool isInFollowerChain(Robot*);
    void giveWay();
    void retreat();
    void getNeighbors();
    void solveIntersectionConflict(Robot*);
    void solveOppositeConflict(Robot*);

    Robot* determinePriority(Robot*, Robot*);

    void updateDetectionArea();
    bool findGiveWayNode();
    bool isInPath(std::shared_ptr<Cell>);
    void takeAction();
    std::shared_ptr<Cell> step();/*next step in path*/

    void fetchNeighborInfo();

    /*GETTERS AND SETTERS*/
    int getID();
    int getNFollowers();
    int getNeighborsRequestingNode(); /*return how many neighbors are requesting this node*/
    size_t relativePathSize(); /*returns the relation between the first generated path and the path history*/
    std::array<int, 2> getPos(); /*get x,y position on the grid (is it redundant?)*/
    std::shared_ptr<Cell> getCurrentCell();
    std::vector<std::shared_ptr<Cell>> getArea(); /*retyurn detection area, used for drawing*/
    std::shared_ptr<Cell> getGoal();
    std::vector<std::shared_ptr<Cell>> getPath();
    std::array<int, 6> getRuleCount();
    void setPos(std::shared_ptr<Cell>);
    void logPos();
    void setGoal(std::shared_ptr<Cell>);/*set goal at x,y position*/
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

    int id;
    int numberFollowers;
    int plannedAction;
    bool noConflictDetected;
    bool givingWay;
    bool done;

    size_t pathLength;

    void move(std::shared_ptr<Cell>); /*move to a new position, only one cell at a time*/
    void followPath(); /*follow generated path*/

    const int detectionRadius = 2; /*radius in which the robot can detect another robot*/
    const float waitProbability = 0.00f; /*probability of the robot not moving this time step*/

    std::shared_ptr<Cell> goal; /*where does the robot want to move to*/
    std::shared_ptr<Cell> giveWayNode; /*node to move back and give way to a robot of more priority*/
    std::shared_ptr<Cell> freeNeighboringNode;

    Env* environment; /*environment the robot is on*/

    std::shared_ptr<Cell> currentCell; /*in which cell is the robot currently?*/
    std::shared_ptr<Cell> lastCell; /*last cell the robot has been in*/
    std::vector<std::shared_ptr<Cell>> path; /*path the robot follows when moving*/
    std::vector<std::shared_ptr<Cell>> pathHistory;
    std::vector<std::shared_ptr<Cell>> detectionArea; /*cells in which the robot can detect robots*/
    std::vector<Robot*> neighborsRequestingNode; /*robots requesting the current node*/
    std::vector<Robot*> neighbors; /*robots inside detection area*/
    std::array<int, 6> ruleCount; /*counts, locally, how many times each rule has been triggered*/

    Robot* leader; /*pointer to leader of the chain*/
    Robot* follower; /*pointer to the nearest follower*/
};
