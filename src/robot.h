#pragma once
#include "environment.h"
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
    void findLeader();
    void solveIntersectionConflict(Robot&);
    void giveWay();

    void updateDetectionArea();
    void getGiveWayNode();
    void takeAction();
    std::shared_ptr<Cell> step();/*next step in path*/

    void fetchNeighborInfo();

    /*GETTERS AND SETTERS*/
    int getID();
    int getNFollowers();
    std::array<int, 2> getPos(); /*get x,y position on the grid (is it redundant?)*/
    std::shared_ptr<Cell> getCurrentCell();
    std::vector<std::shared_ptr<Cell>> getArea(); /*retyurn detection area, used for drawing*/
    std::shared_ptr<Cell> getGoal();
    std::vector<std::shared_ptr<Cell>> getPath();
    void setPos(std::shared_ptr<Cell>);
    void logPos();
    void setGoal(std::shared_ptr<Cell>);/*set goal at x,y position*/
    void removeGoal();
    void setLeader(Robot*);
    void setFollower(Robot*);

private:

    static int assignID(){
        return ++currentID;
    }

    static int currentID;

    int id;
    int numberFollowers;
    int lastAction = 0; /*waited = 0, moved = 1*/
    int nextAction = 0; /*wait = 0, move = 1, give way = 2*/

    bool moving = true; /*in case robot needs to be stopped before arriving at goal*/
    

    void move(std::shared_ptr<Cell>); /*move to a new position, only one cell at a time*/
    void followPath(); /*follow generated path*/

    const int detectionRadius = 2; /*radius in which the robot can detect another robot*/
    const float waitProbability = 0.00f; /*probability of the robot not moving this time step*/

    std::shared_ptr<Cell> goal; /*where does the robot want to move to*/
    std::shared_ptr<Cell> giveWayNode; /*node to move back and give way to a robot of more priority*/

    Env* environment; /*environment the robot is on*/

    std::shared_ptr<Cell> currentCell; /*in which cell is the robot currently?*/
    std::shared_ptr<Cell> lastCell; /*last cell the robot has been in*/
    std::vector<std::shared_ptr<Cell>> path; /*path the robot follows when moving*/
    std::vector<std::shared_ptr<Cell>> pathHistory;
    std::vector<std::shared_ptr<Cell>> detectionArea; /*cells in which the robot can detect robots*/
    std::vector<Robot*> neighborsRequestingNode; /*robots requesting the current node*/
    std::vector<Robot*> neighbors; /*robots inside detection area*/

    Robot* leader; /*pointer to leader of the chain*/
    Robot* follower = nullptr; /*pointer to the nearest follower*/
};
