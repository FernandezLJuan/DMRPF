#pragma once
#include "environment.h"
#include <queue>
#include <unordered_set>

class Env;
class Cell;

class Robot{
public:

    Robot(int x, int y,Env* env): id(assignID()), posX(x), posY(y), environment(env){
        currentCell = environment->getCellByPos(posX, posY);
        this->updateDetectionArea();
    }

    void generatePath(); /*generate path between two positions*/

    bool isMoving();
    void stopRobot();
    void resumeRobot();

    void updateDetectionArea();
    void takeAction();
    std::shared_ptr<Cell> step();/*next step in path*/

    void anyoneThere(); /*placeholder name, no se como llamarle*/

    /*GETTERS AND SETTERS*/
    int getID();
    std::array<int, 2> getPos(); /*get x,y position on the grid (is it redundant?)*/
    std::shared_ptr<Cell> getCurrentCell();
    std::vector<std::shared_ptr<Cell>> getArea(); /*retyurn detection area, used for drawing*/
    void logPos();
    void setGoal(Vector2);/*set goal at x,y position*/

private:

    static int assignID(){
        static int currentID = 0;
        return ++currentID;
    }

    int id;
    int numberFollowers;
    float posX, posY; /*do I really need these anymore?*/

    bool moving = true; /*in case robot needs to be stopped before arriving at goal*/

    void move(std::shared_ptr<Cell>); /*move to a new position, only one cell at a time*/
    void followPath(); /*follow generated path*/

    const int detectionRadius = 2; /*radius in which the robot can detect another robot*/
    const float waitProbability = 0.05f; /*probability of the robot not moving this time step*/

    std::shared_ptr<Cell> goal; /*where does the robot want to move to*/
    std::shared_ptr<Cell> giveWayNode; /*node to move back and give way to a robot of more priority*/

    Env* environment; /*environment the robot is on*/

    std::shared_ptr<Cell> currentCell; /*in which cell is the robot currently?*/
    std::queue<std::shared_ptr<Cell>> path; /*path the robot follows when moving*/
    std::vector<std::shared_ptr<Cell>> detectionArea; /*cells in which the robot can detect robots*/
    std::vector<Robot*> neighborsRequestingNode; /*robots requesting the current node*/
};
