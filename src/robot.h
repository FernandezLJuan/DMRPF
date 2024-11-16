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
        /* path.push_back(currentCell);
        pathHistory.push_back(currentCell); */
        this->updateDetectionArea();

        leader = nullptr;
        follower = nullptr;

        giveWayNode = nullptr;
        freeNeighboringNode = nullptr;

        givingWay = false;
        done = false;
        noConflictDetected = true;

        plannedAction = 1;
        givenWay = 0; /*times the robot has given his way to another robot*/
        pathLength = 0;
        nodeRequests = 0;
        
        ruleCount = {0,0,0,0,0,0};
        conflictCount = {0,0};
    }

    static void resetID(){
        currentID = 0;
    }

    bool atGoal();                       // Is the robot at its goal?
    bool isMoving();                     // Is the robot currently moving?
    bool isGivingWay();                  // Is the robot giving way to another?
    bool isInFollowerChain(Robot*);      // Is the given robot in the follower chain?
    bool isInPath(Cell*);                // Is a given cell part of the robot's path?

    void stopRobot();                    // Stop the robot
    void resumeRobot();                  // Resume the robot's movement
    Cell* step();                        // Get the next step in the path
    void retreat();                      // Move the robot backward
    void giveWay();                      // Allow another robot to proceed

    void resetPath();                    // resets pathHistory and pathLength, then remakesPath
    void generatePath();                 // Generate a path between two positions
    void reconstructPath(std::unordered_map<Cell*, Cell*>, Cell*); // Reconstruct a path from a map
    void insertCell(Cell* c);
    size_t relativePathSize();           // Compare initial path to history
    const std::vector<Cell*>& getPath(); // Get the robot's current path
    void logPath();                      // Log the path
    void takeAction();

    Cell* getGoal();                     // get the current goal
    void setGoal(Cell*);                 // set a goal for the robot
    void removeGoal();                   // remove the current goal
    bool findGiveWayNode();              // find a node to give way

    void setFollower(Robot*);               // set the robot's follower
    int getNFollowers();                    // number of followers
    void findFollowers();                   // identify the robot's followers
    void solveIntersectionConflict(Robot*); // resolve a conflict at an intersection
    void solveOppositeConflict(Robot*);     // resolve a conflict with an opposing robot
    Robot* determinePriority(Robot*, Robot*); // Determine priority between robots

    void updateDetectionArea();          // update the detection area of the robot
    const std::vector<Cell*>& getArea(); // get the detection area (used for drawing)
    void findNeighbors();                 // update neighbors from the environment
    int getNeighborsRequestingNode();    // count how many neighbors are requesting this node
    void fetchNeighborInfo();            // fetch information from neighboring robots
    const std::vector<Robot*>& getNeighbors();

    int getID();                         // get the robot's ID
    std::array<int, 2> getPos();         // get the robot's position on the grid
    Cell* getCurrentCell();              // get the current cell
    const std::array<int, 6> getRuleCount(); // get activation count for rules
    const std::array<int, 2> getConflictCount(); //get detection count for conflicts
    int howManyGiveWays();
    size_t totalRequests();          // returns average of nodeRequests per time step
    void logPos();                       // log the robot's position

private:

    static int assignID(){
        return ++currentID;
    }

    static int currentID;

    int id, numberFollowers, plannedAction, givenWay;
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

    size_t nodeRequests; /*number of neighbors requesting node, updated on each call to findFollowers*/
    
    std::array<int, 6> ruleCount; /*counts, locally, how many times each rule has been triggered*/
    std::array<int, 2> conflictCount; /*counts, locally, how many times each conflict type has been detected*/

    Robot* leader; /*pointer to leader of the chain*/
    Robot* follower; /*pointer to the nearest follower*/
};
