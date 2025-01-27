#include "environment.h"
#include "robot.h"
#include <libconfig.h++>
#include <unistd.h>
#include <chrono>

float totalTime = 0.0f;
float timeElapsed = 0.0f;
float timeStep = 0.1f;

int posToID(Env& env, Vector2 mousePos){
    /*translate mouse position to cell ID in environment based on dimensions*/

    std::array<int,2> cDims = env.cellDims();
    std::array<int,2> origin = env.origin();
    std::array<int,2> eDims = env.getDims();

    int gridX = (mousePos.x - origin[0])/cDims[0];
    int gridY = (mousePos.y - origin[1])/cDims[1];

    int cellID = gridY * eDims[1] + gridX;

    return cellID;
}

void loadWindowSettings(const libconfig::Config& cfg, std::string& title, int& width, int& height) {
    const libconfig::Setting& root = cfg.getRoot();
    const libconfig::Setting& window = root["application"]["window"];
    const libconfig::Setting& size = window["size"];
    
    window.lookupValue("title", title);
    size.lookupValue("w", width);
    size.lookupValue("h", height);
}

void loadEnvironmentSettings(const libconfig::Config& cfg, int& cellW, int& cellH, int& posX, int& posY, 
                             int& rows, int& cols, int& numRobots, float& obsProb, float& dynProb, 
                             float& waitProb, std::string& map) {
    const libconfig::Setting& root = cfg.getRoot();
    const libconfig::Setting& environment = root["application"]["environment"];
    const libconfig::Setting& cellDims = environment["cellDims"];
    const libconfig::Setting& gridDims = environment["gridDims"];
    const libconfig::Setting& origin = environment["origin"];
    
    environment.lookupValue("n_robots", numRobots);
    environment.lookupValue("obsProb", obsProb);
    environment.lookupValue("dynamicProb", dynProb);
    environment.lookupValue("waitProbability", waitProb);
    environment.lookupValue("map", map);

    //default map name if empty
    if(map == "") {
        map = "Covelo";
    }

    cellDims.lookupValue("w", cellW);
    cellDims.lookupValue("h", cellH);
    gridDims.lookupValue("rows", rows);
    gridDims.lookupValue("cols", cols);
    origin.lookupValue("x", posX);
    origin.lookupValue("y", posY);
}

int main(int argc, char* argv[]){

    int numSims = 0; /*number of simulations ran*/
    const int maxSims = (argc>1) ? std::stoi(argv[1]) : 25; /*number of simulations to run, default is 25*/

    /*READ SETTINGS FILE*/
    std::string configFile = "default.cfg";

    int e_cellW, e_cellH, e_posX, e_posY, e_rows, e_cols, e_robs;
    float e_obsProb, e_dynProb, e_waitProb;
    int w_width, w_height;
    std::string w_title, e_map;

    libconfig::Config cfg;
    cfg.readFile(configFile.c_str());

    /*load window settings*/
    loadWindowSettings(cfg, w_title, w_width, w_height);

    /*load environment settings*/
    loadEnvironmentSettings(cfg, e_cellW, e_cellH, e_posX, e_posY, e_rows, e_cols, e_robs, e_obsProb, e_dynProb, e_waitProb, e_map);

    /*initialize the environment*/
    Env grid(e_cellW, e_cellH, e_posX, e_posY, e_rows, e_cols, e_robs, timeStep, e_obsProb, e_dynProb, e_waitProb, e_map);


    /*if the map could not be loaded, randomize the environment*/
    if(grid.load_benchmark()<0){
        grid.randomGrid(); /*if the map file was invalid, randomize everything*/
    }

    /*pause the simulation at the start*/
    grid.pauseSim();

    /*initialize the renderer and cache colors for double the required robots, this saves overhead of color calculation every frame*/
    GridRenderer renderer(e_cellW,e_cellH,e_rows, e_cols);
    renderer.cacheRobotColors(e_robs*2);

    /*define the background color for the simulation*/
    Color bgColor = {0x36,0x39,0x3e,255};

    /*initialize the window*/
    SetTraceLogLevel(LOG_NONE);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(w_width, w_height, w_title.c_str());
    
    /*id of the cell the mouse is over*/
    int mouseID = 0;

    while(!WindowShouldClose() && numSims<maxSims){

        /*capture time between frames*/
        float deltaTime = GetFrameTime();

        /*start rendering*/
        BeginDrawing();

        ClearBackground(bgColor);
        renderer.draw(grid, totalTime);

        if(grid.isRunning()){
            timeElapsed += deltaTime;
            totalTime += deltaTime;
        }
        
        /*get position of the cursor*/
        Vector2 mousePos = GetMousePosition();
        mouseID = posToID(grid,mousePos);

        grid.handle_input(mouseID);

        /*each time step, update the environment*/
        if(timeElapsed>=timeStep){
            /*if updateEnvironment returns true, the simulation has ended*/
            if (grid.updateEnvironment(totalTime)){
                numSims++;
                totalTime = 0.0; /*reset total runtime time, this ensures proper total_time calculation each simulation*/

                /*this condition avoids a re-planification at the end of the last simulation, saving time on large fleets*/
                if(numSims<maxSims){
                    grid.clearTransients();
                    grid.randomizeRobots();
                    grid.resumeSim();
                }
            }
            timeElapsed = 0.0;
        }

        EndDrawing();
        usleep(1000);
    }

    std::cout<<"RAN "<<numSims<<" SIMULATIONS "<<std::endl;

    CloseWindow();
    return 0;
}
