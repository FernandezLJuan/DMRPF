#include "environment.h"
#include "robot.h"
#include <libconfig.h++>
#include <unistd.h>

//TODO: FIND OUT WHY LOOPS APPEAR ON SOME ROBOTS AFTER SOLVING A CONFLICT
//TODO: SOLVE LOOPS IN GIVE WAY TO LEADER AND GIVE WAY IN CONFLICT


float totalTime = 0.0f;
float timeElapsed = 0.0f;
float timeStep = 0.1f;

void loadSettings(std::string path){
/*load settings from file configuration at path*/
    libconfig::Config cfg;

    cfg.readFile(path.c_str());
}

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

int main(int argc, char* argv[]){

    /*READ SETTINGS FILE*/
    std::string configFile = "default.cfg";

    if(argc>1){
        configFile = argv[2];
    }

    /*settings for window and environment*/
    int e_cellW, e_cellH, e_posX, e_posY, e_rows, e_cols, e_robs;
    float e_obsProb;
    int w_width, w_height;
    std::string w_title;
    std::string e_map;

    libconfig::Config cfg;
    cfg.readFile(configFile.c_str());

    /*window settings*/
    const libconfig::Setting& root = cfg.getRoot();
    const libconfig::Setting& window = root["application"]["window"];
    const libconfig::Setting& size = window["size"];

    window.lookupValue("title", w_title);
    size.lookupValue("w", w_width);
    size.lookupValue("h", w_height);

    /*environment settings*/
    const libconfig::Setting& environment = root["application"]["environment"];
    const libconfig::Setting& cellDims = environment["cellDims"];
    const libconfig::Setting& gridDims = environment["gridDims"];
    const libconfig::Setting& origin = environment["origin"];
    
    environment.lookupValue("n_robots", e_robs);
    environment.lookupValue("obsProb", e_obsProb);
    environment.lookupValue("map", e_map);
    
    cellDims.lookupValue("w", e_cellW);
    cellDims.lookupValue("h", e_cellH);

    gridDims.lookupValue("rows",e_rows);
    gridDims.lookupValue("cols", e_cols);

    origin.lookupValue("x", e_posX);
    origin.lookupValue("y", e_posY);

    std::shared_ptr<Env> grid = std::make_shared<Env>(e_cellW, e_cellH, e_posX, e_posY, e_rows, e_cols, e_robs, e_obsProb); /*initializes the environment*/

    if(grid->load_map(e_map)<0){
        grid->randomGrid(); /*if the map file was invalid, randomize everything*/
    }

    GridRenderer renderer(e_cellW,e_cellH,e_rows, e_cols);
    renderer.cacheRobotColors(e_robs*2);

    Color bgColor = {0x36,0x39,0x3e,255};

    SetTraceLogLevel(LOG_NONE);
    InitWindow(w_width, w_height, w_title.c_str());
    SetTargetFPS(30);
    EnableEventWaiting();
    
    int mouseID = 0;

    while(!WindowShouldClose()){
        if(!IsWindowMinimized()){

            float deltaTime = GetFrameTime();

            BeginDrawing();

            ClearBackground(bgColor);
            renderer.draw(*grid, totalTime);

            if(grid->isRunning()){
                timeElapsed += deltaTime;
                totalTime += deltaTime;
            }
            
            Vector2 mousePos = GetMousePosition();
            mouseID = posToID(*grid,mousePos);

            grid->onClick(mouseID);

            if(timeElapsed>=timeStep){
                grid->updateEnvironment();
                timeElapsed = 0.0;
            }

            if(IsKeyDown(KEY_UP)){
                if(timeStep>0.01f)
                    timeStep -=0.01f;
            }
            if(IsKeyDown(KEY_DOWN)){
                if(timeStep<1.0f)
                    timeStep += 0.01f;
            }

            EndDrawing();
            usleep(10000);
        }
    }

    CloseWindow();
    return 0;
}
