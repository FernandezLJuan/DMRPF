#include "environment.h"
#include "robot.h"
#include <libconfig.h++>
#include <unistd.h>

//TODO: ADD KEYBIND TO PLACE ROBOT AT POSITION

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
    int w_width, w_height;
    std::string w_title;

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
    
    cellDims.lookupValue("w", e_cellW);
    cellDims.lookupValue("h", e_cellH);

    gridDims.lookupValue("rows",e_rows);
    gridDims.lookupValue("cols", e_cols);

    origin.lookupValue("x", e_posX);
    origin.lookupValue("y", e_posY);

    std::shared_ptr<Env> grid = std::make_shared<Env>(e_cellW, e_cellH, e_posX, e_posY, e_rows, e_cols, e_robs); /*initializes the environment*/

    GridRenderer renderer(w_width,w_height,e_rows, e_cols);

    SetTraceLogLevel(LOG_NONE);
    InitWindow(w_width, w_height, w_title.c_str());
    SetTargetFPS(60);
    EnableEventWaiting();
    
    int mouseID = 0;
    
    while(!WindowShouldClose()){
        if(!IsWindowMinimized()){

            float deltaTime = GetFrameTime();

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

            BeginDrawing();
            ClearBackground(WHITE);
            renderer.draw(*grid, totalTime);

            if(IsKeyDown(KEY_ENTER)){
                grid->resumeSim();
            }
            if(IsKeyDown(KEY_SPACE)){
                grid->pauseSim();
            }
            if(IsKeyPressed(KEY_R)){
                grid->remakePaths();
            }
            if(IsKeyDown(KEY_UP)){
                timeStep -=0.01f;
            }
            if(IsKeyDown(KEY_DOWN)){
                timeStep += 0.01f;
            }

            EndDrawing();

            usleep(10000);
        }
    }

    CloseWindow();
    return 0;
}
