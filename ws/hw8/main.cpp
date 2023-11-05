// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

#include "time.h"
#include <chrono>

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW4.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW7.h"
#include "hw/HW8.h"

#include "myPRM2D.h"
#include "myRRT2D.h"
#include "myCSpace2d.h"
#include "Helpers.h"

const bool RUN_Q1 = false;
const bool RUN_MISC = true;
const bool RUN_GRADER = false;

int main(int argc, char** argv){
    
    if(RUN_Q1){
        

    }

    if(RUN_MISC){
        // test of cspace constructor
        amp::Environment2D env_hw8_ws1 = amp::HW8::getWorkspace1();
        myCSpace2d cspace(100,100,0,10,0,10);
        amp::CircularAgentProperties agent_props;
        cspace.constructFromCircleAgent(env_hw8_ws1,agent_props);
        amp::Visualizer::makeFigure(cspace);
        amp::Visualizer::makeFigure(env_hw8_ws1);

    }

    if(RUN_GRADER){

    }


    if(!RUN_GRADER)
        amp::Visualizer::showFigures();

    return 0;
}