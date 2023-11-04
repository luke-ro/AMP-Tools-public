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

#include "myPRM2D.h"
#include "myRRT2D.h"
#include "Helpers.h"

const bool RUN_Q1 = false;
const bool RUN_MISC = false;
const bool RUN_GRADER = true;

int main(int argc, char** argv){
    
    if(RUN_Q1){

    }

    if(RUN_GRADER){
        myPRM2D prm(1000, 2.0, false, false);
        myRRT2D rrt(5000, 0.5, 0.05, 0.25, false, false);
        amp::HW7::grade(prm,rrt,"luke.roberson@colorado.edu",argc,argv);
    }

    if(!RUN_GRADER)
        amp::Visualizer::showFigures();

    return 0;
}