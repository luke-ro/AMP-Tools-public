// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

#include "time.h"

// Include the correct homework header
#include "hw/HW6.h"
#include "hw/HW5.h"
#include "hw/HW2.h"

#include "myWFPoint.h"
#include "CSpace2D.h"
#include "Helpers.h"
#include "time.h"

int main(int argc, char** argv) {
    //test wavefron
    myWFPoint wf;
    // srand(time(NULL));
    int seed = time(NULL); //failed 1697687252
    std::cout<<"seed: "<< seed<<"\n";
    amp::Problem2D ws =  amp::HW2::getWorkspace2();
    auto grid_ptr = wf.constructDiscretizedWorkspace(ws);
    amp::Path2D wf_path;
    wf_path =  wf.planInCSpace(ws.q_init, ws.q_goal, *grid_ptr);
    std::cout<<"Path distance for wavefront on HW2 WS2: "<< wf_path.length()<<"\n";

    amp::Problem2D rand_prob;
    amp::Path2D rand_path;
    amp::HW6::generateAndCheck(wf,rand_path, rand_prob,true,seed);

    //make plots
    amp::Visualizer::makeFigure(ws,wf_path);
    amp::Visualizer::makeFigure(*grid_ptr);
    amp::Visualizer::makeFigure(rand_prob, rand_path);
    amp::Visualizer::showFigures();
    return 0;
}