// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

#include "time.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

#include "myGDAlgo.h"
#include "Helpers.h"

int main(int argc, char** argv) {

    srand(time(NULL));
    // srand(3);


    double epsilon=0.1;
    double dstar_goal=3.0;
    double zeta=1.0;
    double Qstar=3.0;
    double eta=.3;
    double alpha=0.1;
    myGDAlgo gd(epsilon, dstar_goal, zeta, Qstar, eta, alpha);
    std::cout<<"here"<<"\n";

    amp::Problem2D prob_ez = amp::HW5::getWorkspace1();
    amp::Problem2D prob = amp::HW2::getWorkspace1();

    amp::Path2D path_ez = gd.plan(prob_ez);
    amp::Visualizer::makeFigure(prob_ez,path_ez);
    std::cout<<"path_ez length: "<< H::pathDistane(path_ez,0,path_ez.waypoints.size()-1)<<"\n";

    amp::Path2D path_hw2ws2 = gd.plan(amp::HW2::getWorkspace2());
    std::cout<<"path_hw2ws2 length: "<< H::pathDistane(path_hw2ws2,0,path_hw2ws2.waypoints.size()-1)<<"\n";
    amp::Visualizer::makeFigure(amp::HW2::getWorkspace2(),path_hw2ws2);

    amp::Path2D path = gd.plan(prob);
    amp::HW5::check(path, prob);
    amp::Visualizer::makeFigure(prob,path);
    std::cout<<"path_hw2ws1 length: "<< H::pathDistane(path,0,path.waypoints.size()-1)<<"\n";

    // amp::HW5::grade(gd, "luke.roberson@colorado.edu", argc, argv);

    amp::Visualizer::showFigures();
}