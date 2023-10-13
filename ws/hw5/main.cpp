// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

#include "time.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

#include "myGDAlgo.h"

int main(int argc, char** argv) {

    // srand(time(NULL));
    srand(3);

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

    amp::Path2D path = gd.plan(prob);
    amp::HW5::check(path, prob);

    amp::Visualizer::makeFigure(prob_ez,gd.plan(prob_ez));
    amp::Visualizer::makeFigure(prob,path);

    amp::HW5::grade(gd, "luke.roberson@colorado.edu", argc, argv);

    amp::Visualizer::showFigures();
}