// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

#include "myGDAlgo.h"

int main(int argc, char** argv) {
    double epsilon=0.1;
    double dstar_goal=5.0;
    double zeta=1.0;
    double Qstar=0.5;
    double eta=1.0;
    double alpha=0.1;
    myGDAlgo gd(epsilon, dstar_goal, zeta, Qstar, eta, alpha);
    std::cout<<"here"<<"\n";

    // amp::Problem2D prob = amp::HW5::getWorkspace1();
    amp::Problem2D prob = amp::HW2::getWorkspace1();

    amp::Path2D path = gd.plan(prob);

    amp::Visualizer::makeFigure(prob,path);
    amp::Visualizer::showFigures();
}