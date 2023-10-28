// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

#include "time.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW4.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW7.h"

#include "myPRM2D.h"

int main(){
    amp::Problem2D prob_hw5_ws1 = amp::HW5::getWorkspace1();
    amp::Problem2D prob_hw2_ws1 = amp::HW2::getWorkspace1();
    myPRM2D prm;
    amp::Path2D path1 = prm.plan(prob_hw2_ws1);

    amp::Visualizer::makeFigure(prob_hw2_ws1, path1);

    amp::Visualizer::showFigures();
    return 0;
}