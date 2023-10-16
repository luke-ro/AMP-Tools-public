// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

#include "time.h"

// Include the correct homework header
#include "hw/HW6.h"
#include "hw/HW5.h"
#include "hw/HW2.h"

#include "myWaveFront.h"
#include "CSpace2D.h"
#include "Helpers.h"

int main(int argc, char** argv) {
    myWaveFront wf;
    auto grid_ptr = wf.constructDiscretizedWorkspace(amp::HW2::getWorkspace2());
    amp::Visualizer::makeFigure(amp::HW2::getWorkspace2());
    amp::Visualizer::makeFigure(*grid_ptr);
    amp::Visualizer::showFigures();
    return 0;
}