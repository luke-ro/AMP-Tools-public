// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "CSpace2D.h"

// using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /*** Q1 ***/
    CSpace2D cspace(-5.0,5.0,-5.0,5.0);
    amp::Polygon triang = amp::HW4::getEx1TriangleObstacle();
    amp::Polygon C_triang = cspace.minkDiff(triang,triang);
    for (auto vert : C_triang.verticesCCW()){
        std::cout << vert[0]<< ", " << vert[1] << "\n";
    }

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "luke.roberson@colorado.edu", argc, argv);
    return 0;
}