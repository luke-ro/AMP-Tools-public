// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "CSpace2D.h"
#include "Arm2L.h"
#include "MyGridCon.h"
#include "Rotate.h"

// using namespace amp;
const double pi = 3.1415;

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

    // stuff to show the vertices of the cspace obstacle for q1
    std::vector<amp::Polygon> triang_vec;
    triang_vec.push_back(C_triang);
    amp::Visualizer::makeFigure(triang_vec,0.0);


    /*** 1b ***/
    Eigen::Vector2d zero {0.0,0.0};
    std::vector<amp::Polygon> pgs;
    std::vector<double> heights;
    amp::Polygon temp;
    int n= 12;
    for(int i=0; i<=n; i++){
        temp = Rotate::rotatePG(triang, 2.0*3.1415/double(n)*double(i), zero);
        pgs.push_back(cspace.minkDiff(triang,temp));
        heights.push_back(0.2*double(i));
    }
    amp::Visualizer::makeFigure(pgs,heights);

    /*** 2a ***/
    Eigen::Vector2d base {0,0};
    Eigen::Vector3d state_2a = {pi/6, pi/3, 7*pi/4};
    std::vector<double> lengths_2a = {0.5, 1.0, 0.5};
    Arm2L manip_2a(base,lengths_2a);
    amp::Visualizer::makeFigure(manip_2a, state_2a);

    /*** 2b ***/
    std::vector<double> lengths_2b = {1.0, 0.5, 1.0};
    Eigen::Vector2d end_eff_2b {2,0};
    Arm2L manip_2b(base,lengths_2b);
    amp::ManipulatorState state_2b = manip_2b.getConfigurationFromIK(end_eff_2b);
    amp::Visualizer::makeFigure(manip_2b, state_2b);



    /*** 3 ***/
    std::vector<double> lengths_3 = {1.0, 1.0};
    Arm2L manip_3(base, lengths_3);
    CSpace2D gridcon(0, 2*pi, 0, 2*pi, 100, 100);

    amp::Visualizer::makeFigure(gridcon.genCSpace(manip_3,amp::HW4::getEx3Workspace1()));

    amp::Visualizer::makeFigure(gridcon.genCSpace(manip_3,amp::HW4::getEx3Workspace2()));
    
    amp::Visualizer::makeFigure(gridcon.genCSpace(manip_3,amp::HW4::getEx3Workspace3()));
    
    // Grade method
    // MyGridCon* gridcon_grade;
    // amp::HW4::grade<Arm2L>(*gridcon_grade, "luke.roberson@colorado.edu", argc, argv);
    amp::Visualizer::showFigures();
    return 0;
}