// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "CSpace2D.h"
#include "Arm2L.h"

// using namespace amp;

amp::Polygon rotatePG(amp::Polygon pg, double angle, Eigen::Vector2d point){
    Eigen::Matrix3d R;
    Eigen::Matrix3d T1;
    Eigen::Matrix3d T2;

    T1 << 1,  0,  -point[0],
          0,  1,  -point[1],
          0,  0,  1;

    R <<  cos(angle), -sin(angle),  0,
          sin(angle),  cos(angle),  0,
          0,           0,           1;


    T2 << 1,  0,  point[0],
          0,  1,  point[1],
          0,  0,  1;
    
    Eigen::Matrix3d T = T2*R*T1;
    
    std::vector<Eigen::Vector2d> verts;
    for(auto vert: pg.verticesCCW()){
        Eigen::Vector3d u;
        u << vert[0], vert[1], 1;
        Eigen::Vector3d v = T*u;
        Eigen::Vector2d w;
        w << v[0], v[1];
        verts.push_back(w);
    }

    return amp::Polygon(verts);
}

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


    /*** Q2 ***/
    Eigen::Vector2d zero {0.0,0.0};
    std::vector<amp::Polygon> pgs;
    std::vector<double> heights;
    amp::Polygon temp;
    int n= 12;
    for(int i=0; i<=n; i++){
        temp = rotatePG(triang, 2.0*3.1415/double(n)*double(i), zero);
        pgs.push_back(cspace.minkDiff(triang,temp));
        heights.push_back(0.2*double(i));
    }
    amp::Visualizer::makeFigure(pgs,heights);

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "luke.roberson@colorado.edu", argc, argv);
    amp::Visualizer::showFigures();
    return 0;
}