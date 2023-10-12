#include "myGDAlgo.h"

myGDAlgo::myGDAlgo(int sx0, int sx1)
 :amp::GDAlgorithm()
 ,_sx0 (sx0)
 ,_sx1 (sx1)
 {}

amp::Path2D myGDAlgo::plan(const amp::Problem2D& problem){
    std::vector< std::vector<Eigen::Vector2d> > poten (_sx0,
        std::vector<Eigen::Vector2d>(_sx1));
    
    // fill potential func with base potential

    // for each point calculate the repulsive force and 
    // add on top of base potential

    // calculate gradient based off of potential? 
    return amp::Path2D();
}