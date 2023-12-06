// This is the bare min to get a cspace grid. 

#pragma once

#include "AMPCore.h"

#include "Helpers.h"

class myCSpace2d : public amp::GridCSpace2D{
    public:

    myCSpace2d(int s0, int s1, double x0_min, double x0_max, double x1_min, double x1_max):
        GridCSpace2D(s0, s1, x0_min, x0_max, x1_min, x1_max),
        _n0(s0),
        _n1(s1)
    {}

    virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

    bool freeBtwPoints(Eigen::Vector2d p1, Eigen::Vector2d p2);


    void constructFromCircleAgent(const amp::Environment2D& env, double radius);

    int _n0;
    int _n1;
};
