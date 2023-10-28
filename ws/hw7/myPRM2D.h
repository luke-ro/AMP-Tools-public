#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"

#include "myAStar.h"
#include "Helpers.h"

class myPRM2D : public amp::PRM2D{
    public:

    myPRM2D(int N=1000):_N_MAX(N),_neigh_radius(1.0){}

    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    private:

    const int _N_MAX;
    const double _neigh_radius;
};